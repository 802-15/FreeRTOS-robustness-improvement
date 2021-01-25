/*
 * Kalman filtering application designed for testing redundancy
 * Copyright © 2020, Mario Senecic
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * https://github.com/802-15/FreeRTOS-robustness-improvement
 *
 */

#include "application.h"


/* FreeRTOS handles */
static TaskHandle_t filter_task = NULL;
static TaskHandle_t print_task = NULL;
static TimerHandle_t measurement_timer = NULL;
static QueueHandle_t measurement_queue = NULL;
static QueueHandle_t results_queue = NULL;

/* Measurement data */
static BaseType_t measurement_count = 1;

/* Filter state restoration and cleanup structures */
kalman_handle_t filter_storage = {0};
kalman_state_t * kalman_state = NULL;

/* Default filter values */
vector_t default_x_state = {0};
vector_t default_y_state = {0};
vector_t default_x_cov = {0};
vector_t default_y_cov = {0};

static void displacement_data_update(TimerHandle_t xTimer)
{
    /* Load data from the constant measurements array and store it
     * to the queue accessible to the filtering task. Timer period
     * corresponds to the sample time of the system in consideration,
     * and the application will fail if the filter task has not cleared
     * the measurement by the next measurement.
     */
    (void) xTimer;
    measurement_t measurement_struct = {0};

    TASK_4_START

    /* Display the results if final measurement was reached */
    if (measurement_count >= MEASUREMENTS) {
        xTimerStop(xTimer, 0);
        vTaskSuspend(filter_task);
        vTaskResume(print_task);
        gpio_led_state(LED6_BLUE_ID, 1);
        return;
    }

    /* Obtain the measurement by reading the values and sending them to the measurement queue */
    measurement_struct.x_value = x_displacement[measurement_count];
    measurement_struct.y_value = y_displacement[measurement_count];
    measurement_count++;

    /* Block if the queue was not emptied by the filtering task */
    if(uxQueueMessagesWaiting(measurement_queue) == 1)
    {
        for(;;);
    }
    xQueueSendToBack(measurement_queue, &measurement_struct, 0);

    /* Reset the timer and toggle the PIN for tracing purposes */
    TASK_4_FINISH;
    xTimerReset(xTimer, 0);
    vTaskResume(filter_task);
}

static void filtering_task_function(void *pvParameters)
{
    /* Initialize two single-axis kinematic kalman filters used to
     * smooth out the 'measured' displacement data. This task executes
     * configTIME_REDUNDANT_INSTANCES times seemingly in parallel.
     */
    BaseType_t error_code = 0;
    BaseType_t instance_number = 0;
    kf_t * x_kalman_filter = NULL;
    kf_t * y_kalman_filter = NULL;
    kalman_state_t * filter_state = NULL;
    measurement_t measurement_struct = {0};
    result_t result_struct = {0};

    instance_number = xTaskGetInstanceNumber();

    /* Initialize two simple kalman filters for 2 axis displacement filtering */
    error_code = kalman_create(&x_kalman_filter, 0.1);
    if (error_code != KALMAN_OK) {
        while(1);
    }

    error_code = kalman_create(&y_kalman_filter, 0.2);
    if (error_code != KALMAN_OK) {
        while(1);
    }

    filter_storage.x_filters[instance_number] = x_kalman_filter;
    filter_storage.y_filters[instance_number] = y_kalman_filter;

    if (!pvParameters) {
        /* Initialize filter to default values */
        kalman_init(&x_kalman_filter->xp, &x_kalman_filter->Pp, &default_x_state, &default_x_cov);
        kalman_init(&y_kalman_filter->xp, &y_kalman_filter->Pp, &default_y_state, &default_y_cov);
    } else {
        /* Restore the filter state if the task was reset */
        filter_state = (kalman_state_t * ) pvParameters;
        kalman_init(&x_kalman_filter->xp, &x_kalman_filter->Pp, &filter_state->x_state[instance_number], &filter_state->x_cov[instance_number]);
        kalman_init(&y_kalman_filter->xp, &y_kalman_filter->Pp, &filter_state->y_state[instance_number], &filter_state->y_cov[instance_number]);
    }

    for(;;) {
            gpio_trace_instance(instance_number);

            /* Predict the system state */
            kalman_predict(x_kalman_filter);
            kalman_predict(y_kalman_filter);

            /* Obtain the measurements */
            xQueuePeek(measurement_queue, &measurement_struct, 0);

            /* Run the filter on the measurements */
            kalman_update(x_kalman_filter, measurement_struct.x_value);
            kalman_update(y_kalman_filter, measurement_struct.y_value);

            /* Update system state */
            kalman_state->x_state[instance_number].data[0] = x_kalman_filter->xp.data[0];
            kalman_state->x_state[instance_number].data[1] = x_kalman_filter->xp.data[1];
            kalman_state->y_state[instance_number].data[0] = y_kalman_filter->xp.data[0];
            kalman_state->y_state[instance_number].data[1] = y_kalman_filter->xp.data[1];

            /* Send results to print queue */
            result_struct.x_pos = x_kalman_filter->xp.data[0];
            result_struct.y_pos = y_kalman_filter->xp.data[0];
            result_struct.x_vel = x_kalman_filter->xp.data[1];
            result_struct.y_vel = y_kalman_filter->xp.data[1];

            /* Compare the results from multiple instances and run the failure handle if they differ */
            xTaskInstanceDone( (int) (result_struct.x_pos + result_struct.y_pos) * 1e5);
            gpio_trace_instance(instance_number);

            /* First thread to leave the barrier clears the queue */
            xQueueReceive(measurement_queue, &measurement_struct, 0);

            /* Send a new entry to the results/print queue */
            if (instance_number == 0) {
                xQueueSendToBack(results_queue, &result_struct, 0);
            }

            vTaskCallAPISynchronized(filter_task, vTaskSuspend);
    }
}

static void print_measurement_data(void *pvParameters)
{
    /* Print task waits for the filtering to be completed.
     * After being unblocked the task will print out all the
     * results.
     */
    (void) pvParameters;
    result_t filtering_result = {0};
    int data_point = 0;

    for(;;) {
        /* Print task waits for the result messages */
        if (!uxQueueMessagesWaiting(results_queue))
        {
            vTaskDelay(100);
            continue;
        }

        data_point++;
        xQueueReceive(results_queue, &filtering_result, 0);

        SERIAL_PRINT("%d, %.8lf, %.8lf, %.8lf, %.8lf", data_point,
            filtering_result.x_pos, filtering_result.x_vel, filtering_result.y_pos, filtering_result.y_vel);
    }
}

static void filter_failure_handler(void)
{
    /* This failure handle is activated when the task results
     * differ. It should return the task to the last known
     * correct state.
     */
    for(;;);
}

static void filter_timeout_handler(void)
{
    /* This failure handle is activated when the redundant
     * task times out. Task is restarted after running this
     * function, so the filter resources need to be cleaned
     * up, along with setting up the future task state.
     */
    for (int i = 0; i < TASK_INSTANCES; i++) {
        kalman_destroy(filter_storage.x_filters[i]);
        kalman_destroy(filter_storage.y_filters[i]);
    }
    for(;;);
}

void application_init(void)
{
    /* Initialize the queue, redundant task, and initialize the kalman filter.
     * After giving up control to the scheduler, the measurement timer will be
     * controling the filter execution.
     */
    BaseType_t error = 0;
    TaskFailureHandles_t failure_handles = {0};

    gpio_led_state(LED5_RED_ID, 1);
    SERIAL_PRINT(INIT_MSG);

    /* Measurement queue creation, single measurement */
    measurement_queue = xQueueCreate(1, sizeof(measurement_t));
    if (!measurement_queue) {
        while(1);
    }

    /* Result print queue creation */
    results_queue = xQueueCreate(MEASUREMENTS, sizeof(result_t));
    if (!results_queue) {
        while(1);
    }

    /* Measurement receive timer: measurements arrive every 40 ms */
    measurement_timer = xTimerCreate((const char *) "Measurement", 40/portTICK_RATE_MS, pdFALSE,
        NULL, displacement_data_update);
    if (!measurement_timer) {
        while(1);
    }

    /* Create a Kalman filter calculation task (time redundant task) */
    error = xTaskCreate(filtering_task_function, (const char *) "Kalman",
        configMINIMAL_STACK_SIZE * 2, NULL, configMAX_PRIORITIES-2, &filter_task, pdMS_TO_TICKS(2000));
    if (error <= 0) {
        while(1);
    }

    /* Create a single copy of a non critical UART print task */
    error = xTaskCreateInstance(print_measurement_data, (const char *) "Print",
        configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES-3, &print_task);
    if (error <= 0) {
        while(1);
    }

    /* Kalman filter defaults */
    default_x_state.data[0] = 0.0753;
    default_x_state.data[1] = 0;
    default_y_state.data[0] = -0.2801;
    default_y_state.data[1] = 0;

    default_x_cov.data[0] = 0.1;
    default_x_cov.data[1] = 0.1;
    default_y_cov.data[0] = 0.2;
    default_y_cov.data[1] = 0.2;

    /* Task failure restoration/cleanup functions */
    failure_handles.pvResultFailure = filter_failure_handler;
    failure_handles.pvTimeoutFailure = filter_timeout_handler;
    vTaskRegisterFailureCallback(filter_task, &failure_handles);

    /* Store pointer to shared task data - pointer to global variable is stored for demonstration purposes */
    kalman_state = user_malloc(sizeof(kalman_state_t));
    vTaskStoreData(filter_task, kalman_state);

    xTimerStart(measurement_timer, 0);
    vTaskSuspend(print_task);
    vTaskStartScheduler();
}
