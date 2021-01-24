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


char messageBuffer[256];

/* Formatting macro */
#define SERIAL_PRINT(FORMAT,...) \
    lightFormat(messageBuffer, FORMAT "\r\n", ##__VA_ARGS__ ); \
    USART1_SendString(messageBuffer);

/* FreeRTOS handles */
static TaskHandle_t filter_task = NULL;
static TimerHandle_t measurement_timer = NULL;
static QueueHandle_t measurement_queue = NULL;

/* Measurement data */
static BaseType_t measurement_count = 1;

/* Filter state restoration and cleanup structures */
kalman_handle_t filter_storage = {0};
kalman_state_t * kalman_state = NULL;

/* Default filter values */
vector_t default_x_state;
vector_t default_y_state;
vector_t default_x_cov;
vector_t default_y_cov;

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
    int error_code = pdFALSE;

    /* Obtain a new measurement */
    if (measurement_count >= MEASUREMENTS) {
        return;
    }

    if (measurement_count % 2 == 0) {
        gpio_led_state(LED6_BLUE_ID, 1);
    } else {
        gpio_led_state(LED6_BLUE_ID, 0);
    }

    measurement_struct.x_value = x_displacement[measurement_count];
    measurement_struct.y_value = y_displacement[measurement_count];
    measurement_count++;

    error_code = xQueueSendToBack(measurement_queue, &measurement_struct, 0);
    if(error_code == pdFALSE) {
        while(1);
    }

    /* Reset the timer and toggle the PIN for tracing purposes */
    HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_2);
    xTimerReset(xTimer, 0);
}

static void filtering_task_function(void *pvParameters)
{
    BaseType_t error_code = 0;
    BaseType_t instance_number = 0;
    kf_t * x_kalman_filter = NULL;
    kf_t * y_kalman_filter = NULL;
    kalman_state_t * filter_state = NULL;
    measurement_t measurement_struct = {0};
    double x_measurement = 0;
    double y_measurement = 0;

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

    while (1) {
        if (uxQueueMessagesWaiting(measurement_queue)) {

            if (xTaskGetInstanceNumber() == 0)
                HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_3);

            /* Predict the system state */
            kalman_predict(x_kalman_filter);
            kalman_predict(y_kalman_filter);

            xQueueReceive(measurement_queue, &measurement_struct, 0);
            x_measurement = measurement_struct.x_value;
            y_measurement = measurement_struct.y_value;

            /* Run the filter on the measurements */
            kalman_update(x_kalman_filter, x_measurement);
            kalman_update(y_kalman_filter, y_measurement);

            if (xTaskGetInstanceNumber() == 0)
                HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_3);

            /* Update system state */
            kalman_state->x_state[instance_number].data[0] = x_kalman_filter->xp.data[0];
            kalman_state->x_state[instance_number].data[1] = x_kalman_filter->xp.data[1];
            kalman_state->y_state[instance_number].data[0] = y_kalman_filter->xp.data[0];
            kalman_state->y_state[instance_number].data[1] = y_kalman_filter->xp.data[1];

            if (xTaskGetInstanceNumber() == 0)
                HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_5);

            /* Execute the following block only once */
            if ( xTaskGetInstanceNumber() == 0) {
                /* Print the filtered data */
                SERIAL_PRINT("%d, %.8lf, %.8lf, %.8lf, %.8lf", measurement_count, x_measurement, y_measurement,
                    x_kalman_filter->xp.data[0], y_kalman_filter->xp.data[0]);
            }

            if (xTaskGetInstanceNumber() == 0)
                HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_5);

            /* Successful instance finish */
            xTaskInstanceDone(1);
        }
    }
}

static void filter_failure_handler(void)
{
    /* Return the task state to previous values */
    return;
}

static void filter_timeout_handler(void)
{
    /* Free task related resources in the failure handler */
    for (int i = 0; i < TASK_INSTANCES; i++) {
        kalman_destroy(filter_storage.x_filters[i]);
        kalman_destroy(filter_storage.y_filters[i]);
    }
}

void application_init(void)
{
    /* Initialize the queue, redundant task, and initialize the kalman filter.
     * After giving up control to the scheduler, the measurement timer will be
     * controling the filter execution.
     */
    BaseType_t error = 0;
    TaskFailureHandles_t failure_handles = {0};

    SERIAL_PRINT(INIT_MSG);

    /* Measurement queue initialization */
    measurement_queue = xQueueCreate(100, sizeof(measurement_t));
    if (!measurement_queue) {
        while(1);
    }

    /* Measurement receive timer: measurements arrive every 40 ms */
    measurement_timer = xTimerCreate((const char *) "Measurement", 40/portTICK_RATE_MS, pdFALSE, NULL, displacement_data_update);
    if (!measurement_timer) {
        while(1);
    }

    /* Create a Kalman filter calculation task (time redundant task) */
    error = xTaskCreate(filtering_task_function, (const char *) "Kalman", configMINIMAL_STACK_SIZE * 2, NULL, configMAX_PRIORITIES-2, &filter_task, pdMS_TO_TICKS(100));
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
    gpio_led_state(LED5_RED_ID, 1);
    vTaskStartScheduler();
}
