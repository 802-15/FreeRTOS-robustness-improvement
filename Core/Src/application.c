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
static TaskHandle_t fault_task = NULL;
static TimerHandle_t measurement_timer = NULL;
static QueueHandle_t measurement_queue = NULL;
static QueueHandle_t results_queue = NULL;
static QueueHandle_t stats_queue = NULL;

/* Filter state restoration and cleanup structures */
static kalman_handle_t filter_storage = {0};
static kalman_state_t * kalman_state = NULL;

/* Default filter values */
static vector_t default_x_state = {0};
static vector_t default_y_state = {0};
static matrix_t default_x_cov = {0};
static matrix_t default_y_cov = {0};

static int data_corrupted = 0;
uint32_t timing_cycles = 0;

void vApplicationTickHook(void)
{
    static int watchdog_active = 0;

    if ( !watchdog_active ) {
        MX_WWDG_Init();
        watchdog_active++;
    }

    /* Refresh the watchdog on FreeRTOS tick */
    HAL_WWDG_Refresh(&hwwdg);
}

static void check_final_result(void)
{
    /* Run the final value against the final result and signal the logic analyzer */
    if ( \
        fabs(filter_storage.x_filters[0]->xp.data[0] - (-1.92119181)) > 1e-1 || \
        fabs(filter_storage.y_filters[0]->xp.data[0] - (1.89413702)) > 1e-1 ) {
            return;
        }
    HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_3);
    SERIAL_PRINT("SUCCESS,,,");
}

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
    static BaseType_t measurement_count = 1;

    /* Display the results if final measurement was reached */
    if (measurement_count >= MEASUREMENTS) {
        xTimerStop(xTimer, 0);
        vTaskSuspend(filter_task);
        vTaskSuspend(fault_task);

        if (CAUSE_FAULTS == 2) {
            check_final_result();
            vTaskDelay(100/portTICK_RATE_MS);

            HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_1);
            SERIAL_PRINT("Done,,,");
            vTaskDelay(100/portTICK_RATE_MS);
            NVIC_SystemReset();
        }

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
    BaseType_t instance_states = pdFALSE;
    kf_t * x_kalman_filter = NULL;
    kf_t * y_kalman_filter = NULL;
    kalman_state_t * filter_state = NULL;
    measurement_t measurement_struct = {0};
    result_t result_struct = {0};
    stats_t stats_struct = {0};
    HeapStats_t heap_struct = {0};
    uint32_t task_result = 0;
    uint32_t timing_start = 0;
    uint32_t timing_end = 0;

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
            if (CAUSE_FAULTS == 1) {
                gpio_trace_instance(instance_number);
            }

            /* Predict the system state */
            kalman_predict(x_kalman_filter);
            kalman_predict(y_kalman_filter);

            /* Obtain the measurements */
            xQueuePeek(measurement_queue, &measurement_struct, 0);

            /* Run the filter on the measurements */
            kalman_update(x_kalman_filter, measurement_struct.x_value);
            kalman_update(y_kalman_filter, measurement_struct.y_value);

            /* Send out the result to other nodes; distance in centimiters from the starting point */
            task_result = (uint32_t ) 1e3 * fabs(fabs(filter_storage.x_filters[0]->xp.data[0] -  0.0753) - fabs(filter_storage.y_filters[0]->xp.data[0] - (-0.2801)));

            if ( instance_number == 0 ) {
                timing_start = ARM_CM_DWT_CYCCNT;
            }

            instance_states = xTaskInstanceDone(task_result);

            if ( instance_number == 0 ) {
                timing_end = ARM_CM_DWT_CYCCNT;
                timing_cycles = timing_end - timing_start;
            }

            /* Update last known good state if the execution is allright */
            if (instance_states == pdTRUE) {
                /* Update system state */
                kalman_state->x_state[instance_number].data[0] = x_kalman_filter->xp.data[0];
                kalman_state->x_state[instance_number].data[1] = x_kalman_filter->xp.data[1];
                kalman_state->y_state[instance_number].data[0] = y_kalman_filter->xp.data[0];
                kalman_state->y_state[instance_number].data[1] = y_kalman_filter->xp.data[1];
                /* Update system covariance */
                kalman_state->x_cov[instance_number].data[0][0] = x_kalman_filter->Pp.data[0][0];
                kalman_state->x_cov[instance_number].data[0][1] = x_kalman_filter->Pp.data[0][1];
                kalman_state->x_cov[instance_number].data[1][0] = x_kalman_filter->Pp.data[1][0];
                kalman_state->x_cov[instance_number].data[1][1] = x_kalman_filter->Pp.data[1][1];

                kalman_state->y_cov[instance_number].data[0][0] = y_kalman_filter->Pp.data[0][0];
                kalman_state->y_cov[instance_number].data[0][1] = y_kalman_filter->Pp.data[0][1];
                kalman_state->y_cov[instance_number].data[1][0] = y_kalman_filter->Pp.data[1][0];
                kalman_state->y_cov[instance_number].data[1][1] = y_kalman_filter->Pp.data[1][1];
            }

            /* Compare the results from multiple instances and run the failure handle if they differ */
            if (CAUSE_FAULTS == 1) {
                gpio_trace_instance(instance_number);
            }

            /* First thread to leave the barrier clears the queue */
            xQueueReceive(measurement_queue, &measurement_struct, 0);

            #if (CAUSE_FAULTS != 2)
            if (instance_number == 1 ) {
                /* Send results to print queue */
                result_struct.x_pos = x_kalman_filter->xp.data[0];
                result_struct.y_pos = y_kalman_filter->xp.data[0];
                result_struct.x_vel = x_kalman_filter->xp.data[1];
                result_struct.y_vel = y_kalman_filter->xp.data[1];
                xQueueSendToBack(results_queue, &result_struct, 0);

                /* Get heap stats */
                vPortGetHeapStats(&heap_struct);
                stats_struct.heap_stats = heap_struct;
                xQueueSendToBack(stats_queue, &stats_struct, 0);
            }
            #endif

            /* Wait for the next measurement (timer update) */
            vTaskCallAPISynchronized(filter_task, vTaskSuspend);
    }

    /* Disable warnings */
    (void) stats_struct;
    (void) result_struct;
    (void) heap_struct;
}

static void print_measurement_data(void *pvParameters)
{
    /* Print task waits for the filtering to be completed.
     * After being unblocked the task will print out all the
     * results.
     */
    (void) pvParameters;
    result_t filtering_result = {0};
    stats_t filtering_stats = {0};
    int data_point = 0;

    for(;;) {
        /* Print task waits for the result messages */
        if (!uxQueueMessagesWaiting(results_queue) && !uxQueueMessagesWaiting(stats_queue))
        {
            vTaskDelay(100);
            continue;
        }

        if (uxQueueMessagesWaiting(results_queue)) {
            xQueueReceive(results_queue, &filtering_result, 0);
            data_point++;

            SERIAL_PRINT("%d, %.8lf, %.8lf, %.8lf, %.8lf ", data_point,
                filtering_result.x_pos, filtering_result.x_vel, filtering_result.y_pos, filtering_result.y_vel);

            continue;
        }

        if (uxQueueMessagesWaiting(stats_queue) && PRINT_STATS == 1) {
            xQueueReceive(stats_queue, &filtering_stats, 0);

            SERIAL_PRINT("%lu, %lu, %lu, %lu, %lu",
                filtering_stats.heap_stats.xAvailableHeapSpaceInBytes,
                filtering_stats.heap_stats.xMinimumEverFreeBytesRemaining,
                filtering_stats.heap_stats.xNumberOfFreeBlocks,
                filtering_stats.heap_stats.xSizeOfLargestFreeBlockInBytes,
                filtering_stats.heap_stats.xSizeOfSmallestFreeBlockInBytes);
        }
    }
}

static void filter_failure_handler(void)
{
    /* This failure handle is activated when the task results
     * differ. It should return the task to the last known
     * correct state.
     */

    HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_2);

    if (data_corrupted) {
        for (int i = 0; i < TASK_INSTANCES; i++) {
            kalman_destroy(filter_storage.x_filters[i]);
            kalman_destroy(filter_storage.y_filters[i]);
        }
        memset(kalman_state, 0, sizeof(kalman_state_t));
                SERIAL_PRINT("RESULT_CORRUPTED,,,")
        xTaskReset(&filter_task);
    }

    for (int i = 0; i < TASK_INSTANCES; i++) {
        kalman_init(&filter_storage.x_filters[i]->xp, &filter_storage.x_filters[i]->Pp, &kalman_state->x_state[i], &kalman_state->x_cov[i]);
        kalman_init(&filter_storage.y_filters[i]->xp, &filter_storage.y_filters[i]->Pp, &kalman_state->y_state[i], &kalman_state->y_cov[i]);
    }

    data_corrupted = 1;

    /* Continue executing task */
    SERIAL_PRINT("RESULTS,,,");
}

static void filter_timeout_handler(void)
{
    /* This failure handle is activated when the redundant
     * task times out. Task is restarted after running this
     * function, so the filter resources need to be cleaned
     * up, along with setting up the future task state.
     */

    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_0);

    for (int i = 0; i < TASK_INSTANCES; i++) {
        kalman_destroy(filter_storage.x_filters[i]);
        kalman_destroy(filter_storage.y_filters[i]);
    }
    SERIAL_PRINT("TIMEOUT,,,");
    /* Task is deleted and created again (restart) ...*/
}

static void can_timeout_handler(void)
{
    SERIAL_PRINT("CAN-TIMEOUT,,,");
    NVIC_SystemReset();
}

static void fault_task_function(void *pvParameters)
{
    /* Cause a fault what will make the task time out,
     * or introduce a difference in the final result.
     */
    (void) pvParameters;
    uint32_t random_uint;
    int8_t * random_byte;
    uint8_t random_uchar;
    int fault_number = 0;

    /* 0x20013090 0x200131c8 */
    /* 0x690 */
    /* 0x20000000 */
    /* 0x2C00 */
    const size_t ram_size = 0x690;
    const size_t ram_start = 0x200131c8;

    /* Set up the failure location manually */
    size_t memory_address = 0x20000300;

    for(;;) {
        if (CAUSE_FAULTS == 0) {
            vTaskDelay(FAULT_PERIOD/portTICK_RATE_MS);
            continue;
        }

        fault_number++;
        if (fault_number >= FAULT_NUMBER) {
            vTaskSuspend(fault_task);
        }

        /* Pin toggle for logic analyzer timing diagnostic */
        HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_5);

        if (CAUSE_FAULTS == 1) {
            /* Overwrite byte value or modify bit value manually */
            random_byte = ( int8_t * ) memory_address;
            /* Run this one in debugger! */
            * random_byte ^= ( 1 << get_random_integer() % 8);

            vTaskDelay(FAULT_PERIOD/portTICK_RATE_MS);
        }

        if (CAUSE_FAULTS == 2) {
            /* Flip a random address + bit */
            random_uint = get_random_integer();
            random_uchar = random_uint % 8;

            memory_address = (random_uint % ram_size) + ram_start;
            random_byte = ( int8_t * ) memory_address;

            SERIAL_PRINT("[Address, Address, Bit], 0x%x, %lu, %u", memory_address, random_uint % ram_size, random_uchar);
            vTaskDelay(FAULT_PERIOD/portTICK_RATE_MS);

            * random_byte ^= ( 1 << random_uchar);
        }
    }
}

void application_init(void)
{
    /* Initialize the queue, redundant task, and initialize the kalman filter.
     * After giving up control to the scheduler, the measurement timer will be
     * controling the filter execution.
     */
    BaseType_t error = 0;
    failureHandles_t failure_handles = {0};

    gpio_led_state(LED5_RED_ID, 1);
    if (CAUSE_FAULTS != 2) {
        SERIAL_PRINT(INIT_MSG);
    }
    SERIAL_PRINT("START,,,");

    /* Measurement queue creation, single measurement */
    measurement_queue = xQueueCreate(1, sizeof(measurement_t));
    if (!measurement_queue) {
        while(1);
    }

    #if (CAUSE_FAULTS != 2)
    /* Result print queue creation */
    results_queue = xQueueCreate(MEASUREMENTS, sizeof(result_t));
    if (!results_queue) {
        while(1);
    }

    /* Runtime statistics queue */
    stats_queue = xQueueCreate(MEASUREMENTS, sizeof(stats_t));
    if (!results_queue) {
        while(1);
    }
    #endif

    /* Measurement receive timer: measurements arrive every 40 ms */
    measurement_timer = xTimerCreate((const char *) "Measurement", TIMER_PERIOD/portTICK_RATE_MS, pdFALSE,
        NULL, displacement_data_update);
    if (!measurement_timer) {
        while(1);
    }

    /* Create a Kalman filter calculation task (time redundant task) */
    error = xTaskCreate(filtering_task_function, (const char *) "Kalman",
        configMINIMAL_STACK_SIZE * 2, NULL, configMAX_PRIORITIES-2, &filter_task, FILTER_TIMEOUT/portTICK_RATE_MS);
    if (error <= 0) {
        while(1);
    }

    /* Create a single copy of a non critical UART print task */
    error = xTaskCreateInstance(print_measurement_data, (const char *) "Print",
        configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES-3, &print_task);
    if (error <= 0) {
        while(1);
    }

    /* Create a single copy of a fault inducing task */
    error = xTaskCreateInstance(fault_task_function, (const char *) "Generate fault",
        configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES-2, &fault_task);
    if (error <= 0) {
        while(1);
    }

    /* Kalman filter defaults */
    default_x_state.data[0] = 0.0753;
    default_x_state.data[1] = 0;
    default_y_state.data[0] = -0.2801;
    default_y_state.data[1] = 0;

    default_x_cov.data[0][0] = 0.1;
    default_x_cov.data[1][1] = 0.1;
    default_y_cov.data[0][0] = 0.2;
    default_y_cov.data[1][1] = 0.2;

    /* Task failure restoration/cleanup functions */
    failure_handles.pvRemoteFailureFunc = filter_failure_handler;
    failure_handles.pvLocalTimeoutFunc = filter_timeout_handler;
    failure_handles.pvRemoteTimeoutFunc = can_timeout_handler;
    vTaskRegisterFailureCallback(filter_task, &failure_handles);

    /* Set up the CAN peripheral */
    CAN2_Register();

    /* Register the task on CAN bus */
    vTaskCANRegister(filter_task);

    /* Store pointer to shared task data - pointer to global variable is stored for demonstration purposes */
    kalman_state = user_malloc(sizeof(kalman_state_t));
    memset(kalman_state, 0, sizeof(kalman_state_t));
    vTaskStoreData(filter_task, kalman_state);

    xTimerStart(measurement_timer, 0);
    vTaskSuspend(print_task);

    reset_dwt_timer();
    vTaskStartScheduler();
}
