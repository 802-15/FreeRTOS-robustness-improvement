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
TaskHandle_t filter_task;
TimerHandle_t measurement_timer;

/* Measurement data */
static BaseType_t measurement_count = 1;
static double x_measurement = 0;
static double y_measurement = 0;

kalman_handle_t filter_storage = {0};
kalman_state_t * kalman_state = NULL;

/* Default filter values */
vector_t default_x_state;
vector_t default_y_state;
vector_t default_x_cov;
vector_t default_y_cov;

static void displacement_data_update(TimerHandle_t xTimer)
{
    (void) xTimer;

    /* Obtain a new measurement */
    if (measurement_count >= MEASUREMENTS) {
        measurement_count = 0;
        while(1);
    }

    if (measurement_count % 2 == 0) {
        gpio_led_state(LED6_BLUE_ID, 1);
    } else {
        gpio_led_state(LED6_BLUE_ID, 0);
    }

    x_measurement = x_displacement[measurement_count];
    y_measurement = y_displacement[measurement_count];

    measurement_count++;

    /* Unblock the filtering task */
    vTaskResume(filter_task);
}

static void filtering_task_function(void *pvParameters)
{
    BaseType_t error_code = 0;
    BaseType_t instance_number = 0;
    kf_t * x_kalman_filter = NULL;
    kf_t * y_kalman_filter = NULL;
    kalman_state_t * filter_state = NULL;

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

        /* Predict the system state */
        kalman_predict(x_kalman_filter);
        kalman_predict(y_kalman_filter);

        /* Run the filter on the measurements */
        kalman_update(x_kalman_filter, x_measurement);
        kalman_update(y_kalman_filter, y_measurement);

        /* Update system state */
        kalman_state->x_state[instance_number].data[0] = x_kalman_filter->xp.data[0];
        kalman_state->x_state[instance_number].data[1] = x_kalman_filter->xp.data[1];
        kalman_state->y_state[instance_number].data[0] = y_kalman_filter->xp.data[0];
        kalman_state->y_state[instance_number].data[1] = y_kalman_filter->xp.data[1];

        /* Execute the following block only once */
        if ( xTaskGetInstanceNumber() == 0) {
            /* Print the filtered data */
            SERIAL_PRINT("%d, %.8lf, %.8lf, %.8lf, %.8lf", measurement_count, x_measurement, y_measurement,
                x_kalman_filter->xp.data[0], y_kalman_filter->xp.data[0]);
        }

        xTimerReset(measurement_timer, 0);

        /* Successful instance finish - suspend the task and wait for a new measurement */
        xTaskInstanceDone(1);
        vTaskCallAPISynchronized(filter_task, vTaskSuspend);
    }
}

void filter_failure_handler(void)
{
    /* Free task related resources in the failure handler */
    for (int i = 0; i < TASK_INSTANCES; i++) {
        kalman_destroy(filter_storage.x_filters[i]);
        kalman_destroy(filter_storage.y_filters[i]);
    }
}

void application_init(void)
{
    BaseType_t error = 0;

    SERIAL_PRINT(INIT_MSG);

    /* Create a Kalman filter calculation task (redundant task) */
    error = xTaskCreate(filtering_task_function, (const char *) "Kalman", configMINIMAL_STACK_SIZE * 2, NULL, configMAX_PRIORITIES-2, &filter_task, pdMS_TO_TICKS(5000));
    if (error <= 0) {
        while(1);
    }

    default_x_state.data[0] = 0.0753;
    default_x_state.data[1] = 0;
    default_y_state.data[0] = -0.2801;
    default_y_state.data[1] = 0;

    default_x_cov.data[0] = 0.1;
    default_x_cov.data[1] = 0.1;
    default_y_cov.data[0] = 0.2;
    default_y_cov.data[1] = 0.2;

    /* Task failure restoration function */
    vTaskRegisterFailureCallback(filter_task, filter_failure_handler);

    /* Store pointer to shared task data - pointer to global variable is stored for demonstration purposes */
    kalman_state = user_malloc(sizeof(kalman_state_t));
    vTaskStoreData(filter_task, kalman_state);

    /* Block the filtering task until a measurement was made */
    vTaskSuspend(filter_task);

    /* Measurement receive timer */
    measurement_timer = xTimerCreate((const char *) "Measurement", 100/portTICK_RATE_MS, pdFALSE, NULL, displacement_data_update);
    if (!measurement_timer) {
        while(1);
    }

    xTimerStart(measurement_timer, 0);

    gpio_led_state(LED5_RED_ID, 1);

    vTaskStartScheduler();
}