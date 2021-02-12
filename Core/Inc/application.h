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

#ifndef __APPLICATION_H
#define __APPLICATION_H

#include <math.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "timers.h"

#include "gpio.h"
#include "kalman.h"
#include "kalman_data.h"
#include "usart.h"
#include "xformatc.h"
#include "rng.h"
#include "wwdg.h"


#ifdef __cplusplus
extern "C" {
#endif

/* Initial message */
#define INIT_MSG \
    "\r\n" \
    "Application: Kalman filtering using redundant tasks \r\n" \
    "Built:" __DATE__ " " __TIME__ "\r\n"

/* Set this to the number of task instances */
#define TASK_INSTANCES configTIME_REDUNDANT_INSTANCES

/* Task return codes */
#define TASK_SUCCESS 0
#define TASK_FAILURE 1

/* Application timings (ms) and settings */
#define TIMER_PERIOD 40
#define FILTER_TIMEOUT 20
#define FAULT_PERIOD 200
#define FAULT_NUMBER 10

#define CAUSE_FAULTS 2
#define PRINT_STATS 0

/* This struct holds kalman filter pointers */
typedef struct kalman_handle {
    kf_t * x_filters[TASK_INSTANCES];
    kf_t * y_filters[TASK_INSTANCES];
} kalman_handle_t;

/* Kalman system states */
typedef struct kalman_state_storage {
    vector_t x_state[TASK_INSTANCES];
    vector_t y_state[TASK_INSTANCES];
    matrix_t x_cov[TASK_INSTANCES];
    matrix_t y_cov[TASK_INSTANCES];
} kalman_state_t;

/* Kalman filter measurement struct; for use with a queue */
typedef struct measurement {
    double x_value;
    double y_value;
} measurement_t;

/* Used to pass filtered information to print task */
typedef struct result {
    double x_pos;
    double x_vel;
    double y_pos;
    double y_vel;
} result_t;

/* Place runtime stats in this structure */
typedef struct runtime_stats {
    HeapStats_t heap_stats;
} stats_t;

/* Start up the application */
void application_init(void);

#endif /* __APPLICATION_H */