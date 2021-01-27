/*
 * Simple Kalman filter implementation
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

#ifndef __KALMAN_H
#define __KALMAN_H

#include "matrix.h"
#include "memory_wrappers.h"


/* Filter scale: second order system for a single axis, filtering a single displacement measurement */
#define SYSTEM_ORDER 2
#define MEASUREMENTS_COUNT 1

enum kalman_errno {
    KALMAN_OK,
    KALMAN_ENOMEM,
    KALMAN_ESINGULARITY
};

/**
 * kalman. h
 *
 * Contains the system matrices. These values are constant and
 * should be initialized once per runtime using the helper in "kalman_data.h"
 */
typedef struct dynamic_system_model {
    matrix_t Phi;       /* State transition matrix for the discretized model */
    vector_t H;         /* Measurement transformation matrix */
    matrix_t Q;         /* Process noise covariance matrix */
    matrix_t R;         /* Measurement noise covariance matrix */
} system_t;

/**
 * kalman. h
 *
 * This structure is used for storing the variable system state information.
 * The values are updated in the prediction and update steps of the Kalman filter.
 */
typedef struct kalman_filter_container {
    system_t * model;   /* Pointer to system model structure, constant between usages */
    vector_t x_;        /* System state in the prediction step */
    vector_t xp;        /* System state in the update step */
    vector_t K;         /* Kalman gain array */
    matrix_t P_;        /* Covariance matrix in the prediction step */
    matrix_t Pp;        /* Covariance matrix in the update step */
} kf_t;


/**
 * kalman.h
 * <pre>
 * int kalman_create(kf_t ** filter, double R00);
 * </pre>
 *
 * Initialize memory for kalman filter structures/matrices. Initialize dynamic system.
 *
 * @param filter Double pointer to kalman filter structure
 *
 * @param R00 Measurement noise covariance value
 *
 * @return Returns the cumulative result of malloc calls inside the creation function
 *
 */
int kalman_create(kf_t ** filter, double R00);

/**
 * kalman.h
 * <pre>
 * void kalman_destroy(kf_t * filter);
 * </pre>
 *
 * Free the memory taken up by kalman filter strucures/matrices.
 *
 * @param filter Pointer to kalman filter structure
 *
 */
void kalman_destroy(kf_t * filter);

/**
 * kalman.h
 * <pre>
 * void kalman_init(vector_t * xp, matrix_t * Pp, vector_t * x_values, matrix_t * P_values);
 * </pre>
 *
 * Initialize the kalman filter state and state covariance values.
 * The input array dimensions should match the order of the Kalman filter.
 *
 * @param xp Pointer to vector type, Kalman state vector
 *
 * @param Pp Pointer to matrix type, Kalman covariance matrix
 *
 * @param x_values Vector type which containts the initial state values
 *
 * @param P_values Matrix type which contains the initial covariance values
 *
 */
void kalman_init(vector_t * xp, matrix_t * Pp, vector_t * x_values, matrix_t * P_values);

/**
 * kalman.h
 * <pre>
 * void kalman_destroy(kf_t * filter);
 * </pre>
 *
 * Perform the prediction step on the dynamic system model:
 * Calculate covariance estimate: P_ = Phi * Pp * Phi' + Q
 * Calculate state estimate: x_ = Phi * xp
 *
 * @param filter Pointer to kalman filter structure
 *
 */
void kalman_predict(kf_t * filter);

/**
 * kalman.h
 * <pre>
 * double kalman_update(kf_t * filter, double measurement);
 * </pre>
 *
 * Perform the estimate update step on the dynamic system model:
 * Gain calculation: K = (Pp * H(i)')/Ri
 * State update: xp = x_ + K(i) * (y - H(i) * x_(i-1))
 * Covariance update: Pp = (I - K(i) * H(i)) * Pp
 *
 * @param filter Pointer to kalman filter structure
 *
 * @param measurement Scalar measurement (single axis displacement data)
 *
 * @return P+ matrix trace
 *
 */
double kalman_update(kf_t * filter, double measurement);

/**
 * kalman.h
 * <pre>
 * void modify_states(kf_t * filter, double value_change);
 * </pre>
 *
 * Perturb the kalman system states to cause an error in
 * redundant instance execution.
 *
 * @param filter Pointer to kalman filter structure
 *
 * @param value_change Value to be added to the system states
 *
 */
void modify_states(kf_t * filter, double value_change);

#endif /* __KALMAN_H */