/*
 * Simple Kalman filter implementation.
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

#include <string.h>
#include "kalman.h"


matrix_t matrix_identity;

int kalman_create(kf_t ** filter_handle, double R00)
{
    kf_t * filter = NULL;
    system_t * system = NULL;

    /* Allocate storage for kalman values */
    filter = user_malloc(sizeof(kf_t));
    if(!filter) {
        return KALMAN_ENOMEM;
    }

    memset(filter, 0, sizeof(kf_t));

    filter->model = NULL;

    system = user_malloc(sizeof(system_t));
    if(!system) {
        user_free(system);
        return KALMAN_ENOMEM;
    }

    memset(system, 0, sizeof(system_t));

    /* Set up default system values */
    system->Phi.data[0][0] = 1;
    system->Phi.data[0][1] = 0.01;
    system->Phi.data[1][0] = 0;
    system->Phi.data[1][1] = 1;

    /* Measurement transformation matrix */
    system->H.data[0] = 1;
    system->H.data[1] = 0;

    /* Process noise covariance */
    system->Q.data[0][0] = 0.008;
    system->Q.data[0][1] = 0;
    system->Q.data[1][0] = 0;
    system->Q.data[1][1] = 2;

    /* Measurement noise covariance */
    system->R.data[0][0] = R00;
    system->R.data[0][1] = 0;
    system->R.data[1][0] = 0;
    system->R.data[1][1] = 0;

    filter->model = system;

    *filter_handle = filter;
    return KALMAN_OK;
}

void kalman_destroy(kf_t * filter)
{
    /* Delete kalman filter info stored on the heap */
    if(!filter)
        return;

    if(filter->model)
        user_free(filter->model);

    user_free(filter);
}

void kalman_init(vector_t * xp, matrix_t * Pp, vector_t * x_values, vector_t * P_values)
{
    /* Set up covariance values */
    Pp->data[0][0] = P_values->data[0];
    Pp->data[0][1] = 0;
    Pp->data[0][1] = 0;
    Pp->data[1][1] = P_values->data[1];

    /* Set up state values */
    xp->data[0] = x_values->data[0];
    xp->data[1] = x_values->data[1];

    /* Set up identity matrix */
    matrix_identity.data[0][0] = 1;
    matrix_identity.data[0][1] = 0;
    matrix_identity.data[1][0] = 0;
    matrix_identity.data[1][1] = 1;
}

void kalman_predict(kf_t * filter)
{
    /* Estimate state and covariance - 2 matrix equations */
    /* Calculate covariance estimate: P_ = Phi * Pp * Phi' + Q */
    matrix_transpose(&filter->P_, &filter->model->Phi);
    matrix_multiply(&filter->P_, &filter->Pp, &filter->P_);
    matrix_multiply(&filter->P_, &filter->model->Phi, &filter->P_);
    matrix_add_subtract(&filter->P_, &filter->P_, &filter->model->Q, 0);

    /* Calculate state estimate: x_ = Phi * xp */
    matrix_vector_multiply(&filter->x_, &filter->model->Phi, &filter->xp);
}

double kalman_update(kf_t * filter, double measurement)
{
    double temp = 0;

    /* Advance the estimate state and covariance */
    filter->Pp = filter->P_;
    filter->xp = filter->x_;

    /* Gain calculation: K = (Pp * H(i)')/Ri */
    matrix_vector_multiply(&filter->K, &filter->Pp, &filter->model->H);
    temp = 1.0/filter->model->R.data[0][0];
    vector_multiply_by_scalar(&filter->K, &filter->K, temp);

    /* State update: xp = x_ + K(i) * (y - H(i) * x_(i-1)) */
    temp = vector_row_column_multiply(&filter->model->H, &filter->x_);
    temp = measurement - temp;
    vector_multiply_by_scalar(&filter->xp, &filter->K, temp);
    vector_add_subtract(&filter->xp, &filter->xp, &filter->x_, 0);

    /* Covariance update: Pp = (I - K(i) * H(i)) * Pp */
    vector_column_row_multiply(&filter->Pp, &filter->K, &filter->model->H);
    matrix_add_subtract(&filter->Pp, &matrix_identity, &filter->Pp, 1);
    matrix_multiply(&filter->Pp, &filter->Pp, &filter->P_);

    /* Returns the P matrix trace */
    return filter->Pp.data[0][0] + filter->Pp.data[1][1];
}
