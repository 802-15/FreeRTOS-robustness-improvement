/*
 * Non scaleable 2x2 matrix algebra utilities used with Kalman filter
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

#ifndef __MATRIX_H
#define __MATRIX_H

#include "math.h"

/* Set up the matrix type as 2 dimensional array - size fixed at kalman filter model order (2x2) */
typedef struct {
    double data[2][2];
} matrix_t;

/* Fixed size vector type (2x1) */
typedef struct {
    double data[2];
} vector_t;

/* Matrix operations API */

/* 2x2 matrix multiplication */
void matrix_multiply(matrix_t * result, matrix_t * matrix1, matrix_t * matrix2);

/* 2x2 matrix multiplication with scalar */
void matrix_multiply_by_scalar(matrix_t * result, matrix_t * matrix, double operand);

/* Row/column vector multiplied by a scalar */
void vector_multiply_by_scalar(vector_t * result, vector_t * vector, double operand);

/* Add or subtract two 2x2 matrices */
void matrix_add_subtract(matrix_t * result, matrix_t * matrix1, matrix_t * matrix2, int operand);

/* Transpose 2x2 matrix */
void matrix_transpose(matrix_t * result, matrix_t * matrix);

/* 2x2 matrix multiplied by a row column */
void matrix_vector_multiply(vector_t * result, matrix_t * matrix, vector_t * vector);

/* Subtract two 2x1 column vectors */
void vector_add_subtract(vector_t * result, vector_t * vector1, vector_t * vector2, int operand);

/* Multiply vector row with a vector column */
double vector_row_column_multiply(vector_t * vector1, vector_t * vector2);

/* Multiply a vector column with a vector row */
void vector_column_row_multiply(matrix_t * result, vector_t * vector1, vector_t * vector2);

#endif /* __MATRIX_H */