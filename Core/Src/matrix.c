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

#include "matrix.h"


void matrix_multiply(matrix_t * result, matrix_t * matrix1, matrix_t * matrix2)
{
    /* 2x2 matrix multiplication helper */
    result->data[0][0] = matrix1->data[0][0] * matrix2->data[0][0] + matrix1->data[0][1] * matrix2->data[1][0];
    result->data[0][1] = matrix1->data[0][0] * matrix2->data[0][1] + matrix1->data[0][1] * matrix2->data[1][1];
    result->data[1][0] = matrix1->data[1][0] * matrix2->data[0][0] + matrix1->data[1][1] * matrix2->data[1][0];
    result->data[1][1] = matrix1->data[1][0] * matrix2->data[0][1] + matrix1->data[1][1] * matrix2->data[1][1];

    result->data[0][0] = roundf(result->data[0][0] * 1e12) / 1e12;
    result->data[0][1] = roundf(result->data[0][0] * 1e12) / 1e12;
    result->data[1][0] = roundf(result->data[0][0] * 1e12) / 1e12;
    result->data[1][1] = roundf(result->data[0][0] * 1e12) / 1e12;
}

void matrix_multiply_by_scalar(matrix_t * result, matrix_t * matrix, double operand)
{
    /* 2x2 multiplied by a scalar */
    result->data[0][0] = matrix->data[0][0] * operand;
    result->data[0][1] = matrix->data[0][1] * operand;
    result->data[1][0] = matrix->data[1][0] * operand;
    result->data[1][1] = matrix->data[1][1] * operand;
}

void vector_multiply_by_scalar(vector_t * result, vector_t * vector, double operand)
{
    /*2x1 multiplied by scalar */
    result->data[0] = vector->data[0] * operand;
    result->data[1] = vector->data[1] * operand;
}

void matrix_add_subtract(matrix_t * result, matrix_t * matrix1, matrix_t * matrix2, int operand)
{
    /* 2x2 matrix addition/subtraction */
    if (operand == 0) {
        /* Add */
        result->data[0][0] = matrix1->data[0][0] + matrix2->data[0][0];
        result->data[0][1] = matrix1->data[0][1] + matrix2->data[0][1];
        result->data[1][0] = matrix1->data[1][0] + matrix2->data[1][0];
        result->data[1][1] = matrix1->data[1][1] + matrix2->data[1][1];
    } else {
        /* Subtract */
        result->data[0][0] = matrix1->data[0][0] - matrix2->data[0][0];
        result->data[0][1] = matrix1->data[0][1] - matrix2->data[0][1];
        result->data[1][0] = matrix1->data[1][0] - matrix2->data[1][0];
        result->data[1][1] = matrix1->data[1][1] - matrix2->data[1][1];
    }
}

void matrix_transpose(matrix_t * result, matrix_t * matrix)
{
    /* 2x2 matrix transpose */
    result->data[0][0] = matrix->data[0][0];
    result->data[0][1] = matrix->data[1][0];
    result->data[1][0] = matrix->data[0][1];
    result->data[1][1] = matrix->data[1][1];
}

void matrix_vector_multiply(vector_t * result, matrix_t * matrix, vector_t * vector)
{
    /* 2x2 * 2x1 multiplication */
    result->data[0] = matrix->data[0][0] * vector->data[0] + matrix->data[0][1] * vector->data[1];
    result->data[1] = matrix->data[1][0] * vector->data[0] + matrix->data[1][1] * vector->data[1];
}

void vector_add_subtract(vector_t * result, vector_t * vector1, vector_t * vector2, int operand)
{
    /* 2x1 matrix addition/subtraction */
    if (operand == 0) {
        result->data[0] = vector1->data[0] + vector2->data[0];
        result->data[1] = vector1->data[1] + vector2->data[1];
    } else {
        result->data[0] = vector1->data[0] - vector2->data[0];
        result->data[1] = vector1->data[1] - vector2->data[1];
    }
}

double vector_row_column_multiply(vector_t * vector1, vector_t * vector2)
{
    /* Multiply a row and column to a scalar result */
    return (vector1->data[0] * vector2->data[0] + vector1->data[1] * vector2->data[1]); 
}

void vector_column_row_multiply(matrix_t * result, vector_t * vector1, vector_t * vector2)
{
    /* Multiply a column and a row to a matrix result */
    result->data[0][0] = vector1->data[0] * vector2->data[0];
    result->data[0][1] = vector1->data[0] * vector2->data[1];
    result->data[1][0] = vector1->data[1] * vector2->data[0];
    result->data[1][1] = vector1->data[1] * vector2->data[1];
}
