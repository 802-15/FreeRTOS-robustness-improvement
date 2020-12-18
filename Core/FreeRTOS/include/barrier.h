/*
 * FreeRTOS Kernel V10.4.2
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


#ifndef INC_BARRIER_H
#define INC_BARRIER_H

#include "semphr.h"

typedef struct barrierHandle
{
    UBaseType_t uxArriveCounter;            /*< Count of threads that have entered the barrier */
    UBaseType_t uxLeaveCounter;             /*< Count of threads that have exited the barrier */
    UBaseType_t uxFlag;                     /*< Barrier active flag (determines if it will pass threads) */

    SemaphoreHandle_t xCounterMutex;        /*< Barrier counter protection mutex */
    SemaphoreHandle_t xBarrierSemaphore;    /*< Barrier waiting semaphore mutex */
} barrierHandle_t;

/**
 * barrier. h
 * <pre>
 * BaseType_t xBarrierCreate( barrierHandle_t ** pxTaskBarrierHandle );
 * </pre>
 * Create a barrier instance for task instance synchronization.
 *
 * @param pxTaskBarrierHandle Used to pass back a handle by which the barrier
 * information can be accessed.
 *
 * @return pdPASS if the task was successfully created and added to a ready
 * list, otherwise an error code defined in the file projdefs.h
 *
 */
BaseType_t xBarrierCreate( barrierHandle_t ** pxTaskBarrierHandle );

/**
 * barrier. h
 * <pre>
 * void vBarrierEnter( barrierHandle_t * pxBarrierHandle,
 *                     TaskHandle_t * pxTaskInstance );
 * </pre>
 * Simple barrier synchronization using existing FreeRTOS objects.
 *
 * @param pxBarrierHandle Used to pass back a handle by which the barrier
 * information can be accessed.
 *
 */
void vBarrierEnter( barrierHandle_t * pxBarrierHandle );

#endif /* INC_BARRIER_H */