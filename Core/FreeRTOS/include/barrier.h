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
#include "timers.h"

/* *INDENT-OFF* */
#ifdef __cplusplus
    extern "C" {
#endif
/* *INDENT-ON* */

typedef struct functionContainer
{
    void ( * pvFailureFunc ) ( void );      /*< Callback function container required for passing function pointers using timer ID API */
    TaskHandle_t * xTaskToReset;            /*< Task handle is used to reset the task from the timer callback */
} callbackContainer_t;

typedef struct barrierHandle
{
    UBaseType_t uxArriveCounter;            /*< Count of threads that have entered the barrier */
    UBaseType_t uxLeaveCounter;             /*< Total count of threads that can enter this barrier */
    UBaseType_t uxFlag;                     /*< Barrier active flag (used to check barrier status) */
    UBaseType_t uxTimerFlag;                /*< Barrier timer flag, set once the timer has been started */

    SemaphoreHandle_t xCounterMutex;        /*< Barrier counter protection mutex */
    SemaphoreHandle_t xBarrierSemaphore;    /*< Barrier waiting semaphore mutex */

    TimerHandle_t xBarrierTimer;            /*< Barrier can time out if the instances take too long to complete */

    callbackContainer_t pxCallbackStruct;   /*< Stores pointer to callback function */
} barrierHandle_t;

/**
 * barrier.h
 * <pre>
 * BaseType_t xBarrierCreate( barrierHandle_t ** pxTaskBarrierHandle, void ( * pvFailureFunc ) ( void ), TickType_t xTimeoutTicks , TaskHandle_t * pxCreatedTask );
 * </pre>
 * Create a barrier for task instances synchronization. The barrier
 * instance consits of a mutex, counting semaphore, state variable and two
 * counter variables. An additional timer is created for reseting the task
 * in case it gets blocked.
 *
 * @param pxTaskBarrierHandle Used to pass back a handle by which the barrier
 * information can be accessed.
 *
 * @param pvFailureFunc Pointer to the barrier watchdog timer callback function.
 * This function is the same as the task failure function called if all instances
 * did not return an expected value.
 *
 * @param xTimeoutTicksPointer Number of ticks which servers as the watchdog
 * timer period.
 *
 * @param pxCreatedTask Task handle pointer used for re-creating the task
 * in case the barrier times out.
 *
 * @return pdPASS if the barrier was successfully created and added to the
 * redundant task TCB, otherwise an error code defined in the file projdefs.h
 *
 */
BaseType_t xBarrierCreate( barrierHandle_t ** pxTaskBarrierHandle, void ( * pvFailureFunc ) ( void ), TickType_t xTimeoutTicks , TaskHandle_t * pxCreatedTask );

/**
 * barrier.h
 * <pre>
 * void vBarrierEnter( barrierHandle_t * pxBarrierHandle );
 * </pre>
 * Simple barrier synchronization using existing FreeRTOS objects. Once a thread
 * calls this function it will not resume execution until other threads have
 * called it too. The threads will exit the barrier one by one, in a turnstyle
 * manner. This implementation is taken from the "Little book of semaphores",
 * page 29, second edition, document version 2.2.1.
 *
 * This function must be called outside of critical sections.
 *
 * @param pxBarrierHandle Barrier handle is used to access the information related
 * to the barrier.
 *
 */
void vBarrierEnter( barrierHandle_t * pxBarrierHandle );

/**
 * barrier.h
 * <pre>
 * void vBarrierSignal( barrierHandle_t * pxBarrierHandle );
 * </pre>
 * Signal the barrier and release the waiting threads. This is done by
 * giving the sempahore and setting the barrier flag to unused (pdFALSE) state.
 *
 * @param pxBarrierHandle Barrier handle is used to access the information
 * related to the barrier.
 *
 */
void vBarrierSignal( barrierHandle_t * pxBarrierHandle );

/**
 * barrier.h
 * <pre>
 * BaseType_t xBarrierDestroy( barrierHandle_t * pxBarrierHandle );
 * </pre>
 * Destroy the barrier instance by deleting the semaphores, timers and freeing
 * the barrier structure.
 *
 * @param pxBarrierHandle Barrier handle is used to access the information
 * related to the barrier.
 *
 * @return pdPASS if the barrier was destroyed, otherwise an error code
 * defined in the file projdefs.h
 *
 */
BaseType_t xBarrierDestroy( barrierHandle_t * pxBarrierHandle );

/**
 * barrier.h
 * <pre>
 * void vBarrierTimerCallback( TimerHandle_t xTimer );
 * </pre>
 *
 * Callback function for the oneshot barrier "watchdog" timer.
 * This timer must ensure that the barrier will do the proper error
 * handling in case some or all threads do not make it to the barrier.
 * This function is called from the FreeRTOS timer daemon and it should
 * not result in changing the state of said task to 'blocked'.
 * Currently the timer will execute the failure handle and restart the task.
 *
 * @param xTimer Timer handle to ensure the timer callback prototype conformity
 *
 */
void vBarrierTimerCallback( TimerHandle_t xTimer );

#endif /* INC_BARRIER_H */
