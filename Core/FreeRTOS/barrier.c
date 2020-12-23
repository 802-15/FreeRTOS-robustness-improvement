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


/* Standard includes. */
#include <stdlib.h>

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "barrier.h"
#include "task.h"


BaseType_t xBarrierCreate( barrierHandle_t ** pxTaskBarrierHandle, void * pxTaskHandle, TickType_t xTimeoutTicks )
{
    barrierHandle_t * pxBarrierHandle = NULL;

    pxBarrierHandle = pvPortMalloc( sizeof( barrierHandle_t ) + 2 * sizeof( SemaphoreHandle_t ) + sizeof( TimerHandle_t ) );
    if ( !pxBarrierHandle )
        return pdFREERTOS_ERRNO_ENOMEM;

    pxBarrierHandle->uxArriveCounter = 0;
    pxBarrierHandle->uxLeaveCounter = configTIME_REDUNDANT_INSTANCES;
    pxBarrierHandle->uxFlag = pdFALSE;
    pxBarrierHandle->uxTimerFlag = pdFALSE;
    pxBarrierHandle->xCounterMutex = 0;
    pxBarrierHandle->xBarrierSemaphore = 0;
    pxBarrierHandle->xBarrierTimer = 0;

    pxBarrierHandle->xCounterMutex = xSemaphoreCreateMutex();
    if ( !pxBarrierHandle->xCounterMutex )
        goto error_out;

    /* The semaphore should be given/taken by threads one by one */
    pxBarrierHandle->xBarrierSemaphore = xSemaphoreCreateCounting( 1, 1 );
    if ( !pxBarrierHandle->xBarrierSemaphore )
        goto error_out;

    /* Create the one-shot timer and start it  */
    pxBarrierHandle->xBarrierTimer = xTimerCreate( "Barrier timer", xTimeoutTicks, pdFALSE, pxTaskHandle, vBarrierTimerCallback );
    if ( !pxBarrierHandle->xBarrierTimer )
    {
        goto error_out;
    }

    *pxTaskBarrierHandle = pxBarrierHandle;
    return pdPASS;

error_out:
    /* vSemaphoreDelete calls vPortFree eventually */
    vSemaphoreDelete( pxBarrierHandle->xCounterMutex );
    vSemaphoreDelete( pxBarrierHandle->xBarrierSemaphore );
    xTimerDelete( pxBarrierHandle->xBarrierTimer, 100 );
    vPortFree( pxBarrierHandle );
    pxBarrierHandle = NULL;
    return pdFREERTOS_ERRNO_ENOMEM;
}

void vBarrierEnter( barrierHandle_t * pxBarrierHandle )
{
    if ( !pxBarrierHandle )
        return;

    if( xSemaphoreTake( pxBarrierHandle->xCounterMutex, portMAX_DELAY ) )
    {
        pxBarrierHandle->uxArriveCounter++;
        pxBarrierHandle->uxFlag = pdTRUE;
        xSemaphoreGive( pxBarrierHandle->xCounterMutex );

        if ( pxBarrierHandle->uxArriveCounter == 1 )
        {
            /* First thread will take the semaphore to block other threads */
            xSemaphoreTake( pxBarrierHandle->xBarrierSemaphore, portMAX_DELAY );
        }
    }

    /* Reset the barrier's timer on each new thread */
    xTimerReset( pxBarrierHandle->xBarrierTimer, 0 );

    if ( pxBarrierHandle->uxArriveCounter == pxBarrierHandle->uxLeaveCounter )
    {
        /* The last thread will leave the barrier and signal the rest of the threads externally */
        pxBarrierHandle->uxArriveCounter--;
        xTimerStop( pxBarrierHandle->xBarrierTimer, 0 );
    }
    else
    {
        /* All the threads that have entered the barrier wait here */
        xSemaphoreTake( pxBarrierHandle->xBarrierSemaphore, portMAX_DELAY );
        pxBarrierHandle->uxArriveCounter--;

        /* Threads will exit one by one */
        xSemaphoreGive( pxBarrierHandle->xBarrierSemaphore );
    }
    /* End of barrier */
}

void vBarrierSignal( barrierHandle_t * pxBarrierHandle )
{
    pxBarrierHandle->uxFlag = pdFALSE;
    xSemaphoreGive( pxBarrierHandle->xBarrierSemaphore );
}

BaseType_t xBarrierDestroy( barrierHandle_t * pxBarrierHandle )
{
    /* If the barrier is in use, it must be reset */
    if ( pxBarrierHandle->uxFlag == pdTRUE )
    {
        vBarrierSignal ( pxBarrierHandle );
        return pdFREERTOS_ERRNO_EACCES;
    }

    if ( pxBarrierHandle->xBarrierTimer )
    {
        xTimerDelete( pxBarrierHandle->xBarrierTimer, 0 );
    }

    taskENTER_CRITICAL();

    if ( uxSemaphoreGetCount( pxBarrierHandle->xCounterMutex ) == 0 || uxSemaphoreGetCount ( pxBarrierHandle->xBarrierSemaphore ) == 0 )
    {
        taskEXIT_CRITICAL();
        return pdFREERTOS_ERRNO_EACCES;
    }

    vSemaphoreDelete( pxBarrierHandle->xCounterMutex );
    vSemaphoreDelete( pxBarrierHandle->xBarrierSemaphore );
    vPortFree( pxBarrierHandle );

    pxBarrierHandle = NULL;

    taskEXIT_CRITICAL();
    return pdPASS;
}

void vBarrierTimerCallback( TimerHandle_t xTimer )
{
    /* Callback function for the oneshot barrier "watchdog" timer.
    * This timer must ensure that the barrier will do the proper error
    * handling in case some or all threads do not make it to the barrier.
    * This function is called from the FreeRTOS timer daemon and it should
    * result in changing the state of said task to 'blocked' */
    pvFailureHandle();
    xTimerReset( xTimer, 0 );
}
