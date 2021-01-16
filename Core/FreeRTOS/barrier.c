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


BaseType_t xBarrierCreate( barrierHandle_t ** pxTaskBarrierHandle, TaskFailureFunction_t pvFailureFunc, TickType_t xTimeoutTicks , TaskHandle_t * pxCreatedTask )
{
    barrierHandle_t * pxBarrierHandle = NULL;

    /* Barrier can't have a timer set up without a task handle for reseting the task */
    if ( xTimeoutTicks != 0 && !pxCreatedTask )
        goto error_out;

    pxBarrierHandle = pvPortMalloc( sizeof( barrierHandle_t ) );
    if ( !pxBarrierHandle )
        return pdFREERTOS_ERRNO_ENOMEM;

    pxBarrierHandle->uxArriveCounter = 0;
    pxBarrierHandle->uxLeaveCounter = configTIME_REDUNDANT_INSTANCES;
    pxBarrierHandle->uxFlag = pdFALSE;
    pxBarrierHandle->uxTimerFlag = pdFALSE;
    pxBarrierHandle->xCounterMutex = 0;
    pxBarrierHandle->xBarrierSemaphore = 0;
    pxBarrierHandle->xBarrierTimer = 0;

    pxBarrierHandle->pxCallbackStruct.pvFailureFunc = pvFailureFunc;
    pxBarrierHandle->pxCallbackStruct.xTaskToReset = pxCreatedTask;

    pxBarrierHandle->xCounterMutex = xSemaphoreCreateMutex();
    if ( !pxBarrierHandle->xCounterMutex )
        goto error_out;

    /* The semaphore should be given/taken by threads one by one */
    pxBarrierHandle->xBarrierSemaphore = xSemaphoreCreateCounting( 1, 1 );
    if ( !pxBarrierHandle->xBarrierSemaphore )
        goto error_out;

    /* Create the one-shot timer */
    if ( xTimeoutTicks )
    {
        pxBarrierHandle->xBarrierTimer = xTimerCreate( "Barrier timer", xTimeoutTicks, pdFALSE, ( void * ) &pxBarrierHandle->pxCallbackStruct, vBarrierTimerCallback );
        if ( !pxBarrierHandle->xBarrierTimer )
        {
            goto error_out;
        }
    }

    #if ( configUSE_SPATIAL_REDUNDANCY == 1 )
        pxBarrierHandle->uxRemoteCounter = 0;

        pxBarrierHandle->xRemoteCounterMutex = xSemaphoreCreateMutex();
        if ( !pxBarrierHandle->xRemoteCounterMutex )
            goto error_out;

    #endif

    *pxTaskBarrierHandle = pxBarrierHandle;
    return pdPASS;

error_out:
    /* vSemaphoreDelete calls vPortFree eventually */
    vSemaphoreDelete( pxBarrierHandle->xCounterMutex );
    vSemaphoreDelete( pxBarrierHandle->xBarrierSemaphore );
    xTimerDelete( pxBarrierHandle->xBarrierTimer, 0 );
    vPortFree( pxBarrierHandle );
    pxBarrierHandle = NULL;
    #if ( configUSE_SPATIAL_REDUNDANCY == 1 )
        vSemaphoreDelete( pxBarrierHandle->xRemoteCounterMutex );
    #endif
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
    if ( pxBarrierHandle->xBarrierTimer )
        xTimerReset( pxBarrierHandle->xBarrierTimer, 0 );

    if ( pxBarrierHandle->uxArriveCounter == pxBarrierHandle->uxLeaveCounter )
    {
        /* The last thread will leave the barrier and signal the rest of the threads externally */
        pxBarrierHandle->uxArriveCounter--;
        if ( pxBarrierHandle->xBarrierTimer )
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

BaseType_t xBarrierGetState( barrierHandle_t * pxBarrierHandle )
{
    if ( pxBarrierHandle->uxFlag == pdTRUE && pxBarrierHandle->uxArriveCounter > 0 )
    {
        return pdTRUE;
    }
    else
    {
        return pdFALSE;
    }
}

void vBarrierDestroy( barrierHandle_t * pxBarrierHandle )
{
    if ( !pxBarrierHandle )
        return;

    taskENTER_CRITICAL();

    if ( pxBarrierHandle->xBarrierTimer )
    {
        xTimerDelete( pxBarrierHandle->xBarrierTimer, 0 );
    }

    vSemaphoreDelete( pxBarrierHandle->xCounterMutex );
    vSemaphoreDelete( pxBarrierHandle->xBarrierSemaphore );
    #if ( configUSE_SPATIAL_REDUNDANCY == 1 )
        vSemaphoreDelete( pxBarrierHandle->xRemoteCounterMutex );
    #endif

    vPortFree( pxBarrierHandle );

    taskEXIT_CRITICAL();

    pxBarrierHandle = NULL;
}

void vBarrierTimerCallback( TimerHandle_t xTimer )
{
    BaseType_t errorCode;
    callbackContainer_t * xCallbackContainer = NULL;

    xCallbackContainer = ( callbackContainer_t * ) pvTimerGetTimerID( xTimer );
    if ( xCallbackContainer && xCallbackContainer->pvFailureFunc )
    {
        /* Execute failure function from the timer daemon thread */
        xCallbackContainer->pvFailureFunc();
    }

    taskENTER_CRITICAL();

    /* With one or more threads blocked, reset the task */
    if ( xCallbackContainer->xTaskToReset )
    {
        errorCode = xTaskReset( xCallbackContainer->xTaskToReset );
    }

    /* Task reset should never fail since the same memory gets reallocated immediately */
    ( void ) errorCode;
    taskEXIT_CRITICAL();
}

/* Spatial redundancy barrier addon: receive from can continuously and synchronize local barrier */
#if ( configUSE_SPATIAL_REDUNDANCY == 1 )

void vBarrierCANReceive( barrierHandle_t * pxBarrierHandle, BaseType_t xTaskState, UBaseType_t uxExecCount, uint32_t uxTaskID )
{
    CANSyncMessage_t xMessage = {0};
    BaseType_t message_sent = pdFAIL;
    BaseType_t messages_received = pdFAIL;

    /* Send out the sync message to the other nodes */
    xMessage.uxMessageType = CAN_MESSAGE_SYNC;
    xMessage.uxTaskState = xTaskState;
    xMessage.uxExecCount = uxExecCount;
    xMessage.uxID = uxTaskID;

    message_sent = xCANSendSyncMessage( &xMessage );

    /* Set up remote counter to current value of nodes */
    pxBarrierHandle->uxRemoteCounter = uxDetectedNodes;

    for( ; ; )
    {
        /* Receive messages continously */
        messages_received = xCANReceiveSyncMessages();
        if ( messages_received == pdFAIL )
        {
            /* Failed while receiving message - resume local execution */
            break;
        }

        /* Break from the loop if the barrier is released */
        #if 0
        if ( !pxBarrierHandle->uxRemoteCounter )
        {
            /* Return back to 'instance done' function */
            break;
        }
        #endif

        /* Re-send local status */
        if( message_sent == pdFAIL )
        {
            message_sent = xCANSendSyncMessage( &xMessage );
        }

        /* Yield this task */
        taskYIELD();
    }
}

void vBarrierCANSynchronize( barrierHandle_t * pxBarrierHandle )
{
    if( xSemaphoreTake( pxBarrierHandle->xRemoteCounterMutex, portMAX_DELAY ) )
    {
        pxBarrierHandle->uxRemoteCounter--;
        xSemaphoreGive( pxBarrierHandle->xRemoteCounterMutex );
    }
}

#endif /* configUSE_SPATIAL_REDUNDANCY */
