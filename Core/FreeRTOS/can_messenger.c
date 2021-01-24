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

#include "can_messenger.h"


/* Receive errno is set inside the CAN receive interrupt if an error occured. */
volatile BaseType_t xCANReceiveErrno = 0;

/* Global CAN receive queue pointer, must be prepared in the application layer */
static QueueHandle_t xCANReceiveQueue = NULL;

/* CAN handlers for this platform are stored in this global structure */
static CANHandlers_t xCANHandlers = { 0 };

/* Keep track of the nodes detected on the CAN bus */
static SemaphoreHandle_t xCANNodesMutex = NULL;
static UBaseType_t uxCANDetectedNodes = 0;
static uint32_t prvNodeIDArray[ CAN_MAXIMUM_NUMBER_OF_NODES - 1 ] = { 0 };

/* CAN timer is used to prevent the kernel from hanging while waiting for CAN messages */
static TimerHandle_t xCANTimer = NULL;
static void prvCANTimeoutCallback( TimerHandle_t xTimer );


BaseType_t xCANMessengerInit( void )
{
    BaseType_t error_code = pdFAIL;
    BaseType_t messages_received = pdFAIL;

    /* Check if queue is created */
    if( !xCANReceiveQueue || uxQueueMessagesWaiting( xCANReceiveQueue ) )
    {
        return error_code;
    }

    /* Check if the handlers were properly registered */
    if( !xCANHandlers.pvCANInitFunc || !xCANHandlers.pvCANDeInitFunc || !xCANHandlers.pvCANSendFunc )
    {
        return error_code;
    }

    configASSERT( xCANHandlers.xCANTimeout > 0 );

    xCANTimer = xTimerCreate( "CAN Timer", xCANHandlers.xCANTimeout, pdFALSE, NULL, prvCANTimeoutCallback );

    if( !xCANTimer )
    {
        return error_code;
    }

    /* CAN node globals protection mutex */
    xCANNodesMutex = xSemaphoreCreateMutex();

    if( !xCANNodesMutex )
    {
        return error_code;
    }

    uxCANDetectedNodes = 0;

    /* Run the CAN init function */
    if( xCANHandlers.uxCANStatus != CAN_STATUS_OK )
    {
        xCANHandlers.pvCANInitFunc();
        xCANHandlers.uxCANStatus = CAN_STATUS_OK;
    }

    /* Send a CAN init message */
    xCANSendStartStopMessage( pdTRUE );

    for( ; ; )
    {
        /* Process messages to store info on other nodes */
        messages_received = xCANReceiveSyncMessages();

        if( messages_received == pdFALSE )
        {
            break;
        }

        /* Break from the loop when all the remote nodes are detected */
        if( uxCANDetectedNodes == CAN_MAXIMUM_NUMBER_OF_NODES - 1 )
        {
            break;
        }
    }

    error_code = pdPASS;
    return error_code;
}

void vCANMessengerDeinit( void )
{
    /* Send a can stop message */
    xCANSendStartStopMessage( pdFALSE );

    /* Stop the can transciever */
    xCANHandlers.pvCANDeInitFunc();

    xQueueReset( xCANReceiveQueue );
    xTimerDelete( xCANTimer, 0 );

    xCANHandlers.uxCANStatus = CAN_STATUS_FAILED;
}

void vCANRegister( CANHandlers_t * pxHandlers,
                   QueueHandle_t xReceiveQueue )
{
    taskENTER_CRITICAL();
    /* Store pointers to queues */
    xCANReceiveQueue = xReceiveQueue;

    /* Assign handles to the global CAN handles structre */
    xCANHandlers.uxCANStatus = CAN_STATUS_FAILED;
    xCANHandlers.uxCANNodeRole = pxHandlers->uxCANNodeRole;
    xCANHandlers.uxNodeID = pxHandlers->uxNodeID;
    xCANHandlers.xCANTimeout = pxHandlers->xCANTimeout;

    xCANHandlers.pvCANInitFunc = pxHandlers->pvCANInitFunc;
    xCANHandlers.pvCANDeInitFunc = pxHandlers->pvCANDeInitFunc;
    xCANHandlers.pvCANSendFunc = pxHandlers->pvCANSendFunc;

    taskEXIT_CRITICAL();
}

UBaseType_t xCANElementSize( void )
{
    return sizeof( CANSyncMessage_t );
}

BaseType_t xCANSendStartStopMessage( UBaseType_t uxMessageIsStartup )
{
    /* Startup message is sent when starting the scheduler or
     * when another startup message is received.
     */
    BaseType_t error_code = pdFAIL;
    CANSyncMessage_t xMessage = { 0 };

    if( xCANHandlers.uxCANStatus != CAN_STATUS_OK )
    {
        return error_code;
    }

    if( uxMessageIsStartup == pdPASS )
    {
        xMessage.uxMessageType = CAN_MESSAGE_STARTUP;
    }
    else
    {
        xMessage.uxMessageType = CAN_MESSAGE_STOP;
    }

    xMessage.uxCANNodeRole = xCANHandlers.uxCANNodeRole;
    xMessage.uxID = xCANHandlers.uxNodeID;

    error_code = xCANHandlers.pvCANSendFunc( &xMessage );
    return error_code;
}

BaseType_t xCANSendSyncMessage( CANSyncMessage_t * pxMessage )
{
    /* Synchronization message is sent when the task completes.
     */
    BaseType_t error_code = pdFAIL;

    pxMessage->uxCANNodeRole = xCANHandlers.uxCANNodeRole;

    error_code = xCANHandlers.pvCANSendFunc( pxMessage );
    return error_code;
}

BaseType_t xCANReceiveSyncMessages( void )
{
    /*
     * Process messages from the receive CAN queue:
     * Draw messages from the queue, and modify task states
     * from this function. The function fails is message reception
     * has failed at some point.
     */
    BaseType_t error_code = pdPASS;

    while( uxQueueMessagesWaiting( xCANReceiveQueue ) )
    {
        CANSyncMessage_t xMessage = { 0 };
        BaseType_t i = 0;

        error_code = xQueueReceive( xCANReceiveQueue, &xMessage, 0 );

        if( error_code == pdFAIL )
        {
            return error_code;
        }

        /* Access message status */
        switch( xMessage.uxMessageType )
        {
            case CAN_MESSAGE_STARTUP:

                if( xSemaphoreTake( xCANNodesMutex, portMAX_DELAY ) )
                {
                    for( i = 0; i < CAN_MAXIMUM_NUMBER_OF_NODES - 1; i++ )
                    {
                        if( xMessage.uxID == prvNodeIDArray[ i ] )
                        {
                            break;
                        }

                        if( ( prvNodeIDArray[ i ] == 0 ) && ( xMessage.uxID != xCANHandlers.uxNodeID ) )
                        {
                            /* A new node has appeared, copy the ID */
                            prvNodeIDArray[ i ] = xMessage.uxID;
                            uxCANDetectedNodes++;
                            /* Always respond with a start message of your own */
                            xCANSendStartStopMessage( pdTRUE );
                            break;
                        }
                    }

                    xSemaphoreGive( xCANNodesMutex );
                    break;
                }

            case CAN_MESSAGE_STOP:

                if( xSemaphoreTake( xCANNodesMutex, portMAX_DELAY ) )
                {
                    for( i = 0; i < CAN_MAXIMUM_NUMBER_OF_NODES - 1; i++ )
                    {
                        if( xMessage.uxID == prvNodeIDArray[ i ] )
                        {
                            /* Remove the node from internal list */
                            prvNodeIDArray[ i ] = 0;
                            uxCANDetectedNodes--;
                            xSemaphoreGive( xCANNodesMutex );
                            break;
                        }
                    }

                    /* Ignore this message since the node is not present */
                    xSemaphoreGive( xCANNodesMutex );
                    break;
                }

            case CAN_MESSAGE_SYNC:

                /* A task finished executing - store the result or ignore
                 * the message if this is  a secondary node.
                 */
                if( xCANHandlers.uxCANNodeRole == CAN_NODE_PRIMARY )
                {
                    vTaskRemoteData( xMessage.uxTaskState, xMessage.uxID, CAN_MESSAGE_SYNC );
                }

                break;

            case CAN_MESSAGE_ARBITRATION:

                /* An arbitration message with the correct result was
                 * received. Task behaviour will be modified in tasks.c
                 */
                if( xCANHandlers.uxCANNodeRole == CAN_NODE_SECONDARY )
                {
                    vTaskRemoteData( xMessage.uxTaskState, xMessage.uxID, CAN_MESSAGE_ARBITRATION );
                }

                break;

            case CAN_MESSAGE_TIMEOUT:
                /* This node will no longer try to sync its CAN task */
                vTaskCANDisable();
                break;

            default:
                /* Something went wrong */
                error_code = pdFAIL;
                break;
        }
    }

    return error_code;
}

void vCANSendReceive( barrierHandle_t * pxBarrierHandle,
                      CANSyncMessage_t * pxMessage )
{
    BaseType_t message_sent = pdFAIL;
    BaseType_t messages_received = pdFAIL;

    if( pxMessage )
    {
        message_sent = xCANSendSyncMessage( pxMessage );
    }

    /* Set up remote counter to current value of nodes */
    if( xSemaphoreTake( pxBarrierHandle->xRemoteCounterMutex, portMAX_DELAY ) )
    {
        pxBarrierHandle->uxRemoteCounter = uxCANDetectedNodes;
        xSemaphoreGive( pxBarrierHandle->xRemoteCounterMutex );
    }

    /* Start the CAN timer before processing the message queue.
     * The timer is meant to prevent this loop from hanging while
     * waiting for synchronization or arbitration messages. CAN
     * timeout should be tight since every node executes
     * same/similar pieces of code, and the final timeout is
     * offset only by message transmission lengths and
     * cycles spent on arbitration.
     */
    xTimerStart( xCANTimer, 0 );

    for( ; ; )
    {
        /* Receive messages continously */
        messages_received = xCANReceiveSyncMessages();

        if( messages_received == pdFAIL )
        {
            break;
        }

        /* Re-send local status */
        if( ( message_sent == pdFAIL ) && pxMessage )
        {
            message_sent = xCANSendSyncMessage( pxMessage );
        }

        /* Break from the loop if the barrier is released */
        if( !pxBarrierHandle->uxRemoteCounter || ( uxCANDetectedNodes == 0 ) )
        {
            break;
        }
    }

    xTimerStop( xCANTimer, 0 );
}

void vCANRemoteSignal( barrierHandle_t * pxBarrierHandle,
                       BaseType_t xIsArbitrationMessage )
{
    if( xSemaphoreTake( pxBarrierHandle->xRemoteCounterMutex, portMAX_DELAY ) )
    {
        /* Arbitration messages grant immediate barrier release */
        if( xIsArbitrationMessage )
        {
            pxBarrierHandle->uxRemoteCounter = 0;
        }
        else
        {
            pxBarrierHandle->uxRemoteCounter--;
        }

        xSemaphoreGive( pxBarrierHandle->xRemoteCounterMutex );
    }
}

BaseType_t xCANPrimaryNode( void )
{
    if( xCANHandlers.uxCANNodeRole == CAN_NODE_PRIMARY )
    {
        return pdTRUE;
    }
    else
    {
        return pdFALSE;
    }
}

static void prvCANTimeoutCallback( TimerHandle_t xTimer )
{
    ( void ) xTimer;
    CANSyncMessage_t xMessage = { 0 };

    taskENTER_CRITICAL();

    /* Unblock the thread stuck in vCANSendReceive */
    uxCANDetectedNodes = 0;

    /* This node will no longer try to sync its CAN task */
    vTaskCANDisable();

    /* Send out the timeout message */
    xMessage.uxMessageType = CAN_MESSAGE_TIMEOUT;
    xMessage.uxID = xCANHandlers.uxNodeID;
    xCANSendSyncMessage( &xMessage );

    /*Run the user specified callback after getting the function pointer */
    taskEXIT_CRITICAL();

    if( xCANHandlers.xCANTimeoutCallback )
    {
        xCANHandlers.xCANTimeoutCallback();
    }
}

void vCANTimerCallbackSetup( TaskFailureFunction_t pvTimeoutFunc )
{
    if( !pvTimeoutFunc )
    {
        return;
    }

    xCANHandlers.xCANTimeoutCallback = pvTimeoutFunc;
}
