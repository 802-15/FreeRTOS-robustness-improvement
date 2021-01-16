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


/* Receive errno is set up inside ISR to notify FreeRTOS if we failed to receive the message */
volatile BaseType_t xCANReceiveErrno;

/* Global CAN send and receive queue pointers, must be prepared in the application layer */
static QueueHandle_t xCANReceiveQueue;
static QueueHandle_t xCANSendQueue;

/* CAN handlers for this platform are stored in this global structure */
static CANHandlers_t xCANHandlers;

/* Keep track of the nodes detected on the CAN bus */
static uint32_t uxNodeID = 0;
static uint32_t prvNodeIDArray[CAN_MAXIMUM_NUMBER_OF_NODES] = {0};
UBaseType_t uxDetectedNodes = 0;


BaseType_t xCANMessengerInit( void )
{
    BaseType_t error_code = pdFAIL;

    /* Check if Queues are initialized */
    if ( !xCANReceiveQueue || uxQueueMessagesWaiting( xCANReceiveQueue ) )
    {
        return error_code;
    }

    if ( !xCANSendQueue || uxQueueMessagesWaiting( xCANSendQueue ) )
    {
        return error_code;
    }

    /* Check if the handlers were properly registered */
    if ( !xCANHandlers.pvCANInitFunc || !xCANHandlers.pvCANDeInitFunc || !xCANHandlers.pvCANSendFunc )
    {
        return error_code;
    }

    uxDetectedNodes = 0;

    /* Run the CAN init function */
    xCANHandlers.pvCANInitFunc();
    xCANHandlers.uxCANStatus = CAN_STATUS_OK;

    /* Send a CAN init function */
    xCANSendStartStopMessage( pdPASS );

    error_code = pdPASS;

    return error_code;
}

void vCANMessengerDeinit( void )
{
    /* Send a can stop message */
    xCANSendStartStopMessage( pdFAIL );

    /* Stop the can transciever */
    xCANHandlers.pvCANDeInitFunc();

    xQueueReset( xCANReceiveQueue );
    xQueueReset( xCANSendQueue );

    xCANHandlers.uxCANStatus = CAN_STATUS_FAILED;
}

void vCANRegister( CANHandlers_t * pxHandlers, QueueHandle_t xSendQueue, QueueHandle_t xReceiveQueue, uint32_t uxID )
{
    /* Store pointers to queues */
    xCANReceiveQueue = xReceiveQueue;
    xCANSendQueue = xSendQueue;

    /* Assign handles to the global CAN handles structre */
    xCANHandlers.uxCANStatus = CAN_STATUS_FAILED;
    xCANHandlers.pvCANInitFunc = pxHandlers->pvCANInitFunc;
    xCANHandlers.pvCANDeInitFunc = pxHandlers->pvCANDeInitFunc;
    xCANHandlers.pvCANSendFunc = pxHandlers->pvCANSendFunc;

    /* Store node ID */
    uxNodeID = uxID;
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
    CANSyncMessage_t xMessage = {0};

    if ( uxMessageIsStartup == pdPASS )
    {
        xMessage.uxMessageType = CAN_MESSAGE_STARTUP;
    }
    else
    {
        xMessage.uxMessageType = CAN_MESSAGE_STOP;
    }

    if ( !uxNodeID )
    {
        return error_code;
    }

    xMessage.uxID = uxNodeID;

    error_code = xQueueSendToBack( xCANSendQueue, &xMessage, 0 );
    if ( error_code == pdFAIL )
    {
        return error_code;
    }

    error_code = xCANHandlers.pvCANSendFunc( &xMessage );
    return error_code;
}

BaseType_t xCANSendSyncMessage( CANSyncMessage_t * pxMessage )
{
    /* Synchronization message is sent when the task completes.
     */
    BaseType_t error_code = pdFAIL;

    error_code = xQueueSendToBack( xCANSendQueue, pxMessage, 0 );
    if ( error_code == pdFAIL )
    {
        return error_code;
    }

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
    CANSyncMessage_t xMessage = {0};
    BaseType_t i = 0;

    while( uxQueueMessagesWaiting( xCANReceiveQueue ) )
    {
        error_code = xQueueReceive( xCANReceiveQueue, &xMessage, 0 );
        if ( error_code == pdFAIL )
        {
            return error_code;
        }

        /* Access message status */
        switch ( xMessage.uxMessageType )
        {
            case CAN_MESSAGE_STARTUP:
                for ( i = 0; i < CAN_MAXIMUM_NUMBER_OF_NODES; i++ )
                {
                    if ( xMessage.uxID == prvNodeIDArray[i] )
                    {
                        /* Node already registered locally, discard it */
                        break;
                    }
                }
                /* A new node has appeared, copy the ID */
                prvNodeIDArray[uxDetectedNodes] = xMessage.uxID;
                uxDetectedNodes++;
                break;

            case CAN_MESSAGE_STOP:
                for ( i = 0; i < CAN_MAXIMUM_NUMBER_OF_NODES; i++ )
                {
                    if ( xMessage.uxID == prvNodeIDArray[i] )
                    {
                        /* Remove the node from internal list */
                        prvNodeIDArray[uxDetectedNodes] = 0;
                        uxDetectedNodes--;
                        break;
                    }
                }
                /* Ignore this message since the node is not present */
                break;

            case CAN_MESSAGE_SYNC:
                /* A task finished executing - locate the redundant task by
                 * searching the ID list */
                for ( i = 0; i < CAN_MAXIMUM_NUMBER_OF_TASKS; i++ )
                {
                    if ( xMessage.uxID == prvNodeIDArray[i] )
                    {
                        vTaskModifyCANState( xMessage.uxTaskState, xMessage.uxID );
                    }
                }
                break;

            default:
                /* Something went wrong */
                error_code = pdFAIL;
                break;
        }
    }

    return error_code;
}
