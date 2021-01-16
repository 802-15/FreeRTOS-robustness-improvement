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

#ifndef _CAN_MESSENGER_H_
#define _CAN_MESSENGER_H_

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

/* *INDENT-OFF* */
#ifdef __cplusplus
    extern "C" {
#endif
/* *INDENT-ON* */


/* CAN message length in bytes */
#define CAN_MESSAGE_BYTES 8
#define CAN_ID_BYTES 4

/* Suggested queue length, it might need to be adjusted to suit the application needs */
#define CAN_QUEUE_LENGTH configCAN_QUEUE_LENGTH

/* Maximum number of nodes to be expected: hard coded */
#define CAN_MAXIMUM_NUMBER_OF_NODES 5
#define CAN_MAXIMUM_NUMBER_OF_TASKS 10

/* This callback is set inside the CAN RX ISR */
extern volatile BaseType_t xCANReceiveErrno;
extern volatile BaseType_t xCANSendErrno;

extern UBaseType_t uxDetectedNodes;

/* CAN status */
enum can_transciever_states
{
    CAN_STATUS_FAILED,              /*< CAN transciever has failed */
    CAN_STATUS_OK,                  /*< CAN transciever was initialized */
};

/* CAN message types: Correspond to various states of task instances */
enum can_message_codes
{
    CAN_MESSAGE_FAIL,               /*< If fail was received, the has entered the failure handle / reset */
    CAN_MESSAGE_STARTUP,            /*< Scheduler start sync message */
    CAN_MESSAGE_STOP,               /*< Scheduler was stopped instance */
    CAN_MESSAGE_SYNC,               /*< All local instances done synchronization */
};

/*
 * CAN message struct is used to deliver information related to
 * syncrhonization of redundant tasks between different CPUs
 */
typedef struct
{
    uint8_t uxMessageType;          /*< Contains info on the message type */
    uint8_t uxExecCount;            /*< Execution count */
    uint8_t uxTaskState;            /*< Cumulative task instance state, defined in projdefs.h */
    uint32_t uxID;                  /*< Task ID: set as the first 4 bytes of the task name OR Node ID: CAN node ID */
} CANSyncMessage_t;

/*
 * Pointer to CAN wrapper functions - prototypes for functions to be supplied
 * from the application layer are defined here
 */
typedef BaseType_t ( * CANSendHandle_t ) ( CANSyncMessage_t * );
typedef void ( * CANHandle_t ) ( void );

/*
 * This structure contains pointers to the CAN related functions.
 * CAN transciever state is also stored here.
 */
typedef struct
{
    UBaseType_t uxCANStatus;
    CANHandle_t pvCANInitFunc;
    CANHandle_t pvCANDeInitFunc;
    CANSendHandle_t pvCANSendFunc;
} CANHandlers_t;

/* API: */

BaseType_t xCANMessengerInit( void );

void vCANMessengerDeinit( void );

void vCANRegister( CANHandlers_t * pxHandlers, QueueHandle_t xSendQueue, QueueHandle_t xReceiveQueue, uint32_t uxID );

BaseType_t xCANSendStartStopMessage( UBaseType_t uxMessageIsStartup );

BaseType_t xCANSendSyncMessage( CANSyncMessage_t * pxMessage );

BaseType_t xCANReceiveSyncMessages( void );

UBaseType_t xCANElementSize( void );

#endif /* __CAN_MESSENGER_H_ */
