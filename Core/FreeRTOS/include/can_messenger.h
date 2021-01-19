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
#include "barrier.h"

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

/* Number of nodes to be expected */
#define CAN_MAXIMUM_NUMBER_OF_NODES configCAN_NODES

/* These errno variables can be set inside the HAL CAN routines if the
 * messages rx/tx fails on the hardware level
 */
extern volatile BaseType_t xCANReceiveErrno;
extern volatile BaseType_t xCANSendErrno;

/* Number of detected CAN nodes must be available outside of this module,
 * for barrier functions
 */
extern UBaseType_t uxCANDetectedNodes;

/* CAN status */
enum can_transciever_states
{
    CAN_STATUS_FAILED,              /*< CAN transciever has failed */
    CAN_STATUS_OK,                  /*< CAN transciever was initialized */
};

/* CAN node role: primary or secondary */
enum can_node_role
{
    CAN_NODE_PRIMARY,               /*< This node controls the task result arbitration procedure */
    CAN_NODE_SECONDARY,             /*< This node receives the result of majority voting */
};

/* CAN message types: Correspond to various states of task instances */
enum can_message_codes
{
    CAN_MESSAGE_FAIL,               /*< If fail was received, the has entered the failure handle / reset */
    CAN_MESSAGE_STARTUP,            /*< Scheduler start sync message */
    CAN_MESSAGE_STOP,               /*< Scheduler was stopped instance */
    CAN_MESSAGE_SYNC,               /*< All local instances done synchronization */
    CAN_MESSAGE_ARBITRATION,        /*< Sends out a task success/failure message */
};

/*
 * CAN message struct is used to deliver information related to
 * syncrhonization of redundant tasks between different CPUs
 */
typedef struct
{
    uint8_t uxMessageType;          /*< Contains info on the message type */
    uint8_t uxTaskState;            /*< Local task instance state, defined in projdefs.h */
    uint8_t uxCANNodeRole;          /*< Identifies the node role on the CAN bus */
    uint32_t uxID;                  /*< Task result: a single unsigned integer representing the exeuction result OR Node ID: CAN node ID */
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
    UBaseType_t uxCANStatus;        /*< CAN transciver status */
    UBaseType_t uxCANNodeRole;      /*< CAN role node: primary/secondary */
    CANHandle_t pvCANInitFunc;      /*< Transciever initialization function pointer */
    CANHandle_t pvCANDeInitFunc;    /*< Transciever stop function pointer */
    CANSendHandle_t pvCANSendFunc;  /*< Send message function pointer */
    uint32_t uxNodeID;              /*< 32 bit node identifier */
} CANHandlers_t;

/**
 * can_messenger.h
 * <pre>
 * BaseType_t xCANMessengerInit( void );
 * </pre>
 *
 * Called from the vTaskSchedulerStart to start the CAN transciever and
 * prepare the variables for sending and receiving the messages.
 * If vCANRegister was not called from the application layer to provide
 * the right CAN options, this function will fail.
 *
 * If this function fails, the scheduler will not start.
 *
 * @return pdPASS if all CAN related parameters were assigned
 *
 */
BaseType_t xCANMessengerInit( void );

/**
 * can_messenger.h
 * <pre>
 * void vCANMessengerDeinit( void );
 * </pre>
 *
 * Called from the scheduler end function to stop the can transciever
 * and reset the send/receive queues.
 *
 */
void vCANMessengerDeinit( void );

/**
 * can_messenger.h
 * <pre>
 * void vCANRegister( CANHandlers_t * pxHandlers,
                      QueueHandle_t xSendQueue,
                      QueueHandle_t xReceiveQueue );
 * </pre>
 *
 * Register the CAN functionality by passing CAN related
 * function pointers. The functions should be implemented in the application
 * layer by the FreeRTOS user. Additionally the unique 32bit identifier for
 * this node must be provided, along with a node role integer - specifies
 * id the node will be doing the arbitration (primary) or it receives the
 * 'correct' result from the remote node.
 *
 * @param pxHandlers Pointer to CAN handlers array containing function pointers
 *
 * @param xSendQueue Send queue pointer, it must be created in the application layer
 *
 * @param xReceiveQueue Receive queue pointer, created in the application layer
 *
 */
void vCANRegister( CANHandlers_t * pxHandlers,
                   QueueHandle_t xSendQueue,
                   QueueHandle_t xReceiveQueue );

/**
 * can_messenger.h
 * <pre>
 * BaseType_t xCANSendStartStopMessage( UBaseType_t uxMessageIsStartup );
 * </pre>
 *
 * Send the CAN start message to the bus. All the other nodes should respond
 * with their start messages shortly after reception.
 *
 * @param uxMessageIsStartup pdPASS if the message is the CAN startup message
 *
 * @return pdPASS if the message was sent, or pdFAIL if it wasn't
 *
 */
BaseType_t xCANSendStartStopMessage( UBaseType_t uxMessageIsStartup );

/**
 * can_messenger.h
 * <pre>
 * BaseType_t xCANSendSyncMessage( CANSyncMessage_t * pxMessage );
 * </pre>
 *
 * Sends the CAN syncrhonization or arbitration message based
 * on the current task state.
 *
 * @param pxMessage Pointer to CAN message struct
 *
 * @return pdPASS if the message was sent, or pdFAIL if it wasn't
 *
 */
BaseType_t xCANSendSyncMessage( CANSyncMessage_t * pxMessage );

/**
 * can_messenger.h
 * <pre>
 * BaseType_t xCANReceiveSyncMessages( void );
 * </pre>
 *
 * Receives and handles the CAN messages in the FreeRTOS layer, by dequeueing
 * the receive queue.
 *
 * @return pdFAIL if the message reception has failed at some point
 *
 */
BaseType_t xCANReceiveSyncMessages( void );

/**
 * can_messenger.h
 * <pre>
 * UBaseType_t xCANElementSize( void );
 * </pre>
 *
 * Get CAN structure size from the application layer.
 *
 * @return CAN struct size
 *
 */
UBaseType_t xCANElementSize( void );

/**
 * can_messenger.h
 * <pre>
 * void vCANSendReceive( barrierHandle_t * pxBarrierHandle, CANSyncMessage_t * pxMessage );
 * </pre>
 *
 * This function should be called by a single thread of a redundant task,
 * after making sure all the local threads have reached the barrier.
 * This function will then send a message to the remote nodes running the
 * same tasks to synchronize with them and periodically receive messages
 * from the remote nodes to change the state of local tasks.
 *
 * @param pxBarrierHandle Barrier handle is used to access the information
 * related to the barrier.
 *
 * @param pxMessage Pointer to synchronization message to be sent out
 *
 */
void vCANSendReceive( barrierHandle_t * pxBarrierHandle, CANSyncMessage_t * pxMessage );

/**
 * can_messenger.h
 * <pre>
 * void vCANRemoteSignal( barrierHandle_t * pxBarrierHandle, BaseType_t xIsArbitrationMessage );
 * </pre>
 *
 * This function is called from task.c to increment the remote
 * barrier counter. Remote counter serves as a way of syncronizing
 * the barriers across CPUs interconnected by CAN bus.
 *
 * @param pxBarrierHandle Barrier handle is used to access the information
 * related to the barrier.
 *
 * @param xIsArbitrationMessage If the message is arbitration, this should be set to pdTRUE
 * to release the barrier immediately.
 *
 */
void vCANRemoteSignal( barrierHandle_t * pxBarrierHandle, BaseType_t xIsArbitrationMessage );

/**
 * can_messenger.h
 * <pre>
 * BaseType_t xCANPrimaryNode( void );
 * </pre>
 *
 * Used to check if this node is registered as the primary node
 *
 * @return pdTRUE if primary, pdFALSE otherwise
 *
 */
BaseType_t xCANPrimaryNode( void );

#endif /* __CAN_MESSENGER_H_ */
