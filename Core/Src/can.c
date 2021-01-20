/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */

/* CAN receive queue pointer. This pointer is passed to FreeRTOS
 * where the queue content is accessed. */
QueueHandle_t receive_queue;

/* Assign an ID to the node */
uint32_t can_node_id;

/* Node role selection */
uint32_t can_node_role;

/* USER CODE END 0 */

CAN_HandleTypeDef hcan2;

/* CAN2 init function */
void MX_CAN2_Init(void)
{

  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 16;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_10TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_5TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = ENABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspInit 0 */

  /* USER CODE END CAN2_MspInit 0 */
    /* CAN2 clock enable */
    __HAL_RCC_CAN2_CLK_ENABLE();
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN2 GPIO Configuration
    PB12     ------> CAN2_RX
    PB13     ------> CAN2_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* CAN2 interrupt Init */
    HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN2_RX1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN2_RX1_IRQn);
  /* USER CODE BEGIN CAN2_MspInit 1 */

  /* USER CODE END CAN2_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspDeInit 0 */

  /* USER CODE END CAN2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN2_CLK_DISABLE();
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN2 GPIO Configuration
    PB12     ------> CAN2_RX
    PB13     ------> CAN2_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_12|GPIO_PIN_13);

    /* CAN2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
    HAL_NVIC_DisableIRQ(CAN2_RX1_IRQn);
  /* USER CODE BEGIN CAN2_MspDeInit 1 */

  /* USER CODE END CAN2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/**
  * @brief Performs all the steps required to start CAN2 peripheral.
  * The priority of receive interrupt must be logically lowered, so it
  * can use "FromISR" routines of FreeRTOS.
  */
void CAN2_Init(void)
{
  CAN_FilterTypeDef can_filter;

  /* Set up CAN2 transciever and interrupts */
  MX_CAN2_Init();

  /* Set up filters for fif0 */
  can_filter.FilterActivation = ENABLE;
  can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter.FilterIdHigh = 0x0000;
  can_filter.FilterIdLow = 0x0000;
  can_filter.FilterMaskIdHigh = 0x0000;
  can_filter.FilterMaskIdLow = 0x0000;
  can_filter.FilterBank = 0;
  can_filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  HAL_CAN_ConfigFilter(&hcan2, &can_filter);

  /* Disable subpriority bits using this HAL->CMSIS call */
  HAL_NVIC_SetPriorityGrouping(0);

  /* Set priority to logically lower than MAX_SYSCALL_INTERRUPT_PRIORITY */
  HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
  HAL_NVIC_SetPriority(CAN2_RX1_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(CAN2_RX1_IRQn);

  /* Set up notifications for the interrupt mode: message pending in FIF0 */
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);

  /* Activate node on the bus */
  HAL_CAN_Start(&hcan2);
}

/**
  * @brief Send a message using CAN2 peripheral.
  * This function must map the CAN message structure
  * defined in FreeRTOS to the HAL compatible data structure,
  * in this case an array of bytes. After that the blocking
  * HAL TX function is called.
  */
long CAN2_Send(CANSyncMessage_t * message)
{
  long error_code = 0;
  uint8_t send_buffer[7] = {0};
  uint32_t tx_mailbox_number = 0;
  CAN_TxHeaderTypeDef header;

  send_buffer[0] = message->uxMessageType;
  send_buffer[1] = message->uxTaskState;
  send_buffer[2] = message->uxCANNodeRole;
  memcpy(&send_buffer[3], &message->uxID, 4);

  /* Set up the HAL TX handle */
  header.StdId = can_node_role;     /* Set up unique ID for each node - frame wise */
  header.DLC = CAN_MESSAGE_BYTES;   /* Send 7 bytes*/
  header.IDE = CAN_ID_STD;          /* Standard identifier */
  header.RTR = CAN_RTR_DATA;        /* Can remote transmission request */

  /* Add message to the queue */
  error_code = HAL_CAN_AddTxMessage(&hcan2, &header, send_buffer, &tx_mailbox_number);

  /* Flip the GPIO state */
  gpio_led_toggle(LED6_BLUE_ID);

  /* Return code must be correctly mapped to FreeRTOS pdPASS/pdFAIL defines */
  if (error_code == HAL_OK) {
    return 1;
  } else {
    return 0;
  }
}

/**
  * @brief This function gets called when CAN2 interrupt
  * occurs. It should copy the received data to the receive
  * queue available in FreeRTOS, after first copying the message
  * to the FreeRTOS message structure.
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  int error_code = 0;
  CANSyncMessage_t can_message = {0};
  uint8_t receive_buffer[7];
  CAN_RxHeaderTypeDef receive_header;

  /* New message has arrived to FIF0_0, move it to the FreeRTOS queue */
  error_code = HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &receive_header, receive_buffer);
  if (error_code != HAL_OK) {
    xCANReceiveErrno = CAN_STATUS_FAILED;
    return;
  }

  /* Check if queue is full */
  error_code = xQueueIsQueueFullFromISR(receive_queue);
  if (error_code == pdTRUE) {
    xCANReceiveErrno = CAN_STATUS_FAILED;
    return;
  }

  /* Convert uint8_t array to CAN message struct */
  can_message.uxMessageType = receive_buffer[0];
  can_message.uxTaskState = receive_buffer[1];
  can_message.uxCANNodeRole = receive_buffer[2];

  /* Use memcpy to extract id (uint32_t) from the buffer */
  memcpy(&can_message.uxID, &receive_buffer[3], 4);

  /* Flip the GPIO state */
  gpio_led_toggle(LED4_GREEN_ID);

  /* Send to the back of the receive queue */
  xQueueSendToBackFromISR(receive_queue, &can_message, NULL);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  /* Use only Fifo0, so this callback should not be executed */
  (void) hcan;
  while(1);
}

/**
  * @brief Stop the CAN2 peripheral, disable the receive
  * interrupt callback and disable the peripheral.
  */
void CAN2_DeInit(void)
{
  /* Stop sending and receiving messages */
  HAL_CAN_Stop(&hcan2);

  /* Deactivate notification */
  HAL_CAN_DeactivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);

  /* Wrap around CubeMX generated function */
  HAL_CAN_MspDeInit(&hcan2);
}

/**
  * @brief Create message queues and pass their pointers to FreeRTOS, along
  * with pointers to CAN related functions. The ID parameter is used to
  * signify priority in each message, and it should be different between
  * boards.
  */
void CAN2_Register(void)
{
  size_t element_size;
  CANHandlers_t canHandlers;

  /* This might not be ideal if you have boards of the same revision */
  can_node_id = HAL_GetREVID();
  canHandlers.uxNodeID = can_node_id;

  /* ###########################
   * TEST SETUP SPECIFIC CONFIGURATION
   * ###########################
   */
  if (can_node_id == 8192) {
    can_node_role = CAN_NODE_PRIMARY;
  } else {
    can_node_role = CAN_NODE_SECONDARY;
  }

  /* Primary/secondary node selection (only one node can be primary) */
  canHandlers.uxCANNodeRole = can_node_role;

  /* Create send and receive queues; no checks */
  element_size = xCANElementSize();
  receive_queue = xQueueCreate(CAN_QUEUE_LENGTH, element_size);

  /* Register CAN handlers for FreeRTOS */
  canHandlers.pvCANInitFunc = CAN2_Init;
  canHandlers.pvCANDeInitFunc = CAN2_DeInit;
  canHandlers.pvCANSendFunc = CAN2_Send;

  vCANRegister(&canHandlers, receive_queue);
}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
