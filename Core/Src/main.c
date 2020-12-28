/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

char messageBuffer[256];

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define SERIAL_PRINT(FORMAT,...) \
  lightFormat(messageBuffer, FORMAT "\r\n", ##__VA_ARGS__ ); \
  USART1_SendString(messageBuffer);

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

TaskHandle_t  blinkyHandle;
TimerHandle_t blinkyTimerHandle;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

 void vTimerCallback(TimerHandle_t xTimer)
 {
  xTimerStop(xTimer, 0);

  /* Resume the blinky task from timer callback */

  SERIAL_PRINT("Unblocking task!");
  vTaskResume(blinkyHandle);
 }

void blinkyFailureTest(void)
{
  /* Dummy failure function that blinks the leds.
     This function will be executed by a single thread. */
  const TickType_t xDelay = 200/portTICK_RATE_MS;
  int blinkTimes = 10;

  while(blinkTimes) {
    gpio_led_state(LED3_ORANGE_ID, 1);
    gpio_led_state(LED5_RED_ID, 1);
    gpio_led_state(LED4_GREEN_ID, 1);
    gpio_led_state(LED6_BLUE_ID, 1);

    vTaskDelay(xDelay);

    gpio_led_state(LED3_ORANGE_ID, 0);
    gpio_led_state(LED5_RED_ID, 0);
    gpio_led_state(LED4_GREEN_ID, 0);
    gpio_led_state(LED6_BLUE_ID, 0);

    vTaskDelay(xDelay);
    blinkTimes--;
  }
  SERIAL_PRINT("Done running failure handle!");
}

static void blinkTask(void *pvParameters)
{
  /* Time redundant blink task used to demonstrate the basic behaviour of
   * a time redundant task running with two instances. Each instance will blink
   * its own LEDs.
   */
  (void) pvParameters;
  const TickType_t xDelay = 1000/portTICK_RATE_MS;

  /* Get the running instance number and store it on the instance stack */
  BaseType_t instanceNumber = xTaskGetInstanceNumber();

  while (1) {

    if ( instanceNumber % 2 == 0 ) {
      gpio_led_state(LED3_ORANGE_ID, 1);
      gpio_led_state(LED5_RED_ID, 0);
     } else {
      gpio_led_state(LED4_GREEN_ID, 1);
      gpio_led_state(LED6_BLUE_ID, 0);
    }

    vTaskDelay(xDelay);

    if ( instanceNumber % 2 == 0 ) {
      gpio_led_state(LED3_ORANGE_ID, 0);
      gpio_led_state(LED5_RED_ID, 1);
    } else {
      gpio_led_state(LED4_GREEN_ID, 0);
      gpio_led_state(LED6_BLUE_ID, 1);
    }

    vTaskDelay(xDelay);

    SERIAL_PRINT("Instance %d done", instanceNumber);
    xTaskInstanceDone( instanceNumber );

    xTimerReset(blinkyTimerHandle, 0);
    vTaskCallAPISynchronized( blinkyHandle, vTaskSuspend );
  }
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  int error = 0;

  /* Start up the peripherals */
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  HAL_UART_MspInit(&huart1);
  MX_USART1_UART_Init();

  SERIAL_PRINT(INIT_MSG);

  /* Create a simple blinky demonstration task */
  error = xTaskCreate(blinkTask, (const char *) "Blinky", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES-2, &blinkyHandle, pdMS_TO_TICKS(5000));
  if (error <= 0) {
    while(1);
  }

  /* Demonstration failure handle is registered */
  vTaskRegisterFailureCallback( blinkyHandle, &blinkyFailureTest );

  /* Task unblock timer */
  blinkyTimerHandle = xTimerCreate("Timer2", pdMS_TO_TICKS(7000), pdTRUE, ( void * ) 0, vTimerCallback);

  vTaskStartScheduler();
}

/* USER CODE END 0 */

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
