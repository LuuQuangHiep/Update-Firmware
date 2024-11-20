/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <malloc.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* Definitions for task1 */
osThreadId_t task1Handle;
const osThreadAttr_t task1_attributes = {
  .name = "task1",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for task2 */
osThreadId_t task2Handle;
const osThreadAttr_t task2_attributes = {
  .name = "task2",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for task3 */
osThreadId_t task3Handle;
const osThreadAttr_t task3_attributes = {
  .name = "task3",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
void blinkled01(void *argument);
void blinkled02(void *argument);
void UART(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	SCB->VTOR=FLASH_BASE|0x20000;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  HAL_FLASH_Unlock();
  FLASH_EraseInitTypeDef sector3 = {
  				 .TypeErase = FLASH_TYPEERASE_SECTORS,
  				 .Banks = FLASH_BANK_1,
  				 .Sector = FLASH_SECTOR_3,
  				 .NbSectors = 1,
  				 .VoltageRange = FLASH_VOLTAGE_RANGE_1,
  		 };
  		 uint32_t* sector_err = 0;
  		 HAL_FLASHEx_Erase(&sector3, &sector_err);
  		 char* fw_new = (char*)"veryfied";
  		 for(int i = 0; i < sizeof(fw_new); i++)
  		 {
  			 HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, 0x0800C000 +i, fw_new[i]);
  		 }
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of task1 */
  task1Handle = osThreadNew(blinkled01, NULL, &task1_attributes);

  /* creation of task2 */
  task2Handle = osThreadNew(blinkled02, NULL, &task2_attributes);

  /* creation of task3 */
  task3Handle = osThreadNew(UART, NULL, &task3_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD12 PD13 PD14 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_blinkled01 */
/**
  * @brief  Function implementing the task1 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_blinkled01 */
void blinkled01(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
    osDelay(1000);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
    osDelay(1000);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_blinkled02 */
/**
* @brief Function implementing the task2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_blinkled02 */
void blinkled02(void *argument)
{
  /* USER CODE BEGIN blinkled02 */
  /* Infinite loop */
  for(;;)
  {
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
	    osDelay(2000);
	    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
	    osDelay(2000);
  }
  /* USER CODE END blinkled02 */
}

/* USER CODE BEGIN Header_UART */
int fw_recv_done = 0;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	fw_recv_done = 1;
}

#define UART_USE_DMA
/**
* @brief Function implementing the task3 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UART */
void UART(void *argument)
{
  /* USER CODE BEGIN UART */
  /* Infinite loop */
	uint8_t* msg = (uint8_t*)"PROGRAM V21 \r\n";
	uint8_t* recv_buff[32] = {0};
	HAL_UART_Transmit(&huart2, msg, strlen((char*)msg), HAL_MAX_DELAY);
	HAL_UART_Receive_DMA(&huart2, recv_buff, sizeof(recv_buff));

  for(;;)
  {
	  if(strstr((char*)recv_buff,"\r\n") != 0)
	  {
		 HAL_UART_DMAStop(&huart2);
		 if(strstr((char*)recv_buff,"\r\n")!=0)
		 {
			 if(strstr(recv_buff,"update")!=0)
			 {
				 int new_fw_size = 0;
				 sscanf(recv_buff,"update =%d",&new_fw_size);
				 char msg_1[32] = {0};
				 sprintf(msg_1,"Plese send %d byte data:\r\n" , new_fw_size);
				 msg = (uint8_t*)msg_1;
				 HAL_UART_Transmit(&huart2, msg, strlen((char*)msg), HAL_MAX_DELAY);
				 uint8_t* new_fw = (uint8_t*)malloc(new_fw_size);
				 HAL_UART_Receive_DMA(&huart2, new_fw, new_fw_size);
				 while(fw_recv_done != 1)
					 {
						 osDelay(1);

					 }
				 HAL_FLASH_Unlock();
				 FLASH_EraseInitTypeDef sector6 = {
						 .TypeErase = FLASH_TYPEERASE_SECTORS,
						 .Banks = FLASH_BANK_1,
						 .Sector = FLASH_SECTOR_6,
						 .NbSectors = 1,
						 .VoltageRange = FLASH_VOLTAGE_RANGE_1,
				 };
				 uint32_t* sector_err = 0;
				 HAL_FLASHEx_Erase(&sector6, &sector_err);
				 for(int i = 0; i < new_fw_size; i++)
				 {
					 HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, 0x08040000 +i, new_fw[i]);
				 }
				 msg = (uint8_t*)"Dowload new success\r\n";
				 HAL_UART_Transmit(&huart2, msg, strlen((char*)msg), HAL_MAX_DELAY);
				 FLASH_EraseInitTypeDef sector7 = {
								 .TypeErase = FLASH_TYPEERASE_SECTORS,
								 .Banks = FLASH_BANK_1,
								 .Sector = FLASH_SECTOR_7,
								 .NbSectors = 1,
								 .VoltageRange = FLASH_VOLTAGE_RANGE_1,
						 };
						 HAL_FLASHEx_Erase(&sector7, &sector_err);
						 char* old_fw = (char*)0x08020000;
						 for(int i = 0; i < 131072; i++)
						 {
							 HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, 0x08060000 +i, old_fw[i]);
						 }
						 msg = (uint8_t*)"Backup...\r\n";
						 HAL_UART_Transmit(&huart2, msg, strlen((char*)msg), HAL_MAX_DELAY);
						FLASH_EraseInitTypeDef sector3 = {
										 .TypeErase = FLASH_TYPEERASE_SECTORS,
										 .Banks = FLASH_BANK_1,
										 .Sector = FLASH_SECTOR_3,
										 .NbSectors = 1,
										 .VoltageRange = FLASH_VOLTAGE_RANGE_1,
								 };
								 HAL_FLASHEx_Erase(&sector3, &sector_err);
								 char* new_interrup = (char*)"new_fw";
								 for(int i = 0; i < strlen(new_interrup); i++)
								 {
									 HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, 0x0800C000 +i, new_interrup[i]);
								 }
								 msg = (uint8_t*)"Done!\r\n";
				 				 HAL_NVIC_SystemReset();

			  }else {
				  msg = (uint8_t*)"Command not found\r\n";
			  }
			  HAL_UART_Transmit(&huart2, msg, strlen((char*)msg), HAL_MAX_DELAY);
			  memset(recv_buff, 0 , sizeof(recv_buff));
			  HAL_UART_Receive_DMA(&huart2, recv_buff, sizeof(recv_buff));
			  }
	  }
    osDelay(1);
  }
  /* USER CODE END UART */
}

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
