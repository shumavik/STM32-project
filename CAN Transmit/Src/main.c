/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
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
CAN_HandleTypeDef hcan1;

CRC_HandleTypeDef hcrc;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */
uint8_t str_Tx[24] = {0};
uint8_t str_Rx[24] = {0};
uint8_t CAN_buf[8] = {0};
uint8_t SPI_str_Rx[24] = {0};
int flag = 0;
int USB_flag = 0;
int CAN_flag = 0;
int count = 0;
int End_receive = 0;
CAN_FilterTypeDef sFilterConfig;
CAN_RxHeaderTypeDef RxHeader;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_CRC_Init(void);
static void MX_SPI1_Init(void);
static void MX_UART4_Init(void);
/* USER CODE BEGIN PFP */
void CAN1_Tx(void);
void MK_Processing(uint8_t *s); // Перевод в префиксный код
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi == &hspi1)
	{
		if (hspi1.RxXferCount == 0)
		{
			flag = 1;
		}
	}
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{	
	HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, CAN_buf);
	HAL_CAN_DeactivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	for (int i = 0; i < 8; i++)
	{
		str_Rx[count] = CAN_buf[i];
		count++;
	}
	if (count == 24)
	{
		CAN_flag = 1;
		count = 0;
	}
	End_receive = 1;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint32_t Rx_Message_CRC[6] = {0};
	uint32_t Tx_Message_CRC[5] = {0};
	uint32_t CRC_Rx;
	uint32_t CRC_Tx;
  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_CRC_Init();
  MX_SPI1_Init();
  MX_USB_DEVICE_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */
	// Настройка фильтра приема
	sFilterConfig.FilterBank = 0; // Выбор банка (всего банков 14)
	sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0; // Привязка буфера на прием
	sFilterConfig.FilterIdHigh = 0x245<<5; // Максимальный ID сообщения в списке 
	sFilterConfig.FilterIdLow = 0x0000; // Минимальный ID сообщения в списке
	sFilterConfig.FilterMaskIdHigh = 0x0000; // Максимальная битовая маска для ID, аналогично ethernet
	sFilterConfig.FilterMaskIdLow = 0x0000; // Минимальная битовая маска для ID
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; // Формат фильтра
	sFilterConfig.FilterActivation = CAN_FILTER_ENABLE; // Разрешаем работу фильтра
	
	
	HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig); // Применение настройки фильтра для CAN1
	
	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		HAL_SPI_Receive_IT(&hspi1, SPI_str_Rx, 24);
		if (flag == 1)
		{
			CAN1_Tx(); // Передача по CAN
			flag = 0;
		}
		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == 1 && CAN_flag == 1)
			{
				HAL_Delay(500);
				if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == 0)
				{
					CDC_Transmit_FS(str_Rx, 24);
				}
			}
		
		HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, str_Rx); // Прием по CAN
		if (CAN_flag == 1)
		{
			// Подготовка к вычислению контрольной суммы принятого собщения
				for (int i = 0, j = 0; i < 6; i++, j+=4)
						{
							Rx_Message_CRC[i] = str_Rx[j+3] | (str_Rx[j+2] << 8) | (str_Rx[j+1] << 16) | (str_Rx[j] << 24);
						}
				
				CRC_Rx = HAL_CRC_Calculate(&hcrc, Rx_Message_CRC, 6); // Вычисление CRC принимаемого сообщения
						
				if (CRC_Rx == 0) // Если CRC(data+CRC) == 0, то ошибок при передаче не было
				{
						// Успешный прием
						HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_13);
						HAL_Delay(2000);
						HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_13);
						/*
							МК	Обработка
						*/
						for (int i = 0; i < 20; i++)
						str_Tx[i] = str_Rx[i];
						
						MK_Processing(str_Tx);
					
						for (int i = 0, j = 0; i < 5; i++, j+=4)
						{
							Tx_Message_CRC[i] = str_Tx[j+3] | (str_Tx[j+2] << 8) | (str_Tx[j+1] << 16) | (str_Tx[j] << 24);
						}
						
						// Формируем новое сообщение с новым CRC
						CRC_Tx = HAL_CRC_Calculate(&hcrc, Tx_Message_CRC, 5);
						for (int i = 20, j = 24; i < 24 ; i++, j-=8)
						str_Tx[i] = (uint8_t)( CRC_Tx >> j);
						HAL_UART_Transmit(&huart4, str_Tx, 24, 1500);
				}
				else
				{
					for (int i = 0; i < 10; i++)
					{
						HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_13);
						HAL_Delay(500);
					}
				}
				CAN_flag = 0;
		}
		HAL_Delay(100);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV6;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 2;
  hcan1.Init.Mode = CAN_MODE_SILENT_LOOPBACK;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_SLAVE;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_LSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PG13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void MK_Processing(uint8_t *s)
{
	uint8_t buf = 0;
	int i = 19;
	while (s[i] >= '0' && s[i] <= '9')
	{
		buf = s[i];
		for (int j = 19; j > 0; j--)
		s[j] = s[j-1];
		s[0] = buf;
	}
}
void CAN1_Tx(void)
{
	CAN_TxHeaderTypeDef TxHeader;
	
	//uint8_t StrTransmit[5] = {"HELLO"};
	
	uint32_t TxMailBox;
	uint8_t buf[8] = {0};
	int k = 8;
	
	memcpy(buf, SPI_str_Rx, 8);
	TxHeader.DLC = 8;
	TxHeader.StdId = 0x001;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	
	for (int i = 0; i < 3; i++)
	{
		if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, buf, &TxMailBox) != HAL_OK)
		{
			Error_Handler();
		}
		while (End_receive != 1)
		HAL_Delay(100);	
		HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
		for (int i = 0; i < 8 ; i++)
		{
			buf[i] = SPI_str_Rx[k];
			k++;
		}
		End_receive = 0;
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
