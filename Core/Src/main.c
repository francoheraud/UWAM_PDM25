/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <stdbool.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void adjust_fan(uint8_t data);
static void CAN_SendTestFrame(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;

uint8_t TxDataCAN[8];
uint8_t RxDataCAN[8];
uint32_t mailbox;

bool has_tx_failed = false;


static void CAN_SendTestFrame(void)
{
	//static uint8_t counter = 0;
	uint8_t data[8] = {0, 0, 0, 0xBE, 0, 0, 0, 0};
	uint32_t mbox;

	TxHeader.StdId 	= 0x030;
	TxHeader.IDE 	= CAN_ID_STD;
	TxHeader.RTR 	= CAN_RTR_DATA;
	TxHeader.DLC 	= 8;

	//data[0] = counter++; //changes on each send
	if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, data, &mbox) != HAL_OK)
		has_tx_failed = true;
}

void adjust_fan(uint8_t data)
{
	uint16_t CCR_Width;
	uint32_t ARR_Value = (TIM3->ARR) * data;
	CCR_Width = ARR_Value / 200u;

	TIM3->CCR3 = CCR_Width;
	TIM3->CR1 |= TIM_CR1_CEN;
	TIM3->CCER |= TIM_CCER_CC3E;
}

/*
void can_handler(uint8_t* message)
{
	uint8_t bytes_to_read = message[3]; //< nth bit determines whether or not to read nth byte
	uint8_t duty_for_adjust = (uint16_t)message * 200u / 255u;
    adjust_fan(duty_for_adjust);
}
*/


// re implemented!!
/*
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *h)
{
    uint8_t d[8];
    if (HAL_CAN_GetRxMessage(h, CAN_RX_FIFO0, &RxHeader, d) != HAL_OK)
        return;

    if (RxHeader.StdId == PDM_CONTROL_ID) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    }

    uint8_t duty_255        = d[3];
    uint8_t duty_for_adjust = (uint16_t)duty_255 * 200 / 255;
    adjust_fan(duty_for_adjust);
}
*/


//LIN CODE SHIT HERE
//uint8_t TxData[20]; //define tx buffer
//
//uint8_t pid_Calc (uint8_t ID) //function pid_calc takes ID as parameter
//{
// 	if (ID>0x3F) Error_Handler(); // ensures range of ID < 63
//	uint8_t IDBuf[6];             // declares array of 6 unsigned 8 bit integers named IDBuf
//	for (int i=0; i<6; i++)     // loop from 0<i<5, repeating for each value of i
//	{
//		IDBuf[i] = (ID>>i)&0x01;    // bitemasks the LSB and then puts that in the array, right shifts by 1 each time (see below toggle)
//	}
//	// Parity bit calculations
//	uint8_t P0 = (IDBuf[0] ^ IDBuf[1] ^ IDBuf[2] ^ IDBuf[4])&0x01; // as per datasheet computes XOR followed by bitmask for 'safety?'
//	uint8_t P1 = (~(IDBuf[1] ^ IDBuf[3] ^ IDBuf[4] ^ IDBuf[5]))&0x01; // pre sure the brackets here are in the wrong order coz this is the situation where you do need the bitmask but it wont work like this but this is what the guy wrote so idk
//
//	// chuck parity bits in the ID variable (in the buffer???)
//	ID = ID | (P0<<6) | (P1<<7); // ohh pre sure since the parity bits are 8 bit unsigned integers then you just OR all that shit together and it gives you a nice PID
//	return ID;

	//Checksum
//}
//uint8_t checksum_calc (uint8_t PID, uint8_t *data, uint8_t size) // parameters PID, pointer to the data and size
//// ok so pre sure this is creating a buffer which is literally the LIN message
//// and then it adds all that shit together and subracts 255 if over
//{
//	uint8_t buffer[size+2]; //array called buffer with size of (size + 2)
//	uint16_t sum=0;
//	buffer[0] = PID; // PID = 0th position in buffer
//	for (int i=0; i<size; i++) // increment position and perform loop each time of adding data into buffer
//	{
//		buffer[i+1] = data[i]; //adds data slot i into the i+1 index
//	}
//
//	for (int i=0; i<size+1; i++)
//	{
//		sum = sum + buffer[i]; // counts through the buffer increments
//		if (sum>0xff) sum = sum-0xff; //ensures it doesnt reach over 255
//	}
//
//	sum = 0xff-sum; //inverts result as per specification
//	return sum;

//}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
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
  MX_USART3_UART_Init();
  MX_CAN_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  HAL_CAN_Start(&hcan);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);   // PB0 (Enable6) now outputs PWM
  adjust_fan(200);
  HAL_CAN_ActivateNotification(&hcan,
		  //CAN_IT_RX_FIFO0_MSG_PENDING | // (pure polling for now)
		  CAN_IT_BUSOFF |
		  CAN_IT_ERROR_WARNING |
		  CAN_IT_ERROR_PASSIVE);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) != 0)
	 {
		 if (HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &RxHeader, RxDataCAN) == HAL_OK)
		 {
			 if (RxHeader.StdId == PDM_CONTROL_ID)
				 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
		 }
	 }
	  CAN_SendTestFrame(); //self-tx every 100ms *added 10:44
	  HAL_Delay(100);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //LIN SHIT
//	  TxData[0] = 0x55;   //assigns 0x55 (Sync Field) to
//	  TxData[1] = pid_Calc(0x39
//			  ); // send the ID (inside the function) at next position in buffer 57 is 0x39 is the frame ID in the LDF)
//	  for (int i=0; i<8; i++)
//	  {
//		  TxData[i+2] = i;
//	  }
//	  TxData[10] = checksum_calc(TxData[1], TxData+2, 8); //LIN 2.1 Checksum (
//
//	  //Now we send the the forking LIN message
//
//	  HAL_LIN_SendBreak(&huart3);
//	  HAL_UART_Transmit(&huart3, TxData, 11, 1000); //Explaination Below
//	  HAL_Delay(1000); //wait a second between sending
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART3;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 18;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = ENABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
  CAN_FilterTypeDef canfilterconfig = {0};

  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE; //Enable CAN Filter
  canfilterconfig.FilterBank = 0; //which filter bank to use from the assigned ones?
  canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  canfilterconfig.FilterIdHigh = (0x0000 << 5); //ID thats allowed to Pass (PDM ID) for LBK
  canfilterconfig.FilterIdLow = 0x0000;
  canfilterconfig.FilterMaskIdHigh = (0x0000 << 5); //ID thats allowed to Pass (PDM ID) for LBK
  canfilterconfig.FilterMaskIdLow = 0x0000;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canfilterconfig.SlaveStartFilterBank = 14; //doesnt matter in single can Controllers

  if (HAL_CAN_ConfigFilter(&hcan, &canfilterconfig)!=HAL_OK)
  {
  	Error_Handler();
  }

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7199;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 19200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_LIN_Init(&huart3, UART_LINBREAKDETECTLENGTH_11B) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, MCU_LED_Pin|ENABLE1_Pin|ENABLE2_Pin|LIN_Enable_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ENABLE3_GPIO_Port, ENABLE3_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ENABLE7_Pin|ENABLE4_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ENABLE5_GPIO_Port, ENABLE5_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : MCU_LED_Pin ENABLE1_Pin ENABLE2_Pin LIN_Enable_Pin */
  GPIO_InitStruct.Pin = MCU_LED_Pin|ENABLE1_Pin|ENABLE2_Pin|LIN_Enable_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : ENABLE3_Pin */
  GPIO_InitStruct.Pin = ENABLE3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ENABLE3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ENABLE7_Pin ENABLE4_Pin */
  GPIO_InitStruct.Pin = ENABLE7_Pin|ENABLE4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ENABLE5_Pin */
  GPIO_InitStruct.Pin = ENABLE5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ENABLE5_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//CAN



/*
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *h)
{
	uint8_t d[8];
	if (HAL_CAN_GetRxMessage(h, CAN_RX_FIFO0, &RxHeader, d) == HAL_OK)
	{
		void adjust_fan(void);
		//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	}
}

*/
/*
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *h)
{
    uint8_t d[8];
    if (HAL_CAN_GetRxMessage(h, CAN_RX_FIFO0, &RxHeader, d) == HAL_OK)
    {
    	uint8_t duty_255 = d[3];
    	uint8_t duty_for_adjust = (uint16_t)duty_255 * 200u / 255u;
        adjust_fan(duty_for_adjust);
    }
}
*/

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
#ifdef USE_FULL_ASSERT
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
