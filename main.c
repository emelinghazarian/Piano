/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define C4 0.859 // 23ms
#define D4 3.4375 // 5ms
#define E4 13.75 // 1.454ms
#define F4 13.75 // 1.454ms
#define G4 110   // 0.181ms
#define A4 440   // 0.045ms
#define B4 1760  // 0.0113
#define C5 3520  // 0.0056

float Music[25]= {C4, C4, D4, C4, F4,
				E4, C4, C4, D4, C4,
				G4, F4, C4, C4, C5,
				A4, F4, E4, D4, B4,
				B4, A4, F4, G4, F4};
int NoteTime[25]= {125, 125, 250, 250, 250,
				500, 125, 125, 250, 250,
				250, 500, 125, 125, 250,
				250, 250, 250, 250, 125,
				125, 250, 250, 250, 500};
uint8_t  HexOfSin[50]=  {0b10000000,
		   0b10010000,
		   0b10011111,
		   0b10101111,
		   0b10111101,
		   0b11001011,
		   0b11010111,
		   0b11100010,
		   0b11101100,
		   0b11110011,
		   0b11111001,
		   0b11111101,
		   0b11111111,
		   0b11111111,
		   0b11111101,
		   0b11111001,
		   0b11110011,
		   0b11101100,
		   0b11100010,
		   0b11010111,
		   0b11001011,
		   0b10111101,
		   0b10101111,
		   0b10011111,
		   0b10010000,
		   0b10000000,
		   0b01101111,
		   0b01100000,
		   0b01010000,
		   0b01000010,
		   0b00110100,
		   0b00101000,
		   0b00011101,
		   0b00010011,
		   0b00001100,
		   0b00000110,
		   0b00000010,
		   0b00000000,
		   0b00000000,
		   0b00000010,
		   0b00000110,
		   0b00001100,
		   0b00010011,
		   0b00011101,
		   0b00101000,
		   0b00110100,
		   0b01000010,
		   0b01010000,
		   0b01100000,
		   0b01101111
};

int i= 0;
int j= 0;
int debug_flag = 0;
uint32_t tim_val2 = 5;
uint32_t tim_val3 = 5;

float SampleDuration = 0;


uint8_t bit0;
uint8_t bit1;
uint8_t bit2;
uint8_t bit3;
uint8_t bit4;
uint8_t bit5;
uint8_t bit6;
uint8_t bit7;





void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	tim_val3 = __HAL_TIM_GET_COUNTER(&htim3);
	tim_val2 = __HAL_TIM_GET_COUNTER(&htim2);

    if (htim->Instance == TIM3) {
    	i++;

    	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
    	htim3.Instance->ARR = NoteTime[i]* 10;
    	SampleDuration = (1/(Music[i]*50)) * 10000;
    	htim2.Instance->ARR = SampleDuration;

    	j=0;
    	debug_flag = 1;
    }
    if (htim->Instance == TIM2) {

//    	HAL_Delay(1000);

    	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);

        j++;

    	bit0 = (HexOfSin[j] & 0x01);
    	bit1 = ((HexOfSin[j] >> 1) & 0x01);
    	bit2 = ((HexOfSin[j] >> 2) & 0x01);
    	bit3 = ((HexOfSin[j] >> 3) & 0x01);
    	bit4 = ((HexOfSin[j] >> 4) & 0x01);
    	bit5 = ((HexOfSin[j] >> 5) & 0x01);
    	bit6 = ((HexOfSin[j] >> 6) & 0x01);
    	bit7 = ((HexOfSin[j] >> 7) & 0x01);

    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, bit0);
    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, bit1);
    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, bit2);
    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, bit3);
    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, bit4);
    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, bit5);
    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, bit6);
    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, bit7);
        debug_flag = 0;
    }
}
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

	htim3.Instance->ARR = NoteTime[0];
	htim2.Instance->ARR = (1/(Music[0]*50)) * 1000;

	__HAL_TIM_SET_COUNTER(&htim3, 0);
	__HAL_TIM_SET_COUNTER(&htim2, 0);

	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Start_IT(&htim2);

	tim_val2 = __HAL_TIM_GET_COUNTER(&htim2);
	tim_val3 = __HAL_TIM_GET_COUNTER(&htim3);


  /* USER CODE END 2 */

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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 80;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV8;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1000 - 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 250-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1000 - 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 125;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, A0_Pin|A7_Pin|A6_Pin|A5_Pin
                          |A4_Pin|A3_Pin|A2_Pin|A1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pins : A0_Pin A7_Pin A6_Pin A5_Pin
                           A4_Pin A3_Pin A2_Pin A1_Pin */
  GPIO_InitStruct.Pin = A0_Pin|A7_Pin|A6_Pin|A5_Pin
                          |A4_Pin|A3_Pin|A2_Pin|A1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
