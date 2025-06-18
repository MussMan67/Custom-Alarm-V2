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
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart5;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_UART5_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

// MP3 Module
uint8_t volume = 0;
uint8_t checksum = 0;
uint8_t UART_RESET[] = {0x7E, 0xFF, 0x06, 0x0C, 0x00, 0x00, 0x00, 0xFE, 0xEF, 0xEF};
const uint8_t UART_PLAY[] = {0x7E, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x01, 0xFE, 0xF7, 0xEF};
const uint8_t UART_STOP[] = {0x7E, 0xFF, 0x06, 0x0E, 0x00, 0x00, 0x00, 0xFE, 0xED, 0xEF};
uint8_t UART_VOLUME[] = {0x7E, 0xFF, 0x06, 0x06, 0x00, 0x00, 0x00, 0xFE, 0x00, 0xEF};

//  RUMBLE MOTORS


// LED ARRAY
uint8_t LED_ARRAY[16];

// ALARM LOGIC
// the current time according to the MPU, updated every loop
uint32_t currentTime = 0;
// the total length of time between 0 and 12 hours (s)
const uint32_t totalTime = 60 * 60 * 12;
// the time at which to sound the alarm (in ms?)
uint32_t wakeUpTime = 0; //TO-DO
// boolean to track whether or not to sound the alarm
uint8_t wokeUp = 1;
// int to track the number of times the alarm has been slept through in a cycle
uint8_t oversleeps = 0;
uint32_t lastRepeatTime = 0;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


	// LED ARRAY //////////////////////////////////////////////////////////////////////////

	void CALC_LED(void) {
		uint8_t hours = round((wakeUpTime % totalTime) / totalTime * 12.0f);
		for (int i = 0; i < hours; i++) {
			LED_ARRAY[i] = 1;
		}
		for (int i = hours; i < 16; i++) {
			LED_ARRAY[i] = 0;
		}
	}

	void UPDATE_LED(void) {
		// turn on the latch pin (data is allowed to change)
		HAL_GPIO_WritePin(LED_LATCH_GPIO_Port, LED_LATCH_Pin, 1);
		// for all 12 LEDs, turn them on or off based on their corresponding values in the LED array
		// and initiate a rising clock edge so that data transfers
		for (int i = 0; i < 12; i++) {
			HAL_GPIO_WritePin(LED_CLK_GPIO_Port, LED_CLK_Pin, 0);
			HAL_GPIO_WritePin(LED_DATA_GPIO_Port, LED_DATA_Pin, LED_ARRAY[i]); // data is prepared before rising edge
			HAL_GPIO_WritePin(LED_CLK_GPIO_Port, LED_CLK_Pin, 1);
		}
		HAL_GPIO_WritePin(LED_LATCH_GPIO_Port, LED_LATCH_Pin, 0);
		// reset the latch pin (data is done being transferred)
	}

	// MP3 MODULE /////////////////////////////////////////////////////////////////////////

	void MP3_RESET(void) {
		HAL_UART_Transmit(&huart5, UART_RESET, sizeof(UART_RESET), HAL_MAX_DELAY);
	}

	void MP3_PLAY(void) {
		HAL_UART_Transmit(&huart5, UART_PLAY, sizeof(UART_PLAY), HAL_MAX_DELAY);
	}

	void MP3_STOP(void) {
		HAL_UART_Transmit(&huart5, UART_STOP, sizeof(UART_STOP), HAL_MAX_DELAY);
	}

	void MP3_VOLUME(void) {
		HAL_UART_Transmit(&huart5, UART_VOLUME, sizeof(UART_VOLUME), HAL_MAX_DELAY);
	}

	// RUMBLE MOTORS //////////////////////////////////////////////////////////////////////

	void SOUND_RUMBLE(float speed) {
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, (uint8_t) speed * 1400);
	}

	void MUTE_RUMBLE(void) {
		SOUND_RUMBLE(0);
	}

	// VOLUME POTENTIOMETER ///////////////////////////////////////////////////////////////

	uint8_t CHECKSUM_VOLUME(uint8_t volume) {
		return 0xD7 + 30 - volume;
	}

	void UPDATE_VOLUME(void) {
		// convert analog pot value to volume
		HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY);
		volume = (float) (HAL_ADC_GetValue(&hadc2)) / 64 * 30;
		// send command to MP3 Module
		UART_VOLUME[6] = volume;
		UART_VOLUME[8] = CHECKSUM_VOLUME(volume);
		MP3_VOLUME();
	}

	// TIME POTENTIOMETER ///////////////////////////////////////////////////////////////

	void UPDATE_TIME(void) {
		// convert analog pot value to time
		HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
		// set new time
		wakeUpTime = (float) (HAL_ADC_GetValue(&hadc1)) / 1024 * totalTime + currentTime;
		// light up the corresponding number of LEDs
		CALC_LED();
		UPDATE_LED();
		// TO-DO: ADD FADING OUT LOGIC; MAYBE WATCHDOG OR INTERRUPTS
	}

	// SKIP BUTTON //////////////////////////////////////////////////////////////////////

	void SKIP_ALARM(void) {
		wakeUpTime += totalTime * 2; // increment the time of alarm by one day
	}

	// SKIP BUTTON //////////////////////////////////////////////////////////////////////

	void STOP_ALARM(void) {
		wokeUp = 1; // reset wake-up tracker variable
		// pause the MP3 Module and Rumble Motors
		MP3_STOP();
		SOUND_RUMBLE(0);
		oversleeps = 0; // reset oversleeping tracker variable
	}

	// ALARM LOGIC //////////////////////////////////////////////////////////////////////

	uint8_t isWakeUpTime() {
		// if it is past wake-up time, increment wake-up time
		if (currentTime > wakeUpTime) {
			wakeUpTime += totalTime * 2;
			wokeUp = 0; // change wake-up tracker variable
			return 1;
		}
		// return whether or not the alarm should sleep or be on
		return 0;
	}

	void SOUND_ALARM(void) {
		// if it has been [the waiting times of the alarm before sounding again when oversleeping; 9, 6, and 3 minutes]
		// then resound the alarm and cue the rumble motors
		if (currentTime - lastRepeatTime > ((4 - oversleeps) % 4) * totalTime / 144.0f) {
			MP3_PLAY();
			SOUND_RUMBLE(oversleeps / 3.0f);
			oversleeps++; // change the oversleeping tracker variable
			lastRepeatTime = currentTime; // change the last repeat time tracker variable
		}
		// stop the rumble motors after 9 seconds
		if (currentTime - lastRepeatTime > totalTime / 4800.0f) {
			SOUND_RUMBLE(0);
			// once the 3rd oversleep has ended, reset everything and wait for the next day
			if (oversleeps == 4) {
				oversleeps = 0;
				wokeUp = 1;
			}
		}
	}

	// INTERRUPTS ///////////////////////////////////////////////////////////////////////

	void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
		if (GPIO_Pin == STOP_Pin) {
			MP3_PLAY();
			HAL_Delay(2000);
			STOP_ALARM();
			USER_LED_BLINK();
		}

		if (GPIO_Pin == SKIP_Pin) {
			wakeUpTime == 100;
			SKIP_ALARM();
			if (wakeUpTime > 199) {
				USER_LED_BLINK();
			}
		}

		if (GPIO_Pin == MUTE_Pin) {
			MUTE_RUMBLE();
		}
	}

	void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
		currentTime++; // increment the current-time tracker variable
	}

	// RUN MAIN PROGRAM ////////////////////////////////////////////////////////////////

	void RUN(void) {
		UPDATE_VOLUME();
		UPDATE_TIME();
	    if (isWakeUpTime() || !wokeUp) {
	    	SOUND_ALARM();
	    }
	}

	// TESTS ///////////////////////////////////////////////////////////////////////////

	void USER_LED(uint8_t bit) {
		HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, bit);
	}

	void USER_LED_BLINK(float durSeconds) {
		USER_LED(0);
		HAL_Delay(durSeconds * 1000);
		USER_LED(1);
	}

	void USER_LED_BLINK() {
		USER_LED(0);
		HAL_Delay(.5 * 1000);
		USER_LED(1);
	}


	void TEST_VOLUME(void) {
		MP3_PLAY();
		currentTime = 0;
		while (currentTime < 10) {
			UPDATE_VOLUME();
			// ADJUST PHYSICAL VOLUME KNOB
		}
		USER_LED_BLINK();
	}

	void TEST_TIME(void) {
		wakeUpTime = 0;
		HAL_Delay(3000); // MOVE PHYSICAL TIME KNOB TO 0
		if (wakeUpTime == 0) USER_LED_BLINK();

		HAL_Delay(3000); // MOVE PHYSICAL TIME KNOB TO 12
		if (wakeUpTime == 12 * 60 * 60) USER_LED_BLINK();

		HAL_Delay(3000); // MOVE PHYSICAL TIME KNOB TO 6
		if (wakeUpTime > 5 * 60 * 60 && wakeUpTime < 7 * 60 * 60) USER_LED_BLINK();
	}

	void TEST_MUTE(void) {
		SOUND_RUMBLE(.5);
		HAL_Delay(3000); // PRESS PHYSICAL MUTE SWITCH
		USER_LED_BLINK();
	}

	void TEST_SKIP(void) {
		wakeUpTime == 100;
		HAL_Delay(3000); // PRESS PHYSICAL SKIP BUTTON
		if (wakeUpTime >= 200) USER_LED_BLINK();
	}

	void TEST_STOP(void) {
		wakeUpTime == currentTime;
		SOUND_ALARM();
		HAL_Delay(3000); // PRESS PHYSICAL STOP BUTTON
		USER_LED_BLINK();
	}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	HAL_DELAY(2000);
	TEST_VOLUME();

	HAL_DELAY(2000);
	TEST_TIME();

	HAL_DELAY(2000);
	TEST_MUTE();

	HAL_DELAY(2000);
	TEST_MUTE();

	HAL_DELAY(2000);
	TEST_SKIP();

	HAL_DELAY(2000);
	TEST_STOP();
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
  MX_ADC1_Init();
  MX_UART5_Init();
  MX_ADC2_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_10B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc2.Init.Resolution = ADC_RESOLUTION_6B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 59999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  htim3.Init.Prescaler = 9999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 8399;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 9600;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */
  MP3_RESET();
  /* USER CODE END UART5_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_DATA_Pin|USER_LED_Pin|LED_CLK_Pin|LED_LATCH_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SKIP_Pin STOP_Pin */
  GPIO_InitStruct.Pin = SKIP_Pin|STOP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_DATA_Pin USER_LED_Pin LED_CLK_Pin LED_LATCH_Pin */
  GPIO_InitStruct.Pin = LED_DATA_Pin|USER_LED_Pin|LED_CLK_Pin|LED_LATCH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : MUTE_Pin */
  GPIO_InitStruct.Pin = MUTE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MUTE_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

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
