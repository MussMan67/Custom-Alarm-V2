/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SKIP_Pin GPIO_PIN_0
#define SKIP_GPIO_Port GPIOC
#define SKIP_EXTI_IRQn EXTI0_IRQn
#define TIME_Pin GPIO_PIN_1
#define TIME_GPIO_Port GPIOC
#define VOLU_Pin GPIO_PIN_2
#define VOLU_GPIO_Port GPIOC
#define STOP_Pin GPIO_PIN_3
#define STOP_GPIO_Port GPIOC
#define STOP_EXTI_IRQn EXTI3_IRQn
#define LED_DATA_Pin GPIO_PIN_1
#define LED_DATA_GPIO_Port GPIOA
#define RUMB_Pin GPIO_PIN_2
#define RUMB_GPIO_Port GPIOA
#define MUTE_Pin GPIO_PIN_4
#define MUTE_GPIO_Port GPIOA
#define MUTE_EXTI_IRQn EXTI4_IRQn
#define USER_LED_Pin GPIO_PIN_5
#define USER_LED_GPIO_Port GPIOA
#define LED_CLK_Pin GPIO_PIN_6
#define LED_CLK_GPIO_Port GPIOA
#define LED_LATCH_Pin GPIO_PIN_7
#define LED_LATCH_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
