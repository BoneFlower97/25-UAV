/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "wireless.h"
#include "processState.h"
#include "lcdprocess.h"
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define POWER_KEY_Pin GPIO_PIN_14
#define POWER_KEY_GPIO_Port GPIOC
#define LCD_LED_Pin GPIO_PIN_0
#define LCD_LED_GPIO_Port GPIOB
#define LCD_CS_Pin GPIO_PIN_1
#define LCD_CS_GPIO_Port GPIOB
#define LCD_DS_RS_Pin GPIO_PIN_2
#define LCD_DS_RS_GPIO_Port GPIOB
#define SX_CS_Pin GPIO_PIN_10
#define SX_CS_GPIO_Port GPIOB
#define TW3_Pin GPIO_PIN_11
#define TW3_GPIO_Port GPIOB
#define TW4_Pin GPIO_PIN_12
#define TW4_GPIO_Port GPIOB
#define TW1_Pin GPIO_PIN_13
#define TW1_GPIO_Port GPIOB
#define TW2_Pin GPIO_PIN_14
#define TW2_GPIO_Port GPIOB
#define TW5_Pin GPIO_PIN_15
#define TW5_GPIO_Port GPIOB
#define TW6_Pin GPIO_PIN_8
#define TW6_GPIO_Port GPIOA
#define DIO1_Pin GPIO_PIN_9
#define DIO1_GPIO_Port GPIOA
#define DIO1_EXTI_IRQn EXTI9_5_IRQn
#define BUSY_Pin GPIO_PIN_10
#define BUSY_GPIO_Port GPIOA
#define TW7_Pin GPIO_PIN_11
#define TW7_GPIO_Port GPIOA
#define TW8_Pin GPIO_PIN_12
#define TW8_GPIO_Port GPIOA
#define BEEP_Pin GPIO_PIN_15
#define BEEP_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_3
#define LED_GPIO_Port GPIOB
#define KEY_OUT1_Pin GPIO_PIN_4
#define KEY_OUT1_GPIO_Port GPIOB
#define KEY_OUT2_Pin GPIO_PIN_5
#define KEY_OUT2_GPIO_Port GPIOB
#define KEY_OUT3_Pin GPIO_PIN_6
#define KEY_OUT3_GPIO_Port GPIOB
#define KEY_OUT4_Pin GPIO_PIN_7
#define KEY_OUT4_GPIO_Port GPIOB
#define KEY_IN2_Pin GPIO_PIN_8
#define KEY_IN2_GPIO_Port GPIOB
#define KEY_IN1_Pin GPIO_PIN_9
#define KEY_IN1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
