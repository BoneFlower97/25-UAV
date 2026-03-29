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
#include "string.h"
#include "sensors.h"
#include "wireless.h"
#include "stabilizer.h"
#include "link.h"
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
#define SPI1_CS_Pin GPIO_PIN_4
#define SPI1_CS_GPIO_Port GPIOA
#define DIO1_Pin GPIO_PIN_0
#define DIO1_GPIO_Port GPIOB
#define DIO1_EXTI_IRQn EXTI0_IRQn
#define ICM_int_Pin GPIO_PIN_1
#define ICM_int_GPIO_Port GPIOB
#define ICM_int_EXTI_IRQn EXTI1_IRQn
#define BUSY_Pin GPIO_PIN_2
#define BUSY_GPIO_Port GPIOB
#define BMP_CS_Pin GPIO_PIN_12
#define BMP_CS_GPIO_Port GPIOB
#define SX_CS_Pin GPIO_PIN_8
#define SX_CS_GPIO_Port GPIOA
#define TEST_Pin GPIO_PIN_10
#define TEST_GPIO_Port GPIOC
#define LED_STA_Pin GPIO_PIN_5
#define LED_STA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define ICM_CS_HIGH()  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET)
#define ICM_CS_LOW()   HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET)

#define BMP_CS_HIGH()  HAL_GPIO_WritePin(BMP_CS_GPIO_Port, BMP_CS_Pin, GPIO_PIN_SET)
#define BMP_CS_LOW()   HAL_GPIO_WritePin(BMP_CS_GPIO_Port, BMP_CS_Pin, GPIO_PIN_RESET)
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
