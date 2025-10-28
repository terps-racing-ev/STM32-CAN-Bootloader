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
#include "stm32l4xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* Debug LED definitions */
#define LED1_PIN                    GPIO_PIN_1      /* PB1 - Bootloader ready indicator */
#define LED1_GPIO_PORT              GPIOB
#define LED2_PIN                    GPIO_PIN_8      /* PA8 - Valid application indicator */
#define LED2_GPIO_PORT              GPIOA
#define LED3_PIN                    GPIO_PIN_5      /* PB5 - CAN activity indicator */
#define LED3_GPIO_PORT              GPIOB

/* LED control macros */
#define LED1_ON()                   HAL_GPIO_WritePin(LED1_GPIO_PORT, LED1_PIN, GPIO_PIN_SET)
#define LED1_OFF()                  HAL_GPIO_WritePin(LED1_GPIO_PORT, LED1_PIN, GPIO_PIN_RESET)
#define LED1_TOGGLE()               HAL_GPIO_TogglePin(LED1_GPIO_PORT, LED1_PIN)

#define LED2_ON()                   HAL_GPIO_WritePin(LED2_GPIO_PORT, LED2_PIN, GPIO_PIN_SET)
#define LED2_OFF()                  HAL_GPIO_WritePin(LED2_GPIO_PORT, LED2_PIN, GPIO_PIN_RESET)
#define LED2_TOGGLE()               HAL_GPIO_TogglePin(LED2_GPIO_PORT, LED2_PIN)

#define LED3_ON()                   HAL_GPIO_WritePin(LED3_GPIO_PORT, LED3_PIN, GPIO_PIN_SET)
#define LED3_OFF()                  HAL_GPIO_WritePin(LED3_GPIO_PORT, LED3_PIN, GPIO_PIN_RESET)
#define LED3_TOGGLE()               HAL_GPIO_TogglePin(LED3_GPIO_PORT, LED3_PIN)

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
