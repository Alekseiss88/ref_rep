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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RESET_Pin GPIO_PIN_0
#define RESET_GPIO_Port GPIOB
#define PS1_Pin GPIO_PIN_1
#define PS1_GPIO_Port GPIOB
#define PS0_WAKE_Pin GPIO_PIN_2
#define PS0_WAKE_GPIO_Port GPIOB
#define NSS_Pin GPIO_PIN_12
#define NSS_GPIO_Port GPIOB
#define INT_Pin GPIO_PIN_13
#define INT_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define BNO08X_PS0_WAKE_Pin PS0_WAKE_Pin
#define BNO08X_PS0_WAKE_GPIO_Port PS0_WAKE_GPIO_Port

#define BNO08X_PS1_Pin PS1_Pin
#define BNO08X_PS1_GPIO_Port PS1_GPIO_Port

#define BNO08X_RST_Pin RESET_Pin
#define BNO08X_RST_GPIO_Port RESET_GPIO_Port

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
