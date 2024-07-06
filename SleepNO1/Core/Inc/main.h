/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#define MSDriver_ENA_H_Pin GPIO_PIN_11
#define MSDriver_ENA_H_GPIO_Port GPIOF
#define MSDriver_DIR_H_Pin GPIO_PIN_13
#define MSDriver_DIR_H_GPIO_Port GPIOF
#define MSDriver_DIR_L_Pin GPIO_PIN_14
#define MSDriver_DIR_L_GPIO_Port GPIOF
#define MSDriver_PUL_H_Pin GPIO_PIN_15
#define MSDriver_PUL_H_GPIO_Port GPIOF
#define MSDriver_ENA_L_Pin GPIO_PIN_0
#define MSDriver_ENA_L_GPIO_Port GPIOG
#define GDM_UP_Pin GPIO_PIN_14
#define GDM_UP_GPIO_Port GPIOB
#define GDM_UP_EXTI_IRQn EXTI15_10_IRQn
#define GDM_DOWN_Pin GPIO_PIN_15
#define GDM_DOWN_GPIO_Port GPIOB
#define GDM_DOWN_EXTI_IRQn EXTI15_10_IRQn
#define MSDriver_PUL_L_Pin GPIO_PIN_12
#define MSDriver_PUL_L_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
