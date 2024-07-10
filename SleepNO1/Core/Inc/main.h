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

//This is the timer of 4 motor drivers of the chassis
#define CHASSIS_MOTOR_PWM_DRIVER_TIM htim3
//This is the timer of speed calculating unit of the chassis
#define CHASSIS_MOTOR_CALCULATE_TIM htim5

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Pin_Pushpull_MotorDirectionA1_Pin GPIO_PIN_2
#define Pin_Pushpull_MotorDirectionA1_GPIO_Port GPIOE
#define Pin_Pushpull_MotorDirectionB1_Pin GPIO_PIN_3
#define Pin_Pushpull_MotorDirectionB1_GPIO_Port GPIOE
#define Pin_Pushpull_MotorDirectionA2_Pin GPIO_PIN_4
#define Pin_Pushpull_MotorDirectionA2_GPIO_Port GPIOE
#define Pin_Pushpull_MotorDirectionB2_Pin GPIO_PIN_5
#define Pin_Pushpull_MotorDirectionB2_GPIO_Port GPIOE
#define Pin_PullDown_HallEncoderB1_Pin GPIO_PIN_14
#define Pin_PullDown_HallEncoderB1_GPIO_Port GPIOC
#define Pin_PullDown_HallEncoderB2_Pin GPIO_PIN_15
#define Pin_PullDown_HallEncoderB2_GPIO_Port GPIOC
#define Pin_Exti_HallEncoderA1_Pin GPIO_PIN_0
#define Pin_Exti_HallEncoderA1_GPIO_Port GPIOF
#define Pin_Exti_HallEncoderA1_EXTI_IRQn EXTI0_IRQn
#define Pin_Exti_HallEncoderA2_Pin GPIO_PIN_1
#define Pin_Exti_HallEncoderA2_GPIO_Port GPIOF
#define Pin_Exti_HallEncoderA2_EXTI_IRQn EXTI1_IRQn
#define FIX_POS_DJ_Pin GPIO_PIN_8
#define FIX_POS_DJ_GPIO_Port GPIOF
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
#define MCL_GND2_Pin GPIO_PIN_10
#define MCL_GND2_GPIO_Port GPIOD
#define MCL_GND1_Pin GPIO_PIN_11
#define MCL_GND1_GPIO_Port GPIOD
#define MSDriver_PUL_L_Pin GPIO_PIN_12
#define MSDriver_PUL_L_GPIO_Port GPIOD
#define MCL_LEFT_Pin GPIO_PIN_13
#define MCL_LEFT_GPIO_Port GPIOD
#define MCL_RIGHT_Pin GPIO_PIN_14
#define MCL_RIGHT_GPIO_Port GPIOD
#define Pin_PushDJQD3V3_3_Pin GPIO_PIN_13
#define Pin_PushDJQD3V3_3_GPIO_Port GPIOG
#define Pin_PushDJQD3V3_3G15_Pin GPIO_PIN_15
#define Pin_PushDJQD3V3_3G15_GPIO_Port GPIOG
#define Pin_Exti_HallEncoderA4_Pin GPIO_PIN_3
#define Pin_Exti_HallEncoderA4_GPIO_Port GPIOB
#define Pin_Exti_HallEncoderA4_EXTI_IRQn EXTI3_IRQn
#define Pin_Exti_HallEncoderA3_Pin GPIO_PIN_4
#define Pin_Exti_HallEncoderA3_GPIO_Port GPIOB
#define Pin_Exti_HallEncoderA3_EXTI_IRQn EXTI4_IRQn
#define Pin_Pushpull_MotorDirectionB4_Pin GPIO_PIN_5
#define Pin_Pushpull_MotorDirectionB4_GPIO_Port GPIOB
#define Pin_Pushpull_MotorDirectionA4_Pin GPIO_PIN_6
#define Pin_Pushpull_MotorDirectionA4_GPIO_Port GPIOB
#define Pin_Pushpull_MotorDirectionB3_Pin GPIO_PIN_7
#define Pin_Pushpull_MotorDirectionB3_GPIO_Port GPIOB
#define Pin_Pushpull_MotorDirectionA3_Pin GPIO_PIN_8
#define Pin_Pushpull_MotorDirectionA3_GPIO_Port GPIOB
#define Pin_PullDown_HallEncoderB4_Pin GPIO_PIN_0
#define Pin_PullDown_HallEncoderB4_GPIO_Port GPIOE
#define Pin_PullDown_HallEncoderB3_Pin GPIO_PIN_1
#define Pin_PullDown_HallEncoderB3_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
