/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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

extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern UART_HandleTypeDef huart1;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

//This is the timer of 4 motor drivers of the chassis
#define CHASSIS_MOTOR_PWM_DRIVER_TIM htim4
//This is the timer of speed calculating unit of the chassis
#define CHASSIS_MOTOR_CALCULATE_TIM htim5
//This is the uart of controller
#define CHASSIS_DR16_CONTROL_UART huart1

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Pin_Exti_HallEncoderA1_Pin GPIO_PIN_0
#define Pin_Exti_HallEncoderA1_GPIO_Port GPIOC
#define Pin_Exti_HallEncoderA1_EXTI_IRQn EXTI0_IRQn
#define Pin_Exti_HallEncoderA2_Pin GPIO_PIN_1
#define Pin_Exti_HallEncoderA2_GPIO_Port GPIOC
#define Pin_Exti_HallEncoderA2_EXTI_IRQn EXTI1_IRQn
#define Pin_Exti_HallEncoderA3_Pin GPIO_PIN_2
#define Pin_Exti_HallEncoderA3_GPIO_Port GPIOC
#define Pin_Exti_HallEncoderA3_EXTI_IRQn EXTI2_IRQn
#define Pin_Exti_HallEncoderA4_Pin GPIO_PIN_3
#define Pin_Exti_HallEncoderA4_GPIO_Port GPIOC
#define Pin_Exti_HallEncoderA4_EXTI_IRQn EXTI3_IRQn
#define Pin_Pushpull_MotorDirectionA1_Pin GPIO_PIN_4
#define Pin_Pushpull_MotorDirectionA1_GPIO_Port GPIOA
#define Pin_Pushpull_MotorDirectionA2_Pin GPIO_PIN_5
#define Pin_Pushpull_MotorDirectionA2_GPIO_Port GPIOA
#define Pin_Pushpull_MotorDirectionA3_Pin GPIO_PIN_6
#define Pin_Pushpull_MotorDirectionA3_GPIO_Port GPIOA
#define Pin_Pushpull_MotorDirectionA4_Pin GPIO_PIN_7
#define Pin_Pushpull_MotorDirectionA4_GPIO_Port GPIOA
#define Pin_Pushpull_MotorDirectionB1_Pin GPIO_PIN_12
#define Pin_Pushpull_MotorDirectionB1_GPIO_Port GPIOB
#define Pin_Pushpull_MotorDirectionB2_Pin GPIO_PIN_13
#define Pin_Pushpull_MotorDirectionB2_GPIO_Port GPIOB
#define Pin_Pushpull_MotorDirectionB3_Pin GPIO_PIN_14
#define Pin_Pushpull_MotorDirectionB3_GPIO_Port GPIOB
#define Pin_Pushpull_MotorDirectionB4_Pin GPIO_PIN_15
#define Pin_Pushpull_MotorDirectionB4_GPIO_Port GPIOB
#define Pin_PullDown_HallEncoderB1_Pin GPIO_PIN_8
#define Pin_PullDown_HallEncoderB1_GPIO_Port GPIOC
#define Pin_PullDown_HallEncoderB2_Pin GPIO_PIN_9
#define Pin_PullDown_HallEncoderB2_GPIO_Port GPIOC
#define Pin_UART_DR16_Receive_Pin GPIO_PIN_10
#define Pin_UART_DR16_Receive_GPIO_Port GPIOA
#define Pin_PullDown_HallEncoderB3_Pin GPIO_PIN_10
#define Pin_PullDown_HallEncoderB3_GPIO_Port GPIOC
#define Pin_PullDown_HallEncoderB4_Pin GPIO_PIN_11
#define Pin_PullDown_HallEncoderB4_GPIO_Port GPIOC
#define Pin_PWM_MotorDriver1_Pin GPIO_PIN_6
#define Pin_PWM_MotorDriver1_GPIO_Port GPIOB
#define Pin_PWM_MotorDriver2_Pin GPIO_PIN_7
#define Pin_PWM_MotorDriver2_GPIO_Port GPIOB
#define Pin_PWM_MotorDriver3_Pin GPIO_PIN_8
#define Pin_PWM_MotorDriver3_GPIO_Port GPIOB
#define Pin_PWM_MotorDriver4_Pin GPIO_PIN_9
#define Pin_PWM_MotorDriver4_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
