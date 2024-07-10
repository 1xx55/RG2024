#ifndef MSDRIVER_H
#define MSDRIVER_H

// includes
#include "gpio.h"
#include "main.h"
#include "tim.h"
// macros


// function declarations
void MS_Init();

void MS_GO();
void MS_GO_UP();
void MS_GO_DOWN();


// specials
// 需要放入 HAL_GPIO_EXTI_Callback
void GDM_EXTI(uint16_t GPIO_PIN);

// 需要放入 HAL_TIM_PeriodElapsedCallback
void ACC_TIM14_IT(TIM_HandleTypeDef *htim);
#endif
