#ifndef JIXIEBI_H
#define JIXIEBI_H

// includes
#include "main.h"
#include "air_pump.h"
#include "my_servo.h"
#include "stm32f4xx_hal.h"

// macros

// function declarations
void JiXieBi_TIM_IT();
void JiXieBi_TASK_Schedule();
void JiXieBi_Init();

#endif
