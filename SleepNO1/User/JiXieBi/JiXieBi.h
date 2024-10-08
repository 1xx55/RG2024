#ifndef JIXIEBI_H
#define JIXIEBI_H

// includes
#include "main.h"
#include "air_pump.h"
#include "my_servo.h"
#include "stm32f4xx_hal.h"
#include "com_to_raspi.h"
#include "Shoot.h"

// macros

// function declarations
void JiXieBi_TIM_IT();
void JiXieBi_TASK_Schedule();
void JiXieBi_Init();

void JiXieBi_READY();
void JiXieBi_JIAQU();
void JiXieBi_set_fourth_dj(int16_t para);

int IS_JiXieBi_CanLetChassisNextMove();
int IS_JiXieBi_Busy();

#endif
