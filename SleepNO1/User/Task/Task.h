#ifndef TASK_H
#define TASK_H

// includes
#include "main.h"
#include "Shoot.h"
#include "JiXieBi.h"
// macros

// function declarations
void Task_10ms_TIM_IT(TIM_HandleTypeDef *htim);
void Task_TIM_Init();
void Task_Schedule();
#endif
