#ifndef SHOOT_H
#define SHOOT_H

// includes
#include "main.h"
#include "MoCaLun.h"
#include "MSDriver.h"
#include "fix_pos_dj.h"
#include "com_to_raspi.h"


// macros
// shoot tim:10ms

// function declarations
void SHOOT_TIM_IT();
void SHOOT_START();
void SHOOT_ABRUPT();
void SHOOT_set_speed(int speed);
void SHOOT_Init();
void SHOOT_TASK_Schedule();
#endif
