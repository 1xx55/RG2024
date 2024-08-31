#ifndef COM_TO_RASPI_H
#define COM_TO_RASPI_H

// includes
#include "main.h"
#include "usart.h"
#include "JiXieBi.h"
#include "Chassis.hpp"
#include <math.h>

// macros
#define RASPI_USINGUART huart3

#define TO_STM32_ADJ_BLOCK 0x01
#define TO_STM32_CHASSIS_MOVE 0x02
#define TO_STM32_CHASSIS_ROTATE 0x03

#define MESSAGE_HEADER 0x66

#define TO_RASPI_CATCH_START 0x81
#define TO_RASPI_MOVE_FINISH 0x82
#define TO_RASPI_SHOOT_END 0x83

// 处理信息时所用相关参数
// 底盘旋转度数对应轮子旋转度数的系数(待调整)
const float CHASSIS_ROTATE_RAD_TO_WHEEL_RAD = 2.0f;
// 底盘运动距离(cm)对应轮子旋转度数(rad)
const float CHASSIS_CM_TO_RAD = 1.0f / (WHEEL_RADIUS * 100.0f);

// function declarations
void com_raspi_Init();
void handle_received_data();
void send_message_to_raspi(uint8_t message);

void COM_RASPI_TIM_IT();
void COM_RASPI_TASK_SCHEDULE();

#endif
