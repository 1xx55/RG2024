#ifndef COM_TO_RASPI_H
#define COM_TO_RASPI_H

// includes
#include "main.h"
#include "usart.h"
#include "JiXieBi.h"
#include "Chassis.hpp"
#include "camera_pos_dj.h"
#include "Shoot.h"

#include <math.h>

// macros
#define RASPI_USINGUART huart3

#define TO_STM32_ADJ_BLOCK 0x01
#define TO_STM32_CHASSIS_MOVE 0x02
#define TO_STM32_CHASSIS_ROTATE 0x03
#define TO_STM32_CAMERA_ROTATE 0x04
#define TO_STM32_START_JIAQU 0x05
#define TO_STM32_START_SHOOT 0x06
#define TO_STM32_STOPALL 0x07
#define TO_STM32_VELOCITY_LOW 0x08
#define TO_STM32_82LOCK 0x09
#define TO_STM32_82RELEASE 0x0a

#define MESSAGE_HEADER 0x66

#define TO_RASPI_CATCH_START 0x81
#define TO_RASPI_MOVE_FINISH 0x82
#define TO_RASPI_SHOOT_END 0x83
#define TO_RASPI_ROTATE_CAMERA_FINISH 0x84
    #define CAMERA_CW90 0x01
    #define CAMERA_CCW90 0x02
    #define CAMERA_CW180 0x03
    #define CAMERA_TO_0 0x04
#define TO_RASPI_JIAQU_FINISH 0x85

#define MISSION_MOVE  0x00
#define MISSION_JIAQU 0x01
#define MISSION_MOVESLOW 0x02
#define MISSION_SHOOT 0x03

//特殊任务id,在正式进入任务之前进入一个准备状态，轮询是否可以进入任务。
//(防止发射和夹取任务互锁。)
#define TASK_ID_READY 0x55

// 处理信息时所用相关参数
// 底盘旋转度数对应轮子旋转度数的系数(待调整)
const float CHASSIS_ROTATE_RAD_TO_WHEEL_RAD = 11.3f;
// 底盘运动距离(cm)对应轮子旋转度数(rad)
const float CHASSIS_CM_TO_RAD_AHEAD = 1.0f / (WHEEL_RADIUS * 100.0f) * 1.9f; //1.9:实际偏差
const float CHASSIS_CM_TO_RAD_LEFT = 1.0f / 4.045f; //实际
// function declarations
void com_raspi_Init();
void handle_received_data();
void send_message_to_raspi(uint8_t message);

void COM_RASPI_TIM_IT();
void COM_RASPI_TASK_SCHEDULE();

#endif
