/**
 * @file DR16.hpp
 * @author yssickjgd 1345578933@qq.com
 * @brief DR16遥控器
 * @version 0.1
 * @date 2022-05-26
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef DR16_HPP
#define DR16_HPP

/* Includes ------------------------------------------------------------------*/

#include "main.h"
#include "limits.h"

/* Exported macros -----------------------------------------------------------*/

//DR16数据包大小
const int8_t DR16_PACK_LENGTH = 18;
//DR16遥控器死区百分比
const float DR16_DEAD_ZONE = 0.03;

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 拨动开关状态
 * 
 */
enum Enum_DR16_DIP_Status
{
    DR16_DIP_STATUS_UP = 1,
    DR16_DIP_STATUS_DOWN,
    DR16_DIP_STATUS_MIDDLE
};

/**
 * @brief 控制状态
 * 
 */
enum Enum_DR16_Control_Mode
{
    DR16_CONTROL_MODE_ON = 0,
    DR16_CONTROL_MODE_OFF
};

/**
 * @brief 处理后的数据
 * 
 */
struct DR16_Data
{
    uint16_t Right_X;
    uint16_t Left_X;
    uint16_t Left_Y;
    Enum_DR16_DIP_Status Left_DIP;
};

/**
 * @brief 速度类型定义
 * 
 */
struct SpeedTypeDef
{
    //横移 m/s 右为正
    float X;    
    //前后 m/s 前为正
    float Y;    
    //旋转 rad/s 逆时针为正
    float Omega; 
};

class Class_DR16
{
    public:
    
        //接收数据包
        uint8_t Pack[DR16_PACK_LENGTH];

        void Init(UART_HandleTypeDef __Control_UART);

        SpeedTypeDef Get_Velocity();
        Enum_DR16_Control_Mode Get_DR16_Control_Mode();
        uint32_t Get_Receive_Window();
        uint32_t Get_Pre_Receive_Window();

        void Process_TIM_PeriodElapsedCallback();
        void Alive_UART_RxCpltCallback();

    protected:

        //遥控器控制UART端口
        UART_HandleTypeDef Control_UART;

        //处理后的数据
        DR16_Data Data;
        //已接收数量
        uint32_t Receive_Window = 0;
        //之前接收数量
        uint32_t Pre_Receive_Window = 0;

        //底盘控制数据
        SpeedTypeDef Velocity = 
        {
            0,
            0,
            0
        };
        Enum_DR16_Control_Mode DR16_Control_Mode = DR16_CONTROL_MODE_OFF;
        
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
