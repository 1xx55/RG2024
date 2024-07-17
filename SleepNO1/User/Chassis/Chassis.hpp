/**
 * @file Chassis.hpp
 * @author yssickjgd 1345578933@qq.com
 * @brief 底盘控制
 * @version 0.1
 * @date 2022-05-04
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef CHASSIS_HPP
#define CHASSIS_HPP

/* Includes ------------------------------------------------------------------*/

#include "main.h"
#include "Motor.hpp"
// #include "DR16.hpp"
// #include "trace.hpp"

/* Exported macros -----------------------------------------------------------*/

//轮半径, m
const float WHEEL_RADIUS = 0.075f;

//轮组满转线速度, m/s
const float WHEEL_FULL_V = (MOTOR_FULL_OMEGA * WHEEL_RADIUS);  // about 1.3m/s

//omega到m/s映射系数, (横轮距+纵轮距)/2, m/rad
const float OMEGA_TO_MS = ((0.500f + 0.355f) / 2);      //实际0.5006f

//绝对速度限制, 任何情况不能超过
//横移速度上限, m/s
const float X_MAX = 1.5f;    
//前进速度上限, m/s
const float Y_MAX = 1.5f;    
//旋转速度上限, rad/s
const float OMEGA_MAX = 3.0f; 

//巡线相关参数
const float Kp_X4Y = 0.01;
const float Kp_W4Y = 0.1;
const float Kp_W4X = -0.1;
const float Kp_Y4X = 0.01;

/* Exported types ------------------------------------------------------------*/

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
    //旋转 rad/s 逆时针为负
    float Omega; 
    //motor[0]修正
    int8_t delta_0;
    //motor[1]修正
    int8_t delta_1;
    //motor[2]修正
    int8_t delta_2;
    //motor[3]修正
    int8_t delta_3;
};

class Class_Chassis
{
    public:

        //底盘对应的电机
        Class_Motor_With_Hall_Encoder Motor[4];
        //底盘对应的遥控器
//        Class_DR16 DR16;

        void Init(TIM_HandleTypeDef __Driver_PWM_TIM, TIM_HandleTypeDef __Calculate_EXTI_TIM);
        
        void Set_Velocity(SpeedTypeDef __Velocity);
        void Set_Control_Method(Enum_Control_Method __Control_Method);
        
        void Hall_Encoder_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
        void Calculate_TIM_PeriodElapsedCallback();

        void Motion_CorrectWhenMovingAtY();
        void Motion_CorrectWhenMovingAtX();

    protected:

        //电机PWM驱动定时器
        TIM_HandleTypeDef Driver_PWM_TIM;
        //电机计算定时器中断
        TIM_HandleTypeDef Calculate_TIM;

        //底盘控制方式
        Enum_Control_Method Control_Method = Control_Method_OMEGA;

        //底盘速度
        SpeedTypeDef Velocity = 
        {
            0,
            0,
            0,
					
            0,
            0,
            0,
            0
        };

};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
