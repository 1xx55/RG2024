/**
 * @file Motor.hpp
 * @author yssickjgd 1345578933@qq.com
 * @brief 霍尔编码器-直流电机控制
 * @version 0.1
 * @date 2022-05-03
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef MOTOR_HPP
#define MOTOR_HPP

/* Includes ------------------------------------------------------------------*/

#include "main.h"
#include "usart.h"
#include "PID.hpp"

/* Exported macros -----------------------------------------------------------*/

//圆周率
const float PI = 3.14159f;

//电机减速后满转转速, rad/s
const float MOTOR_FULL_OMEGA = (165.0f / 60.0f * 2.0f * PI);        // our额定转速165(r/min) / 60(s/min) * 2pi(rad/r)     空载转速

//电机减速后霍尔编码器编码数, /rad
const float MOTOR_ENCODER_NUM_PER_RAD = (17.0f *49.0f / 2.0f / PI);     //  编码线数(电机编码器旋转一圈的脉冲数):17  减速比1:49  减速后编码线数:17*49=833

//电机PWM满占空比对应的数值
const int32_t MOTOR_CALCULATE_PRESCALER = 32767;

//计算定时器频率, s
const float MOTOR_CALCULATE_PERIOD = 0.01f;  


/* Exported types ------------------------------------------------------------*/

/**
 * @brief 电机控制方式
 * 
 */
enum Enum_Control_Method
{
    Control_Method_OPENLOOP = 0,
    Control_Method_OMEGA,
    Control_Method_ANGLE
};

/**
 * @brief 电机控制方式
 * 
 */
enum Enum_Rotate_Direction
{
    CW = 0,
    CCW
};

class Class_Motor
{
    public:

        void Init(TIM_HandleTypeDef __Driver_PWM_TIM, uint8_t __Driver_PWM_TIM_Channel_x, uint16_t __Output_A_GPIO_Pin, GPIO_TypeDef *__Output_A_GPIOx, uint16_t __Output_B_GPIO_Pin, GPIO_TypeDef *__Output_B_GPIOx);

        void Set_Rotate_Direction_Flag(Enum_Rotate_Direction __Rotate_Direction_Flag);
        void Set_Motor_Full_Omega(float __Motor_Full_Omega);
        void Set_Motor_PWM_Period(int32_t __Motor_PWM_Period);
        void Set_Out(int32_t __Out);

        Enum_Rotate_Direction Get_Rotate_Direction_Flag();
        float Get_Motor_Full_Omega();
        int32_t Get_Motor_PWM_Period();
        int32_t Get_Out();

        void Output();

    protected:

        //电机驱动定时器编号
        TIM_HandleTypeDef Driver_PWM_TIM;
        //定时器通道
        uint8_t Driver_PWM_TIM_Channel_x;
        //电机方向A相
        uint16_t Output_A_GPIO_Pin;
        GPIO_TypeDef *Output_A_GPIOx;
        //电机方向B相
        uint16_t Output_B_GPIO_Pin;
        GPIO_TypeDef *Output_B_GPIOx;

        //电机正向旋转方向
        Enum_Rotate_Direction Rotate_Direction_Flag = CW;
        //电机减速后满转转速, rad/s
        float Motor_Full_Omega = MOTOR_FULL_OMEGA;
        //电机PWM满占空比对应的数值
        int32_t Motor_PWM_Period = MOTOR_CALCULATE_PRESCALER;
        //电机目标输出强度, 即电机PWM占空比的分子
        int32_t Out = 0;

};

class Class_Motor_With_Hall_Encoder : public Class_Motor
{
    public:
    
        //电机速度环PID参数
        Class_PID Omega_PID;
        //电机角度环PID参数
        Class_PID Angle_PID;

        void Init(TIM_HandleTypeDef __Driver_PWM_TIM, uint8_t __Driver_PWM_TIM_Channel_x, uint16_t __Output_A_GPIO_Pin, GPIO_TypeDef *__Output_A_GPIOx, uint16_t __Output_B_GPIO_Pin, GPIO_TypeDef *__Output_B_GPIOx, TIM_HandleTypeDef __Calculate_EXTI_TIM, uint16_t __Input_A_GPIO_Pin, GPIO_TypeDef *__Input_A_GPIOx, uint16_t __Input_B_GPIO_Pin, GPIO_TypeDef *__Input_B_GPIOx);
        
        void Set_Control_Method(Enum_Control_Method Control_Method);
        void Set_Motor_Encoder_Num_Per_Rad(float Motor_Encoder_Num_Per_Rad);
        void Set_Omega_Target(float Omega_Target);
        void Set_Angle_Target(float Angle_Target);

        Enum_Control_Method Get_Control_Method();
        float Get_Omega_Now();
        float Get_Angle_Now();

        void Hall_Encoder_GPIO_EXTI_Callback();
        void Calculate_TIM_PeriodElapsedCallback();

    protected:

        //计算定时器编号
        TIM_HandleTypeDef Calculate_TIM;
        //霍尔编码器方向A
        uint16_t Input_A_GPIO_Pin;
        GPIO_TypeDef *Input_A_GPIOx;
        //霍尔编码器方向B
        uint16_t Input_B_GPIO_Pin;
        GPIO_TypeDef *Input_B_GPIOx;

        //电机控制方式
        Enum_Control_Method Control_Method = Control_Method_OMEGA;
        //霍尔编码器计数
        int32_t Hall_Encoder_Count = 0;
        //霍尔编码器前一个计数
        int32_t Prev_Hall_Encoder_Count = 0;
        //电机减速后霍尔编码器编码数, /rad
        float Motor_Encoder_Num_Per_Rad = MOTOR_ENCODER_NUM_PER_RAD;
        //当前电机转速, rad/s
        float Omega_Now = 0;
        //目标电机转速, rad/s
        float Omega_Target = 0;
        //当前电机角度, rad
        float Angle_Now = 0;
        //目标电机角度, rad
        float Angle_Target = 0;

};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
