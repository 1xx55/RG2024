/**
 * @file Steer.cpp
 * @author yssickjgd 1345578933@qq.com
 * @brief 舵机控制
 * @version 0.1
 * @date 2022-05-11
 * 
 * @copyright Copyright (c) 2022
 * 
 */

/* Includes ------------------------------------------------------------------*/

#include "Steer.hpp"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/**
 * @brief 限幅函数
 * 
 * @tparam Type 
 * @param x 传入数据
 * @param Min 最小值
 * @param Max 最大值
 */
template <typename Type>
void Math_Constrain(Type *x, Type Min, Type Max)
{
    if (*x < Min)
    {
        *x = Min;
    }
    else if (*x > Max)
    {
        *x = Max;
    }
}

/**
 * @brief 求绝对值
 * 
 * @tparam Type 
 * @param x 传入数据
 * @return Type x的绝对值
 */
template <typename Type>
Type Math_Abs(Type x)
{
    return ((x > 0) ? x : -x); 
}

/**
 * @brief 初始化舵机
 * 
 * @param __Driver_PWM_TIM 电机驱动定时器编号
 * @param __Driver_PWM_TIM_Channel_x 电机驱动定时器通道
 */
void Class_Steer::Init(TIM_HandleTypeDef __Driver_PWM_TIM, uint8_t __Driver_PWM_TIM_Channel_x)
{
    Driver_PWM_TIM = __Driver_PWM_TIM;
    Driver_PWM_TIM_Channel_x = __Driver_PWM_TIM_Channel_x;
    HAL_TIM_PWM_Start(&__Driver_PWM_TIM, __Driver_PWM_TIM_Channel_x);
}

/**
 * @brief 设定舵机占空比, 由此确定输出
 * 
 * @param output 舵机的角度
 */
void Class_Steer::Set_Out(float __Out)
{
    //占空比对应时间, 占空比
    float time;
    time = STEER_PWM_MIDDLE + (__Out - Max_Angle / 2) / (Max_Angle / 2) * STEER_PWM_EXTREME_TO_MIDDLE;
    Out = time / STEER_PWM_PERIOD * STEER_PWM_PRESCALER;
}

/**
 * @brief 
 * 
 */
void Class_Steer::Output()
{
    Math_Constrain(&Out, (int32_t)(STEER_PWM_PRESCALER * STEER_PWM_LEFT), (int32_t)(STEER_PWM_PRESCALER * STEER_PWM_RIGHT));
    __HAL_TIM_SetCompare(&Driver_PWM_TIM, Driver_PWM_TIM_Channel_x, Out);
}

/* Function prototypes -------------------------------------------------------*/

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
