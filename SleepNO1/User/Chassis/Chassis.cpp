/**
 * @file Chassis.cpp
 * @author yssickjgd 1345578933@qq.com
 * @brief 底盘控制
 * @version 0.1
 * @date 2022-05-04
 * 
 * @copyright Copyright (c) 2022
 * 
 */

/* Includes ------------------------------------------------------------------*/

#include "Chassis.hpp"


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
    return((x > 0) ? x : -x); 
}

/**
 * @brief 初始化底盘
 * 
 * @param __Driver_PWM_TIM 电机驱动定时器编号
 * @param __Calculate_EXTI_TIM 速度计算定时器编号
 */
void Class_Chassis::Init(TIM_HandleTypeDef __Driver_PWM_TIM, TIM_HandleTypeDef __Calculate_EXTI_TIM)
{
    Driver_PWM_TIM = __Driver_PWM_TIM;
    Calculate_TIM = __Calculate_EXTI_TIM;
    
    //电机初始化
    
    Motor[0].Init(__Driver_PWM_TIM, TIM_CHANNEL_1, Pin_Pushpull_MotorDirectionA1_Pin, Pin_Pushpull_MotorDirectionA1_GPIO_Port, Pin_Pushpull_MotorDirectionB1_Pin, Pin_Pushpull_MotorDirectionB1_GPIO_Port, __Calculate_EXTI_TIM, Pin_Exti_HallEncoderA1_Pin, Pin_Exti_HallEncoderA1_GPIO_Port, Pin_PullDown_HallEncoderB1_Pin, Pin_PullDown_HallEncoderB1_GPIO_Port);

    Motor[1].Init(__Driver_PWM_TIM, TIM_CHANNEL_2, Pin_Pushpull_MotorDirectionA2_Pin, Pin_Pushpull_MotorDirectionA2_GPIO_Port, Pin_Pushpull_MotorDirectionB2_Pin, Pin_Pushpull_MotorDirectionB2_GPIO_Port, __Calculate_EXTI_TIM, Pin_Exti_HallEncoderA2_Pin, Pin_Exti_HallEncoderA2_GPIO_Port, Pin_PullDown_HallEncoderB2_Pin, Pin_PullDown_HallEncoderB2_GPIO_Port);

    Motor[2].Init(__Driver_PWM_TIM, TIM_CHANNEL_3, Pin_Pushpull_MotorDirectionA3_Pin, Pin_Pushpull_MotorDirectionA3_GPIO_Port, Pin_Pushpull_MotorDirectionB3_Pin, Pin_Pushpull_MotorDirectionB3_GPIO_Port, __Calculate_EXTI_TIM, Pin_Exti_HallEncoderA3_Pin, Pin_Exti_HallEncoderA3_GPIO_Port, Pin_PullDown_HallEncoderB3_Pin, Pin_PullDown_HallEncoderB3_GPIO_Port);
    Motor[2].Set_Rotate_Direction_Flag(CCW);

    Motor[3].Init(__Driver_PWM_TIM, TIM_CHANNEL_4, Pin_Pushpull_MotorDirectionA4_Pin, Pin_Pushpull_MotorDirectionA4_GPIO_Port, Pin_Pushpull_MotorDirectionB4_Pin, Pin_Pushpull_MotorDirectionB4_GPIO_Port, __Calculate_EXTI_TIM, Pin_Exti_HallEncoderA4_Pin, Pin_Exti_HallEncoderA4_GPIO_Port, Pin_PullDown_HallEncoderB4_Pin, Pin_PullDown_HallEncoderB4_GPIO_Port);
    Motor[3].Set_Rotate_Direction_Flag(CCW);

    //电机PID初始化
    
    for(int i = 0; i < 4; i++)
    {
        Motor[i].Omega_PID.Init(800.0, 100.0, 0, (float)ULONG_MAX, (float)ULONG_MAX); 
        Motor[i].Angle_PID.Init(200.0, 0.0, 0.0,(float)ULONG_MAX, 30.0); //最后一个参数限制了麦轮稳定速度.

    }
    // for(int i = 2; i < 4; i++)
    // {
    //     Motor[i].Omega_PID.Init(1000.0, 0, 0, (float)ULONG_MAX, (float)ULONG_MAX);
        
    // }

    //遥控器初始化
//    DR16.Init(CHASSIS_DR16_CONTROL_UART);
}

/**
 * @brief 设定底盘速度
 * 
 * @param __Velocity 底盘速度
 */
void Class_Chassis::Set_Velocity(SpeedTypeDef __Velocity)
{
    Velocity = __Velocity;
}

/**
 * @brief 设定底盘控制方式
 * 
 * @param __Control_Method 底盘控制方式
 */
void Class_Chassis::Set_Control_Method(Enum_Control_Method __Control_Method)
{
    Control_Method = __Control_Method;

    for(int i = 0; i < 4; i++)
    {
        Motor[i].Set_Control_Method(__Control_Method);
    }
}

/**
 * @brief 底盘电机霍尔编码器触发中断处理函数
 * 
 */
void Class_Chassis::Hall_Encoder_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    //判断中断源, 并指向电机的中断处理函数
    if(GPIO_Pin == Pin_Exti_HallEncoderA1_Pin)
    {
        Motor[0].Hall_Encoder_GPIO_EXTI_Callback();
    }
    else if(GPIO_Pin == Pin_Exti_HallEncoderA2_Pin)
    {
        Motor[1].Hall_Encoder_GPIO_EXTI_Callback();
    }
    else if(GPIO_Pin == Pin_Exti_HallEncoderA3_Pin)
    {
        Motor[2].Hall_Encoder_GPIO_EXTI_Callback();
    }
    else if(GPIO_Pin == Pin_Exti_HallEncoderA4_Pin)
    {
        Motor[3].Hall_Encoder_GPIO_EXTI_Callback();
    }
}

/**
 * @brief 定时器中断处理函数
 * 
 */
void Class_Chassis::Calculate_TIM_PeriodElapsedCallback()
{
    //遥控器处理
    //DR16.Process_TIM_PeriodElapsedCallback();
    //Velocity = DR16.Get_Velocity();
    
    //速度解算
    Math_Constrain(&Velocity.X, -X_MAX, X_MAX);
    Math_Constrain(&Velocity.Y, -Y_MAX, Y_MAX);
    Math_Constrain(&Velocity.Omega, -OMEGA_MAX, OMEGA_MAX);
    Motor[0].Set_Omega_Target((-OMEGA_TO_MS * Velocity.Omega + Velocity.X - Velocity.Y + 0.01 * Velocity.delta_0) / WHEEL_RADIUS * ((Motor[0].Get_Rotate_Direction_Flag() == CW)?1:(-1)));
    Motor[1].Set_Omega_Target((-OMEGA_TO_MS * Velocity.Omega - Velocity.X - Velocity.Y + 0.01 * Velocity.delta_1) / WHEEL_RADIUS * ((Motor[1].Get_Rotate_Direction_Flag() == CW)?1:(-1)));
    Motor[2].Set_Omega_Target((OMEGA_TO_MS * Velocity.Omega + Velocity.X - Velocity.Y + 0.01 * Velocity.delta_2) / WHEEL_RADIUS * ((Motor[2].Get_Rotate_Direction_Flag() == CW)?1:(-1)));
    Motor[3].Set_Omega_Target((OMEGA_TO_MS * Velocity.Omega - Velocity.X - Velocity.Y + 0.01 * Velocity.delta_3) / WHEEL_RADIUS * ((Motor[3].Get_Rotate_Direction_Flag() == CW)?1:(-1)));

    //电机输出值设定并输出
    for(int i = 0; i < 4; i++)
    {
        Motor[i].Calculate_TIM_PeriodElapsedCallback();
    }
    
}

void Class_Chassis::Set_add_rad(float ahead, float left ,float rotate){
    //设置向前移动转多少圈, 向左移动转多少圈,以及旋转。 参数为负数则为反方向.
    //读取当前角度目标值
    float calc_target[4],org_target[4];
    for(int i=0;i<4;i++){
        calc_target[i] = Motor[i].Get_Angle_Now();
        org_target[i] = calc_target[i];
    }
    //前向转 4个轮子分别叠加角度.1,2号轮子参数多一个负号
    calc_target[0] -= ahead; 
    calc_target[1] -= ahead;
    calc_target[2] += ahead; 
    calc_target[3] += ahead;
    //左向转 2,4号轮子参数多一个负号 
    calc_target[0] += left; 
    calc_target[1] -= left;
    calc_target[2] += left; 
    calc_target[3] -= left; 
    //旋转 2,3号轮子参数多一个负号
    calc_target[0] += rotate; 
    calc_target[1] -= rotate;
    calc_target[2] -= rotate; 
    calc_target[3] += rotate;
    //写入计算得到的目标值
    for(int i=0;i<4;i++){
        if (calc_target[i]-org_target[i]>0)
            Motor[i].Set_Omega_Target(32767.0f);
        else
            Motor[i].Set_Omega_Target(-32767.0f);
        Motor[i].Set_Angle_Target(calc_target[i]);
    }
    //起始共速起步
    
    //end
}

// /**
//  * @brief 沿前后方向巡线移动
//  * 
//  */
// void Class_Chassis:: Motion_CorrectWhenMovingAtY() {
//     int16_t FrontError, BackError;
//     FrontError = Sensor.Trace_Error(0);
//     BackError = Sensor.Trace_Error(1);
//     Velocity.X = (FrontError + BackError) * (Kp_X4Y); //根据整体偏移距离调整左右方向的速度
//     Velocity.Omega = (FrontError - BackError) * (Kp_W4Y);//根据y轴偏移角度调整转动速度
// }

// /**
//  * @brief 沿左右方向巡线移动
//  * 
//  */
// void Class_Chassis:: Motion_CorrectWhenMovingAtX() {
//     int16_t LeftError, RightError;
//     LeftError = Sensor.Trace_Error(2);
//     RightError = Sensor.Trace_Error(3);
//     Velocity.Y = (LeftError + RightError) * (Kp_Y4X); //根据整体偏移距离调整前后方向的速度
//     Velocity.Omega = (LeftError - RightError) * (Kp_W4X);//根据x轴偏移角度调整转动速度
// }

/* Function prototypes -------------------------------------------------------*/

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
