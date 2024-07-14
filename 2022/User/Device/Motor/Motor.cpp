/**
 * @file Motor.cpp
 * @author yssickjgd 1345578933@qq.com
 * @brief 霍尔编码器-直流电机控制
 * @version 0.1
 * @date 2022-05-03
 * 
 * @copyright Copyright (c) 2022
 * 
 */

/* Includes ------------------------------------------------------------------*/

#include "Motor.hpp"

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
 * @brief 初始化电机
 * 
 * @param __Driver_PWM_TIM 电机驱动定时器编号
 * @param __Driver_PWM_TIM_Channel_x 电机驱动定时器通道
 * @param __Output_A_GPIO_Pin 电机方向A相引脚号
 * @param __Output_A_GPIOx 电机方向A相引脚组
 * @param __Output_B_GPIO_Pin 电机方向B相引脚号
 * @param __Output_B_GPIOx 电机方向B相引脚组
 */
void Class_Motor::Init(TIM_HandleTypeDef __Driver_PWM_TIM, uint8_t __Driver_PWM_TIM_Channel_x, uint16_t __Output_A_GPIO_Pin, GPIO_TypeDef *__Output_A_GPIOx, uint16_t __Output_B_GPIO_Pin, GPIO_TypeDef *__Output_B_GPIOx)
{
    Driver_PWM_TIM = __Driver_PWM_TIM;
    Driver_PWM_TIM_Channel_x = __Driver_PWM_TIM_Channel_x;
    Output_A_GPIO_Pin = __Output_A_GPIO_Pin;
    Output_A_GPIOx = __Output_A_GPIOx;
    Output_B_GPIO_Pin = __Output_B_GPIO_Pin;
    Output_B_GPIOx = __Output_B_GPIOx;

    //输出PWM
    HAL_TIM_PWM_Start(&__Driver_PWM_TIM, __Driver_PWM_TIM_Channel_x);
}

/**
 * @brief 设定电机正向旋转方向
 * 
 * @param __Rotate_Direction_Flag 电机正向旋转方向
 */
void Class_Motor::Set_Rotate_Direction_Flag(Enum_Rotate_Direction __Rotate_Direction_Flag)
{
    Rotate_Direction_Flag = __Rotate_Direction_Flag;
}

/**
 * @brief 设定电机PWM满占空比对应的数值
 * 
 * @param __Motor_Full_Omega 电机PWM满占空比对应的数值
 */
void Class_Motor::Set_Motor_Full_Omega(float __Motor_Full_Omega)
{
    Motor_Full_Omega = __Motor_Full_Omega;
}

/**
 * @brief 设定电机PWM满占空比对应的数值
 * 
 * @param __Motor_PWM_Period 电机PWM满占空比对应的数值
 */
void Class_Motor::Set_Motor_PWM_Period(int32_t __Motor_PWM_Period)
{
    Motor_PWM_Period = __Motor_PWM_Period;
}

/**
 * @brief 设定电机目标输出强度, 即电机PWM占空比的分子
 * 
 * @param __Out 电机目标输出强度, 即电机PWM占空比的分子
 */
void Class_Motor::Set_Out(int32_t __Out)
{
    Out = __Out;
}

/**
 * @brief 获取电机正向旋转方向
 * 
 * @return Enum_Rotate_Direction 电机正向旋转方向
 */
Enum_Rotate_Direction Class_Motor::Get_Rotate_Direction_Flag()
{
    return(Rotate_Direction_Flag);
}

/**
 * @brief 获取电机减速后满转转速, rad/s
 * 
 * @return float 电机减速后满转转速, rad/s
 */
float Class_Motor::Get_Motor_Full_Omega()
{
    return(Motor_Full_Omega);
}

/**
 * @brief 获取电机PWM满占空比对应的数值
 * 
 * @return int32_t 电机PWM满占空比对应的数值
 */
int32_t Class_Motor::Get_Motor_PWM_Period()
{
    return(Motor_PWM_Period);
}

/**
 * @brief 获取电机目标输出强度, 即电机PWM占空比的分子
 * 
 * @return int32_t 电机目标输出强度, 即电机PWM占空比的分子
 */
int32_t Class_Motor::Get_Out()
{
    return(Out);
}

/**
 * @brief 设定电机占空比, 确定输出
 * 
 */
void Class_Motor::Output()
{
    if (Out == 0)
    {
        HAL_GPIO_WritePin(Output_A_GPIOx, Output_A_GPIO_Pin, GPIO_PIN_RESET); 
        HAL_GPIO_WritePin(Output_B_GPIOx, Output_B_GPIO_Pin, GPIO_PIN_RESET);
    }
    else if (Out > 0)
    {
        HAL_GPIO_WritePin(Output_A_GPIOx, Output_A_GPIO_Pin, GPIO_PIN_SET); 
        HAL_GPIO_WritePin(Output_B_GPIOx, Output_B_GPIO_Pin, GPIO_PIN_RESET);
    }
    else if (Out < 0)
    {
        HAL_GPIO_WritePin(Output_A_GPIOx, Output_A_GPIO_Pin, GPIO_PIN_RESET); 
        HAL_GPIO_WritePin(Output_B_GPIOx, Output_B_GPIO_Pin, GPIO_PIN_SET);
    }
    __HAL_TIM_SetCompare(&Driver_PWM_TIM, Driver_PWM_TIM_Channel_x, Math_Abs(Out));
}

/**
 * @brief 初始化电机
 * 
 * @param __Driver_PWM_TIM 电机驱动定时器编号
 * @param __Driver_PWM_TIM_Channel_x 电机驱动定时器通道
 * @param __Output_A_GPIO_Pin 电机方向A相引脚号
 * @param __Output_A_GPIOx 电机方向A相引脚组
 * @param __Output_B_GPIO_Pin 电机方向B相引脚号
 * @param __Output_B_GPIOx 电机方向B相引脚组
 * @param __Calculate_TIM 速度计算定时器编号
 * @param __Input_A_GPIO_Pin 霍尔编码器方向A引脚号
 * @param __Input_A_GPIOx 霍尔编码器方向A引脚组
 * @param __Input_B_GPIO_Pin 霍尔编码器方向B引脚号
 * @param __Input_B_GPIOx 霍尔编码器方向B引脚组
 */
void Class_Motor_With_Hall_Encoder::Init(TIM_HandleTypeDef __Driver_PWM_TIM, uint8_t __Driver_PWM_TIM_Channel_x, uint16_t __Output_A_GPIO_Pin, GPIO_TypeDef *__Output_A_GPIOx, uint16_t __Output_B_GPIO_Pin, GPIO_TypeDef *__Output_B_GPIOx, TIM_HandleTypeDef __Calculate_TIM, uint16_t __Input_A_GPIO_Pin, GPIO_TypeDef *__Input_A_GPIOx, uint16_t __Input_B_GPIO_Pin, GPIO_TypeDef *__Input_B_GPIOx)
{
    Driver_PWM_TIM = __Driver_PWM_TIM;
    Driver_PWM_TIM_Channel_x = __Driver_PWM_TIM_Channel_x;
    Output_A_GPIO_Pin = __Output_A_GPIO_Pin;
    Output_A_GPIOx = __Output_A_GPIOx;
    Output_B_GPIO_Pin = __Output_B_GPIO_Pin;
    Output_B_GPIOx = __Output_B_GPIOx;
    Calculate_TIM = __Calculate_TIM;
    Input_A_GPIO_Pin = __Input_A_GPIO_Pin;
    Input_A_GPIOx = __Input_A_GPIOx;
    Input_B_GPIO_Pin = __Input_B_GPIO_Pin;
    Input_B_GPIOx = __Input_B_GPIOx;
    HAL_TIM_PWM_Start(&__Driver_PWM_TIM, __Driver_PWM_TIM_Channel_x);
}

/**
 * @brief 设定电机控制方式
 * 
 * @param __Control_Method 电机控制方式
 */
void Class_Motor_With_Hall_Encoder::Set_Control_Method(Enum_Control_Method __Control_Method)
{
    Control_Method = __Control_Method;
}

/**
 * @brief 设定目标电机转速, rad/s
 * 
 * @param __Omega_Target 目标电机转速, rad/s
 */
void Class_Motor_With_Hall_Encoder::Set_Omega_Target(float __Omega_Target)
{
    Omega_Target = __Omega_Target;
}

/**
 * @brief 设定电机减速后霍尔编码器编码数, /rad
 * 
 * @param __Motor_Encoder_Num_Per_Rad 目标电机转速, rad/s
 */
void Class_Motor_With_Hall_Encoder::Set_Motor_Encoder_Num_Per_Rad(float __Motor_Encoder_Num_Per_Rad)
{
    Motor_Encoder_Num_Per_Rad = __Motor_Encoder_Num_Per_Rad;
}

/**
 * @brief 设定目标电机角度, rad
 * 
 * @param __Angle_Target 目标电机角度, rad
 */
void Class_Motor_With_Hall_Encoder::Set_Angle_Target(float __Angle_Target)
{
    Angle_Target = __Angle_Target;
}

/**
 * @brief 设定电机控制方式
 * 
 * @return Enum_Control_Method 电机控制方式
 */
Enum_Control_Method Class_Motor_With_Hall_Encoder::Get_Control_Method()
{
    return(Control_Method);
}

/**
 * @brief 设定当前电机转速, rad/s
 * 
 * @return float 当前电机转速, rad/s
 */
float Class_Motor_With_Hall_Encoder::Get_Omega_Now()
{
    return(Omega_Now);
}

/**
 * @brief 设定当前电机角度, rad
 * 
 * @return float 当前电机角度, rad
 */
float Class_Motor_With_Hall_Encoder::Get_Angle_Now()
{
    return(Angle_Now);
}

/**
 * @brief 霍尔编码器中断处理函数
 * 
 */
void Class_Motor_With_Hall_Encoder::Hall_Encoder_GPIO_EXTI_Callback()
{
    //计算电机转过的编码数, 以便后续算出角度和速度
    if(((HAL_GPIO_ReadPin(Input_B_GPIOx, Input_B_GPIO_Pin) == 0) ^ (Rotate_Direction_Flag == CW)) == 0)
    {
        Hall_Encoder_Count--;
    }
    else
    {
        Hall_Encoder_Count++;
    }
}

/**
 * @brief 定时器中断处理函数
 * 
 */
void Class_Motor_With_Hall_Encoder::Calculate_TIM_PeriodElapsedCallback()
{
    int32_t delta;
    delta = Hall_Encoder_Count - Prev_Hall_Encoder_Count;
    //计算角度
    Angle_Now += (float)delta / Motor_Encoder_Num_Per_Rad;
    //计算速度
    Omega_Now = (float)delta / Motor_Encoder_Num_Per_Rad / MOTOR_CALCULATE_PERIOD;
    Prev_Hall_Encoder_Count = Hall_Encoder_Count;

    if(Control_Method == Control_Method_OPENLOOP)
    {
        //开环控制直接输出速度
        Out = Omega_Target * Motor_PWM_Period / Motor_Full_Omega;
    }
    else if(Control_Method == Control_Method_OMEGA)
    {
        //速度控制, 单环PID

        Omega_PID.Set_Now(Omega_Now);
        Omega_PID.Set_Target(Omega_Target);
        Omega_PID.Adjust_TIM_PeriodElapsedCallback();
        Out = Omega_PID.Get_Out();

    }
    else if(Control_Method == Control_Method_ANGLE)
    {
        //角度控制, 双环PID

        Angle_PID.Set_Target(Angle_Target);
        
        Angle_PID.Set_Now(Angle_Now);
        Angle_PID.Adjust_TIM_PeriodElapsedCallback();
        Omega_PID.Set_Target(Angle_PID.Get_Out());
			
        Omega_PID.Set_Now(Omega_Now);
        Omega_PID.Adjust_TIM_PeriodElapsedCallback();
        Out = Omega_PID.Get_Out();
    }
    Math_Constrain(&Out, -MOTOR_CALCULATE_PRESCALER, MOTOR_CALCULATE_PRESCALER);
    Output();
}

/* Function prototypes -------------------------------------------------------*/

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
