/**
 * @file PID.cpp
 * @author yssickjgd 1345578933@qq.com
 * @brief PID算法
 * @version 0.1
 * @date 2022-05-08
 * 
 * @copyright Copyright (c) 2022
 * 
 */

/* Includes ------------------------------------------------------------------*/

#include "PID.hpp"

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
 * @brief PID初始化
 * 
 * @param __K_P P值
 * @param __K_I I值
 * @param __K_D D值
 * @param __I_Out_Max 积分限幅
 * @param __Out_Max 输出限幅
 */
void Class_PID::Init(float __K_P, float __K_I, float __K_D, float __I_Out_Max, float __Out_Max)
{
    K_P = __K_P;
    K_I = __K_I;
    K_D = __K_D;
    I_Out_Max = __I_Out_Max;
    Out_Max = __Out_Max;
}

/**
 * @brief 设定PID的P
 * 
 * @param __K_P PID的P
 */
void Class_PID::Set_K_P(float __K_P)
{
    K_P = __K_P;
}

/**
 * @brief 设定PID的I
 * 
 * @param __K_I PID的I
 */
void Class_PID::Set_K_I(float __K_I)
{
    K_I = __K_I;
}

/**
 * @brief 设定PID的D
 * 
 * @param __K_D PID的D
 */
void Class_PID::Set_K_D(float __K_D)
{
    K_D = __K_D;
}

/**
 * @brief 设定积分限幅, 0为不限制
 * 
 * @param __I_Out_Max 积分限幅, 0为不限制
 */
void Class_PID::Set_I_Out_Max(float __I_Out_Max)
{
    I_Out_Max = __I_Out_Max;
}

/**
 * @brief 设定输出限幅, 0为不限制
 * 
 * @param __Out_Max 输出限幅, 0为不限制
 */
void Class_PID::Set_Out_Max(float __Out_Max)
{
    Out_Max = __Out_Max;
}

/**
 * @brief 设定目标值
 * 
 * @param __Target 目标值
 */
void Class_PID::Set_Target(float __Target)
{
    Target = __Target;
}

/**
 * @brief 设定当前值
 * 
 * @param __Now 当前值
 */
void Class_PID::Set_Now(float __Now)
{
    Now = __Now;
}

/**
 * @brief 设定死区, Error在其绝对值内不输出
 * 
 * @param __Dead_Zone 死区, Error在其绝对值内不输出
 */
void Class_PID::Set_Dead_Zone(float __Dead_Zone)
{
    Dead_Zone = __Dead_Zone;
}

/**
 * @brief 设定定速内段阈值, 0为不限制
 * 
 * @param __Variable_Speed_I_A 定速内段阈值, 0为不限制
 */
void Class_PID::Set_Variable_Speed_I_A(float __Variable_Speed_I_A)
{
    Variable_Speed_I_A = __Variable_Speed_I_A;
}

/**
 * @brief 设定变速区间, 0为不限制
 * 
 * @param __Variable_Speed_I_B 变速区间, 0为不限制
 */
void Class_PID::Set_Variable_Speed_I_B(float __Variable_Speed_I_B)
{
    Variable_Speed_I_B = __Variable_Speed_I_B;
}

/**
 * @brief 设定积分分离阈值，需为正数, 0为不限制
 * 
 * @param __I_Separate_Threshold 积分分离阈值，需为正数, 0为不限制
 */
void Class_PID::Set_I_Separate_Threshold(float __I_Separate_Threshold)
{
    I_Separate_Threshold = __I_Separate_Threshold;
}

/**
 * @brief 设定微分先行
 * 
 * @param __D_First 微分先行
 */
void Class_PID::Set_D_First(Enum_D_First __D_First)
{
    D_First = __D_First;
}

/**
 * @brief 获取输出值
 * 
 * @return float 输出值
 */
float Class_PID::Get_Out()
{
    return(Out);
}

/**
 * @brief PID调整值
 * 
 * @return float 输出值
 */
void Class_PID::Adjust_TIM_PeriodElapsedCallback()
{
    float Abs_Error;
    Error = Target - Now;
    Abs_Error = Math_Abs(Error);

    //判断死区
    if(Abs_Error < Dead_Zone)
    {
        Out = 0;
        return;
    }

    //计算p项
    P_Out = K_P * Error;

    //计算i项
    //线性变速积分
    float Speed_Ratio;
    if(Variable_Speed_I_A == 0 && Variable_Speed_I_A == 0)
    {
        Speed_Ratio = 1;
    }
    else
    {
        if(Abs_Error < Variable_Speed_I_B)
        {
            Speed_Ratio = 1;
        }
        else if(Variable_Speed_I_B <= Abs_Error < Variable_Speed_I_A + Variable_Speed_I_B)
        {
            Speed_Ratio = (Variable_Speed_I_A + Variable_Speed_I_B - Abs_Error) / Variable_Speed_I_A;
        }
        if(Abs_Error >= Variable_Speed_I_B)
        {
            Speed_Ratio = 0;
        }
    }
    Integral_Error += Speed_Ratio * D_T * Error;
    if(I_Out_Max != 0)
    {
        Math_Constrain(&Integral_Error, -I_Out_Max / K_I, I_Out_Max / K_I);
    }
    //积分分离
    if(I_Separate_Threshold == 0)
    {
        I_Out = K_I * Integral_Error;
    }
    else
    {
        if(Abs_Error < I_Separate_Threshold)
        {
            I_Out = K_I * Integral_Error;
        }
        else
        {
            I_Out = 0;
        }
    }

    //计算d项
    if(D_First == D_First_YES)
    {
        D_Out = K_D * (Now - Pre_Value);
    }
    else
    {
        D_Out = K_D * (Error - Pre_Error);
    }

    //善后工作
    Pre_Error = Error;
    Pre_Value = Now;
    Out = P_Out + I_Out + D_Out;
    if(Out_Max != 0)
    {
        Math_Constrain(&Out, -Out_Max, Out_Max);
    }
    

}

/* Function prototypes -------------------------------------------------------*/

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
