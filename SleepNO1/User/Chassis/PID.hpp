/**
 * @file PID.hpp
 * @author yssickjgd 1345578933@qq.com
 * @brief PID算法
 * @version 0.1
 * @date 2022-05-03
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef PID_HPP
#define PID_HPP

/* Includes ------------------------------------------------------------------*/

#include "main.h"
#include "limits.h"

/* Exported macros -----------------------------------------------------------*/

//PID定时器周期, s
const float PID_CALCULATE_PERIOD = 0.1f;

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 微分先行
 * 
 */
enum Enum_D_First
{
    D_First_NO = 0,
    D_First_YES
};

class Class_PID
{
    public:

        void Init(float __K_P, float __K_I, float __K_D, float __I_Out_Max, float __Out_Max);

        void Set_K_P(float __K_P);
        void Set_K_I(float __K_I);
        void Set_K_D(float __K_D);
        void Set_I_Out_Max(float __I_Out_Max);
        void Set_Out_Max(float __Out_Max);
        void Set_Target(float __Target);
        void Set_Now(float __Now);
        void Set_Dead_Zone(float __Dead_Zone);
        void Set_Variable_Speed_I_A(float __Variable_Speed_I_A);
        void Set_Variable_Speed_I_B(float __Variable_Speed_I_B);
        void Set_I_Separate_Threshold(float __I_Separate_Threshold);
        void Set_D_First(Enum_D_First __D_First);

        float Get_Out();

        void Adjust_TIM_PeriodElapsedCallback();

    protected:

        //PID计时器周期, s
        float D_T = PID_CALCULATE_PERIOD;
        
        //PID的P
        float K_P;
        //PID的I
        float K_I;
        //PID的D
        float K_D;
        //积分限幅, 0为不限制
        float I_Out_Max;
        //输出限幅, 0为不限制
        float Out_Max;

        //目标值
        float Target = 0;
        //当前值
        float Now = 0;
        //误差
        float Error = 0;
        //输出值
        float Out = 0;

        //前向值
        float Pre_Value;
        //积分值
        float Integral_Error;
        //前向误差
        float Pre_Error;

        //PID单项输出
        //P输出
        float P_Out = 0;
        //I输出
        float I_Out = 0;
        //D输出
        float D_Out = 0;
        
        //死区, Error在其绝对值内不输出
        float Dead_Zone = 0;
        
        //变速积分, A和B为正数
        //A和B均为0, 普通积分
        //Error <= B, 普通积分
        //B < Error <= A + B, 变速积分
        //A + B < Error, 不积分
        //定速内段阈值, 0为不限制
        float Variable_Speed_I_A = 0;
        //变速区间, 0为不限制
        float Variable_Speed_I_B = 0;

        //积分分离阈值，需为正数, 0为不限制
        float I_Separate_Threshold = 0; 

        //微分先行
        Enum_D_First D_First = D_First_NO;
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
