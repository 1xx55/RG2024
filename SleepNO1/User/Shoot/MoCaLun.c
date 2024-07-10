#include "MoCaLun.h"
//using tim4 ch2,ch3 (PD13,14)

//移植代码只需修改此处宏定义
#define TMPORT1 htim4
#define TMPORT2 htim4
#define TMCH1 TIM_CHANNEL_2
#define TMCH2 TIM_CHANNEL_3

void MOCALUN_start(int speed_param){
    //speed_param: 350,375,400 三档速度
    __HAL_TIM_SetCompare(&TMPORT1,TMCH1,speed_param*2);
    __HAL_TIM_SetCompare(&TMPORT2,TMCH2,speed_param*2);
}

void MOCALUN_stop(){
    __HAL_TIM_SetCompare(&TMPORT1,TMCH1,1200);
    __HAL_TIM_SetCompare(&TMPORT2,TMCH2,1200);   
}

void MOCALUN_Init(){
    //满占空比是2000 已配置为500hz
    HAL_Delay(1000); //????
    HAL_TIM_PWM_Start(&TMPORT1,TMCH1);
    HAL_TIM_PWM_Start(&TMPORT2,TMCH2);
    MOCALUN_start(250);
    
}

