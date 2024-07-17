#include "MoCaLun.h"
//using tim4 ch2,ch3 (PD13,14)

//移植代码只需修改此处宏定义
#define TMPORT1 htim8
#define TMPORT2 htim8
#define TMCH1 TIM_CHANNEL_1
#define TMCH2 TIM_CHANNEL_2

void MOCALUN_start(int speed_param){
    //speed_param: 速度参数.(蜜汁)
    __HAL_TIM_SetCompare(&TMPORT1,TMCH1,speed_param*2);
    __HAL_TIM_SetCompare(&TMPORT2,TMCH2,speed_param*2);
}

void MOCALUN_stop(){
    // __HAL_TIM_SetCompare(&TMPORT1,TMCH1,1200);
    // __HAL_TIM_SetCompare(&TMPORT2,TMCH2,1200);   
    MOCALUN_start(250);
}

void MOCALUN_Init(){
    //满占空比是2000 已配置为500hz
    MOCALUN_start(250);
    HAL_TIM_PWM_Start(&TMPORT1,TMCH1);
    HAL_TIM_PWM_Start(&TMPORT2,TMCH2);
    HAL_Delay(3000);//3s
    MOCALUN_stop();
}

