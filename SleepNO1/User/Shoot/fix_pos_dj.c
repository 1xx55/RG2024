#include "fix_pos_dj.h"

//使用tim13 (PF8)
//[Record]舵机:夹紧775，然后790。张开1100

#define ZHANGKAI 1100
#define JIAJIN 840

void FIX_POS_DJ_Init(){
    __HAL_TIM_SetCompare(&htim13,TIM_CHANNEL_1,ZHANGKAI);
    HAL_TIM_PWM_Start(&htim13,TIM_CHANNEL_1);
}

void FIX_POS_DJ_OPEN(){
    __HAL_TIM_SetCompare(&htim13,TIM_CHANNEL_1,ZHANGKAI);
}

void FIX_POS_DJ_CLOSE(){
    __HAL_TIM_SetCompare(&htim13,TIM_CHANNEL_1,JIAJIN);
}
