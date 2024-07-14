#include "camera_pos_dj.h"

//使用tim11 (PF7)
//20MS 20000 0.5-2.5MS 500-2500 -> 0-360(270)

void CAMERA_POS_DJ_Init(){
    __HAL_TIM_SetCompare(&htim11,TIM_CHANNEL_1,500);
    HAL_TIM_PWM_Start(&htim11,TIM_CHANNEL_1);
}

void CAMERA_POS_DJ_ANGLE(int angle){
  __HAL_TIM_SetCompare(&htim11,TIM_CHANNEL_1,(int)(500+(float)(angle)/360.0*2000.0) );
	
}