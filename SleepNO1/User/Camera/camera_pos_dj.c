#include "camera_pos_dj.h"

//使用tim11 (PF7)
//20MS 20000 0.5-2.5MS 500-2500 -> 0-360(270)
int camera_angle = 0;

void CAMERA_POS_DJ_Init(){
    camera_angle = 0;
    __HAL_TIM_SetCompare(&htim11,TIM_CHANNEL_1,500);
    HAL_TIM_PWM_Start(&htim11,TIM_CHANNEL_1);
}

void __CAMERA_SET_ANGLE(){
  //理论范围2000,实际需要略大一点
  //__HAL_TIM_SetCompare(&htim11,TIM_CHANNEL_1,(int)(500.0+(double)(camera_angle)*2000.0/360.0) );
  //__HAL_TIM_SetCompare(&htim11,TIM_CHANNEL_1,(int)(500.0+(double)(camera_angle)*2100.0/360.0) );
  //__HAL_TIM_SetCompare(&htim11,TIM_CHANNEL_1,(int)(500.0+(double)(camera_angle)*2066.0/360.0) );
  //__HAL_TIM_SetCompare(&htim11,TIM_CHANNEL_1,(int)(500.0+(double)(camera_angle)*2033.0/360.0) );
  __HAL_TIM_SetCompare(&htim11,TIM_CHANNEL_1,(int)(500.0+(double)(camera_angle)*2024.0/360.0) );	  
}

void CAMERA_POS_DJ_ANGLE_SET(int angle){
  
  camera_angle = angle;
  camera_angle = (camera_angle%360+360)%360;
  __CAMERA_SET_ANGLE();
	
}

void CAMERA_POS_DJ_ANGLE_ADD(int add_deg){
  camera_angle += add_deg;
  camera_angle = (camera_angle%360+360)%360;
  __CAMERA_SET_ANGLE();
}