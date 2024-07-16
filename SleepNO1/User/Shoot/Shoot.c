#include "Shoot.h"

//author 1xx55
//发射命令.用一个定时器完成
uint16_t time_counter = 1000;
int8_t taskid = -1;
int speed_para = 750;

void SHOOT_TIM_IT(TIM_HandleTypeDef *htim){
    if(htim!=&SHOOT_TIM) return;
    if(time_counter > 1000) return; //10s
    //更新计数变量
    time_counter++;
}

void SHOOT_TASK_Schedule(){
    //put in main while 1
    if      ( time_counter >= 0 && taskid == 0)    {FIX_POS_DJ_CLOSE();taskid++;}
    else if ( time_counter >= 20 && taskid == 1)   {MS_GO_UP();taskid++;}
    else if ( time_counter >= 220 && taskid == 2)  {MOCALUN_start(speed_para);taskid++;}
    else if ( time_counter >= 470 && taskid == 3)  {MOCALUN_stop();taskid++;}
    else if ( time_counter >= 520 && taskid == 4)  {MS_GO_DOWN();taskid++;}
    else if ( time_counter >= 520 && taskid == 5)  {FIX_POS_DJ_OPEN();taskid++;}
}

void SHOOT_START(){ //从0开始计数 开始执行任务 time_counter = 1000代表任务结束
    time_counter = 0;
    taskid = 0;
}

void SHOOT_ABRUPT(){ //意外终止,停止所有模块。
    time_counter = 1000; //任务停止
    taskid = -1;
    MOCALUN_stop();      //摩擦轮停止
    FIX_POS_DJ_OPEN();   //张开夹子
    MS_GO_DOWN();        //丝杆向下运动到底部
}

void SHOOT_set_speed(int speed){
    // speed para: 600无力飞不出去 700劲
    speed_para = speed;
}

void SHOOT_Init(){
    //发射机构初始化
    MS_Init();
    FIX_POS_DJ_Init();
    MOCALUN_Init();
    HAL_TIM_Base_Start_IT(&SHOOT_TIM);
}
