#include "JiXieBi.h"
#include "air_pump.h"
//author 1xx55
//定时器完成
uint16_t jixiebi_time_counter = 10000;
int8_t jixiebi_taskid = -1;

void JiXieBi_TIM_IT(){
    if(jixiebi_taskid == -1) return; 
    //更新计数变量
    jixiebi_time_counter++;
}

void JiXieBi_TASK_Schedule(){
    // Task1: 初始化
    // if      ( time_counter >= 0 && taskid == 0)    {FIX_POS_DJ_CLOSE();taskid++;}
    // else if ( time_counter >= 20 && taskid == 1)   {MS_GO_UP();taskid++;}
    // else if ( time_counter >= 220 && taskid == 2)  {MOCALUN_start(speed_para);taskid++;}
    // else if ( time_counter >= 470 && taskid == 3)  {MOCALUN_stop();taskid++;}
    // else if ( time_counter >= 520 && taskid == 4)  {MS_GO_DOWN();taskid++;}
    // else if ( time_counter >= 520 && taskid == 5)  {FIX_POS_DJ_OPEN();taskid++;}
}

void JiXieBi_Ready(){ //从0开始计数 开始执行任务 time_counter = 1000代表任务结束
    jixiebi_time_counter = 0;
    jixiebi_taskid = 0;
}

void JiXieBi_Init(){
    //机械臂初始化
    AIR_PUMP_Init();
    //机械臂的舵机回到初始姿态,5s
    SERVOCMD_MOVE_TIME_WRITE(1,900,2900);
    SERVOCMD_MOVE_TIME_WRITE(2,150,2900);
    SERVOCMD_MOVE_TIME_WRITE(3,890,2900);
    SERVOCMD_MOVE_TIME_WRITE(4,850,2900);
    AIR_PUMP_OPEN();
    HAL_Delay(3000);
    SERVOCMD_MOVE_TIME_WRITE(1,888,888);
    SERVOCMD_MOVE_TIME_WRITE(2,205,888);
    SERVOCMD_MOVE_TIME_WRITE(3,670,888);
    SERVOCMD_MOVE_TIME_WRITE(4,884,888);//结点1
    HAL_Delay(888);
    SERVOCMD_MOVE_TIME_WRITE(1,888,888);
    SERVOCMD_MOVE_TIME_WRITE(2,370,888);
    SERVOCMD_MOVE_TIME_WRITE(3,670,888);
    SERVOCMD_MOVE_TIME_WRITE(4,884,888);//结点2
    HAL_Delay(888);
    SERVOCMD_MOVE_TIME_WRITE(1,1000,888);
    SERVOCMD_MOVE_TIME_WRITE(2,370,888);
    SERVOCMD_MOVE_TIME_WRITE(3,670,888);
    SERVOCMD_MOVE_TIME_WRITE(4,884,888);//结点3
    HAL_Delay(888);
    SERVOCMD_MOVE_TIME_WRITE(2,370,888);
    SERVOCMD_MOVE_TIME_WRITE(3,360,888);
    HAL_Delay(888);
    SERVOCMD_MOVE_TIME_WRITE(2,160,888);
    SERVOCMD_MOVE_TIME_WRITE(3,360,888);
    HAL_Delay(888);
    SERVOCMD_MOVE_TIME_WRITE(2,160,888);
    SERVOCMD_MOVE_TIME_WRITE(3,150,888);//结点6
    HAL_Delay(888);
    SERVOCMD_MOVE_TIME_WRITE(2,420,888);
    SERVOCMD_MOVE_TIME_WRITE(3,150,888);//结点7
    HAL_Delay(888);
    SERVOCMD_MOVE_TIME_WRITE(2,570,888);
    SERVOCMD_MOVE_TIME_WRITE(3,205,888);//结点8
    HAL_Delay(888);
    AIR_PUMP_CLOSE();
}
