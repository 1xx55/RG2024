#include "JiXieBi.h"
#include "air_pump.h"
//author 1xx55
//定时器完成
uint16_t jixiebi_time_counter = 10000;
int8_t jixiebi_taskid = -1;
int16_t four_dj_para = 880;

void JiXieBi_TIM_IT(){
    if(jixiebi_taskid == -1) return; 
    //更新计数变量
    jixiebi_time_counter++;
}

void JiXieBi_TASK_Schedule(){
    // Task1: 夹取
    if ( jixiebi_time_counter >= 1 && jixiebi_taskid == 0){
        AIR_PUMP_OPEN(); 
        jixiebi_taskid++;  
    } 
    else if ( jixiebi_time_counter >= 2 && jixiebi_taskid == 1){
        SERVOCMD_MOVE_TIME_WRITE(1,570,800);
        SERVOCMD_MOVE_TIME_WRITE(2,240,800);
        SERVOCMD_MOVE_TIME_WRITE(3,1000,800);
        SERVOCMD_MOVE_TIME_WRITE(4,four_dj_para,800); //catch 1
        jixiebi_taskid++;  
    } 
    else if ( jixiebi_time_counter >= 87 && jixiebi_taskid == 2){
        SERVOCMD_MOVE_TIME_WRITE(1,635,500); //catch 2 ,停留久一点给视觉看 ,1s
        send_message_to_raspi(CATCH_START);
        jixiebi_taskid++;  
    } 
    else if ( jixiebi_time_counter >= 187 && jixiebi_taskid == 3){
        SERVOCMD_MOVE_TIME_WRITE(1,570,200); //迅速catch 1
        SERVOCMD_MOVE_TIME_WRITE(3,150,1600); //3号舵机直接拔起
        jixiebi_taskid++; 
    }
    else if ( jixiebi_time_counter >= 237 && jixiebi_taskid == 4){
        SERVOCMD_MOVE_TIME_WRITE(1,700,700);
        SERVOCMD_MOVE_TIME_WRITE(2,160,700); //1,2号舵机调整好弧度
        jixiebi_taskid++; 
    }
    else if ( jixiebi_time_counter >= 350 && jixiebi_taskid == 5){
        SERVOCMD_MOVE_TIME_WRITE(1,650,700);
        SERVOCMD_MOVE_TIME_WRITE(2,670,1000); //2号1号向上 往前送
        SERVOCMD_MOVE_TIME_WRITE(3,205,300); //3号稍微上抬一点
        jixiebi_taskid++; 
    }
    else if ( jixiebi_time_counter >= 460 && jixiebi_taskid == 6){
        SERVOCMD_MOVE_TIME_WRITE(3,305,200); //3号稍微上抬一点
        jixiebi_taskid++;
    }
    else if ( jixiebi_time_counter >= 505 && jixiebi_taskid == 7){
        AIR_PUMP_CLOSE(); //关泵
        jixiebi_taskid++;
    }
    else if ( jixiebi_time_counter >= 565 && jixiebi_taskid == 8){
        JiXieBi_READY(); //机械臂复位
        jixiebi_taskid = -1;
    }
    // Task 1 end.
}

void JiXieBi_READY(){ 
    //2s
    SERVOCMD_MOVE_TIME_WRITE(1,570,2000);
    SERVOCMD_MOVE_TIME_WRITE(2,240,2000);
    SERVOCMD_MOVE_TIME_WRITE(3,1000,2000);
    SERVOCMD_MOVE_TIME_WRITE(4,880,2000); //catch 1
}

void JiXieBi_JIAQU(){ //从0开始计数 开始执行任务 time_counter = 10代表任务结束
    jixiebi_time_counter = 0;
    jixiebi_taskid = 0;
}

void JiXieBi_set_fourth_dj(int16_t para){
    four_dj_para = para;
}

void JiXieBi_Init(){
    //机械臂初始化
    //HAL_UART_Init(&huart2);
    AIR_PUMP_Init();
    com_raspi_Init();
    HAL_Delay(10);
    JiXieBi_READY();
}

//机械臂的舵机回到初始姿态,5s
    // // SERVOCMD_MOVE_TIME_WRITE(1,600,2900);
    // // SERVOCMD_MOVE_TIME_WRITE(2,150,2900);
    // // SERVOCMD_MOVE_TIME_WRITE(3,890,2900);
    // // SERVOCMD_MOVE_TIME_WRITE(4,850,2900);
    // SERVOCMD_MOVE_TIME_WRITE(1,570,2900);
    // SERVOCMD_MOVE_TIME_WRITE(2,240,2900);
    // SERVOCMD_MOVE_TIME_WRITE(3,1000,2900);
    // SERVOCMD_MOVE_TIME_WRITE(4,880,2900);//catch 1
    // AIR_PUMP_OPEN();
    // HAL_Delay(3000);
    // // SERVOCMD_MOVE_TIME_WRITE(1,550,888);
    // // SERVOCMD_MOVE_TIME_WRITE(2,205,888);
    // // SERVOCMD_MOVE_TIME_WRITE(3,670,888);
    // // SERVOCMD_MOVE_TIME_WRITE(4,884,888);//结点1
    // //catch begin
    // SERVOCMD_MOVE_TIME_WRITE(1,635,888); //catch 2
    // HAL_Delay(888);
    // SERVOCMD_MOVE_TIME_WRITE(1,570,888); //catch 1
    // HAL_Delay(888);
    // //catch end
    // SERVOCMD_MOVE_TIME_WRITE(1,550,888);
    // SERVOCMD_MOVE_TIME_WRITE(2,370,888);
    // SERVOCMD_MOVE_TIME_WRITE(3,670,888);
    // SERVOCMD_MOVE_TIME_WRITE(4,884,888);//结点2
    // HAL_Delay(888);
    // SERVOCMD_MOVE_TIME_WRITE(1,700,888);
    // SERVOCMD_MOVE_TIME_WRITE(2,370,888);
    // SERVOCMD_MOVE_TIME_WRITE(3,670,888);
    // SERVOCMD_MOVE_TIME_WRITE(4,884,888);//结点3
    // HAL_Delay(888);
    // SERVOCMD_MOVE_TIME_WRITE(2,370,888);
    // SERVOCMD_MOVE_TIME_WRITE(3,360,888);
    // HAL_Delay(888);
    // SERVOCMD_MOVE_TIME_WRITE(2,160,888);
    // SERVOCMD_MOVE_TIME_WRITE(3,360,888);
    // HAL_Delay(888);
    // SERVOCMD_MOVE_TIME_WRITE(2,160,888);
    // SERVOCMD_MOVE_TIME_WRITE(3,150,888);//结点6
    // HAL_Delay(1888);
    // SERVOCMD_MOVE_TIME_WRITE(2,420,888);
    // SERVOCMD_MOVE_TIME_WRITE(3,150,888);//结点7
    // SERVOCMD_MOVE_TIME_WRITE(4,700,888);//给视觉调角度用 前面延时长一点
    // HAL_Delay(888);
    // SERVOCMD_MOVE_TIME_WRITE(1,650,888);
    // SERVOCMD_MOVE_TIME_WRITE(2,570,888);
    // SERVOCMD_MOVE_TIME_WRITE(3,205,888);//结点8
    // HAL_Delay(888);