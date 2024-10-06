#include "JiXieBi.h"

//author 1xx55
//定时器完成
uint16_t jixiebi_time_counter = 10000;
int8_t jixiebi_taskid = -1;
int16_t four_dj_para = 300;

int8_t have_next_jiaqu = 0;

void JiXieBi_TIM_IT(){
    if(jixiebi_taskid == -1 || jixiebi_taskid == TASK_ID_READY) return; 
    //更新计数变量
    jixiebi_time_counter++;
}

void JiXieBi_TASK_Schedule(){
    // Task1: 夹取&发射一体
    if(jixiebi_taskid == TASK_ID_READY){
        //if(!IS_SHOOT_BUSY()){
        jixiebi_taskid = 0;
        //}
    }

    if ( jixiebi_time_counter >= 1 && jixiebi_taskid == 0){
        AIR_PUMP_OPEN(); 
        jixiebi_taskid++;  
    } 
    // else if ( jixiebi_time_counter >= 2 && jixiebi_taskid == 1){
    //     SERVOCMD_MOVE_TIME_WRITE(1,320,800);
    //     SERVOCMD_MOVE_TIME_WRITE(2,190,800);
    //     SERVOCMD_MOVE_TIME_WRITE(3,1000,800);
    //     SERVOCMD_MOVE_TIME_WRITE(4,300,800);
    //     jixiebi_taskid++;  
    // } 
    // else if ( jixiebi_time_counter >= 87 && jixiebi_taskid == 2){
    //     SERVOCMD_MOVE_TIME_WRITE(1,425,500); //catch 2 
    //     SERVOCMD_MOVE_TIME_WRITE(2,190,500);
    //     SERVOCMD_MOVE_TIME_WRITE(3,1000,500);
    //     SERVOCMD_MOVE_TIME_WRITE(4,300,500);
    //     jixiebi_taskid++;  
    // } 
    // else if ( jixiebi_time_counter >= 167 && jixiebi_taskid == 3){
    //     SERVOCMD_MOVE_TIME_WRITE(1,300,300); //迅速catch 1
    //     SERVOCMD_MOVE_TIME_WRITE(2,190,300);
    //     SERVOCMD_MOVE_TIME_WRITE(3,1000,300);
    //     SERVOCMD_MOVE_TIME_WRITE(4,300,300);
    //     jixiebi_taskid = 103; 
    // }
    // else if (jixiebi_time_counter >= 200 && jixiebi_taskid == 103){
    //     SERVOCMD_MOVE_TIME_WRITE(1,300,300); //迅速catch 1
    //     SERVOCMD_MOVE_TIME_WRITE(2,190,300);
    //     //SERVOCMD_MOVE_TIME_WRITE(3,150,1600); //3号舵机直接拔起
    //     SERVOCMD_MOVE_TIME_WRITE(3,850,300);
    //     send_message_to_raspi(TO_RASPI_CATCH_START); //拿起来了，可以看
    //     SERVOCMD_MOVE_TIME_WRITE(4,300,300);
    //     jixiebi_taskid = 4;
    // }
    // else if ( jixiebi_time_counter >= 237 && jixiebi_taskid == 4){
    //     //SERVOCMD_MOVE_TIME_WRITE(1,420,700);
    //     SERVOCMD_MOVE_TIME_WRITE(1,300,500);
    //     SERVOCMD_MOVE_TIME_WRITE(2,160,500); //1,2号舵机调整好弧度
    //     SERVOCMD_MOVE_TIME_WRITE(3,550,500);
    //     SERVOCMD_MOVE_TIME_WRITE(4,300,500);
    //     if(!IS_Ms_At_Bottom()){
    //         //预防丝杆停在中间,阻塞一下,判定间隔1s
    //         MS_GO_DOWN();
    //         jixiebi_time_counter = 137;
    //     }
    //     else{
    //         //next sub(task
    //         jixiebi_taskid = 104; 
    //     }    
    // }
    // else if (jixiebi_time_counter >= 287 && jixiebi_taskid == 104){
    //     SERVOCMD_MOVE_TIME_WRITE(1,300,700);
    //     SERVOCMD_MOVE_TIME_WRITE(2,160,700);
    //     SERVOCMD_MOVE_TIME_WRITE(3,150,700);
    //     SERVOCMD_MOVE_TIME_WRITE(4,four_dj_para,700); //4号舵机调整
    //     jixiebi_taskid = 5;
    // }
    // else if ( jixiebi_time_counter >= 350 && jixiebi_taskid == 5){
    //     SERVOCMD_MOVE_TIME_WRITE(1,500,1000);
    //     SERVOCMD_MOVE_TIME_WRITE(2,670,1000); //2号1号向上 往前送
    //     SERVOCMD_MOVE_TIME_WRITE(3,205,1000); //3号稍微上抬一点
    //     SERVOCMD_MOVE_TIME_WRITE(4,300,1000);
    //     jixiebi_taskid++; 
    // }
    // else if ( jixiebi_time_counter >= 460 && jixiebi_taskid == 6){
    //     SERVOCMD_MOVE_TIME_WRITE(1,500,1000);
    //     SERVOCMD_MOVE_TIME_WRITE(2,670,1000);
    //     SERVOCMD_MOVE_TIME_WRITE(3,305,1000); //3号稍微上抬一点
    //     SERVOCMD_MOVE_TIME_WRITE(4,300,1000);
    //     jixiebi_taskid++;
    // }
    // else if ( jixiebi_time_counter >= 505 && jixiebi_taskid == 7){
    //     AIR_PUMP_CLOSE(); //关泵
    //     jixiebi_taskid++;
    // }
    // else if ( jixiebi_time_counter >= 565 && jixiebi_taskid == 8){
    //     JiXieBi_READY(); //机械臂复位
    //     SHOOT_START(); //开射
    //     jixiebi_taskid ++;
    // }
    // else if ( jixiebi_time_counter >= 765 && jixiebi_taskid == 9){
    //     send_message_to_raspi(TO_RASPI_JIAQU_FINISH);
    //     jixiebi_taskid ++;
    // }
    // else if ( jixiebi_time_counter >= 1165 && jixiebi_taskid == 10){
    //     //开射4s.完成任务，可接入下一夹取任务
    //     jixiebi_taskid = -1;
    // }


    else if ( jixiebi_time_counter >= 2 && jixiebi_taskid == 1){
        SERVOCMD_MOVE_TIME_WRITE(1,320,800);
        SERVOCMD_MOVE_TIME_WRITE(2,190,800);
        SERVOCMD_MOVE_TIME_WRITE(3,1000,800);
        jixiebi_taskid++;  
    } 
    else if ( jixiebi_time_counter >= 87 && jixiebi_taskid == 2){
        SERVOCMD_MOVE_TIME_WRITE(1,425,500); //catch 2 
        jixiebi_taskid++;  
    } 
    else if ( jixiebi_time_counter >= 187 && jixiebi_taskid == 3){
        SERVOCMD_MOVE_TIME_WRITE(1,300,300); //迅速catch 1
        jixiebi_taskid = 103; 
    }
    else if (jixiebi_time_counter >= 200 && jixiebi_taskid == 103){
        SERVOCMD_MOVE_TIME_WRITE(3,150,1600); //3号舵机直接拔起
        send_message_to_raspi(TO_RASPI_CATCH_START); //拿起来了，可以看
        jixiebi_taskid = 4;
    }
    else if ( jixiebi_time_counter >= 237 && jixiebi_taskid == 4){
        //SERVOCMD_MOVE_TIME_WRITE(1,420,700);
        SERVOCMD_MOVE_TIME_WRITE(2,160,700); //1,2号舵机调整好弧度
        if(!IS_Ms_At_Bottom()){
            //预防丝杆停在中间,阻塞一下,判定间隔1s
            MS_GO_DOWN();
            jixiebi_time_counter = 137;
        }
        else{
            //next sub(task
            jixiebi_taskid = 104; 
        }    
    }
    else if (jixiebi_time_counter >= 287 && jixiebi_taskid == 104){
        SERVOCMD_MOVE_TIME_WRITE(4,four_dj_para,500); //4号舵机调整
        jixiebi_taskid = 5;
    }
    else if ( jixiebi_time_counter >= 350 && jixiebi_taskid == 5){
        SERVOCMD_MOVE_TIME_WRITE(1,500,700);
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
        jixiebi_taskid ++;
    }
    else if ( jixiebi_time_counter >= 615 && jixiebi_taskid == 9){
        SHOOT_START(); //开射
        jixiebi_taskid ++;
    }

    else if ( jixiebi_time_counter >= 765 && jixiebi_taskid == 10){
        send_message_to_raspi(TO_RASPI_JIAQU_FINISH);
        jixiebi_taskid ++;
    }
    else if ( jixiebi_time_counter >= 1065 && jixiebi_taskid == 11){
        //开射4.5s.完成任务，可接入下一夹取任务
        jixiebi_taskid = -1;
    }
    //Task 1 end.
}

void JiXieBi_READY(){ 
    //2s
    SERVOCMD_MOVE_TIME_WRITE(1,320,2000); //-250
    SERVOCMD_MOVE_TIME_WRITE(2,190,2000); //-50
    SERVOCMD_MOVE_TIME_WRITE(3,1000,2000);
    SERVOCMD_MOVE_TIME_WRITE(4,300,2000); //catch 1
}

void JiXieBi_JIAQU(){ //从0开始计数 开始执行任务 time_counter = 10代表任务结束
    if (jixiebi_taskid == -1){
        jixiebi_time_counter = 0;
        jixiebi_taskid = TASK_ID_READY;
    }
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
    have_next_jiaqu = 0;
}

int IS_JiXieBi_CanLetChassisNextMove(){
    if(jixiebi_taskid == -1 || (jixiebi_taskid >=4 && jixiebi_taskid < 100 )) return 1;
    else return 0;
}

int IS_JiXieBi_Busy(){
    if(jixiebi_taskid == -1 || jixiebi_taskid == TASK_ID_READY)return 0;
    else return 1;
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
