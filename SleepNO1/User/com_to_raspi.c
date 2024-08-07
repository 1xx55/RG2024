#include "com_to_raspi.h"

//varients
uint8_t receive_raspy_buffer[16];
uint8_t send_raspy_buffer[16];
uint8_t len_exdata[16] = {0,2,3,2}; //附加数据长度
int receive_raspy_pointer;

//EV
extern Class_Chassis Chassis;

//方便编程，写个宏定义，rcv_pointer指向当前接收到数据的开头
#define rcv_pointer receive_raspy_buffer + receive_raspy_pointer

//任务定义
uint16_t com_raspi_time_counter = 1000;
int8_t com_raspi_taskid = -1;

//functions
void com_raspi_Init(){
    receive_raspy_pointer = 0;
    com_raspi_taskid = -1;
    HAL_UART_Receive_IT(&RASPI_USINGUART,rcv_pointer,1);
}

void handle_received_data(){
    //数据帧格式: 帧头2个0x66,先判帧头
    if(receive_raspy_pointer <= 1){
        if(receive_raspy_buffer[receive_raspy_pointer] == MESSAGE_HEADER){
            receive_raspy_pointer++;
        }
        else receive_raspy_pointer = 0;    
        
        HAL_UART_Receive_IT(&RASPI_USINGUART,rcv_pointer,1);    
    }

    // 收到标志位
    else if(receive_raspy_pointer == 2){ 
        //如果有附加数据
        if(len_exdata[receive_raspy_buffer[2]]){ 
            //接收附加数据
            receive_raspy_pointer = 3;
            HAL_UART_Receive_IT(&RASPI_USINGUART,rcv_pointer,len_exdata[receive_raspy_buffer[2]]); 
        }

        //如果没有附加数据
        else{
            //处理该信息


            //处理完成，等待下一次数据
            receive_raspy_pointer = 0;
            HAL_UART_Receive_IT(&RASPI_USINGUART,rcv_pointer,1);        
        }
    }

    // 带附加数据
    else if(receive_raspy_pointer == 3){ 
        //处理该信息
        switch (receive_raspy_buffer[2])
        {
            case TO_STM32_ADJ_BLOCK:{
                //获取数据
                int16_t para = (int16_t)receive_raspy_buffer[3] * 256 + receive_raspy_buffer[4];
                //机械臂设置参数
                JiXieBi_set_fourth_dj(para);
                break;
            }
            
            case TO_STM32_CHASSIS_MOVE:{
                //获取数据
                float deg = ( (float)receive_raspy_buffer[3] * 256 + receive_raspy_buffer[4] ) / 2.0f / PI;
                float distance = receive_raspy_buffer[5] * CHASSIS_CM_TO_RAD;
                // 解算分量
                float ahead_distance = distance * cosf(deg);
                float left_distance = distance * sinf(deg);
                // 底盘运动指令
                Chassis.Set_add_rad(ahead_distance,left_distance,0);
                //启动底盘是否运动完成任务
                chassis_finish_task_start();
                break;  
            }

            case TO_STM32_CHASSIS_ROTATE:{
                //获取数据
                float rad = ((float)receive_raspy_buffer[3] * 256 + receive_raspy_buffer[4] - 180.0f) /2.0f / PI;
                //TODO:这里有个常数需要整定,底盘旋转角度与轮子旋转角度的换算关系
                rad = rad * CHASSIS_ROTATE_RAD_TO_WHEEL_RAD;
                // 底盘运动指令
                Chassis.Set_add_rad(0,0,rad);
                //启动底盘是否运动完成任务
                chassis_finish_task_start();

                break;
            }

            default:
                break;
        }
        //处理完成，等待下一次数据
        receive_raspy_pointer = 0;
        HAL_UART_Receive_IT(&RASPI_USINGUART,rcv_pointer,1); 
    }
}

void send_message_to_raspi(uint8_t message){
    //帧头
    send_raspy_buffer[0] = MESSAGE_HEADER;
    send_raspy_buffer[1] = MESSAGE_HEADER;
    //消息标志位
    send_raspy_buffer[2] = message;
    //发送
    HAL_UART_Transmit(&RASPI_USINGUART,send_raspy_buffer,3,HAL_MAX_DELAY);
    //TODO:带附加数据的发送。不过现在没有需求，就先不写了qwq不然该函数会复杂一点
}

// TASK
// epsilon: 轮子当前角度与设定角度可接受的误差，认为在误差内轮子即达到目标值，单位rad
#define epsilon 0.01f

void chassis_finish_task_start(){
    com_raspi_time_counter = 0;
    com_raspi_taskid = 0;
}

void COM_RASPI_TIM_IT(){
    if(com_raspi_taskid!=-1)com_raspi_time_counter++;
}

void COM_RASPI_TASK_SCHEDULE(){
    //task1:检查底盘是否运动完成,80ms查一次
    if(com_raspi_taskid == 0 && com_raspi_time_counter >= 8){
        char flag = 1;
        //检查底盘4个轮子角度现在值是否抵达目标值
        for(char i=0; i<4; i++){
            float diff = Chassis.Motor[i].Get_Angle_Target() - Chassis.Motor[i].Get_Angle_Now();
            if(fabs(diff) > epsilon) //轮子当前角度值未到达目标值
                flag = 0;
        }    

        if(!flag){ //轮子未达到目标值
            //重设counter,过80ms后再判
            com_raspi_time_counter = 0;
        }
        else{ //已达到
            //结束任务
            com_raspi_taskid = -1;
            //发送信息
            send_message_to_raspi(TO_RASPI_MOVE_FINISH);
        }
    }
}
