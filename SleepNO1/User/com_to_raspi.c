#include "com_to_raspi.h"

//varients
unsigned char receive_raspy_buffer[16];
unsigned char send_raspy_buffer[16];
unsigned char len_exdata[16] = {0,2,3,2}; //附加数据长度
int receive_raspi_pointer = 0;

//EV
extern Class_Chassis Chassis;

//方便编程，写个宏定义，rcv_pointer指向当前接收到数据的开头
#define rcv_pointer receive_raspy_buffer + receive_raspi_pointer

//任务定义
unsigned short com_raspi_time_counter = 1000;
//暂无任务id定义,因为没有需要触发激活的任务

//此处有一个循环队列储存树莓派下达的系列动作指令
#define QUEUE_MAX_LEN 32
char __movepara_queue_phead = 0, __movepara_queue_pback = 0;
MoveParaTypeDef Mov_mission_queue[QUEUE_MAX_LEN];

//functions
void com_raspi_Init(){
    receive_raspi_pointer = 0;
    com_raspi_time_counter = 1000;
    //com_raspi_taskid = -1;
    __movepara_queue_phead = 0, __movepara_queue_pback = 0;

    HAL_UART_Receive_IT(&RASPI_USINGUART,rcv_pointer,1);
}

void handle_received_data(){
    //数据帧格式: 帧头2个0x66,先判帧头
    if(receive_raspi_pointer <= 1){
        if(receive_raspy_buffer[receive_raspi_pointer] == MESSAGE_HEADER){
            receive_raspi_pointer++;
        }
        else receive_raspi_pointer = 0;    
        
        HAL_UART_Receive_IT(&RASPI_USINGUART,rcv_pointer,1);    
    }

    // 收到标志位
    else if(receive_raspi_pointer == 2){ 
        //如果有附加数据
        if(len_exdata[receive_raspy_buffer[2]]){ 
            //接收附加数据
            receive_raspi_pointer = 3;
            HAL_UART_Receive_IT(&RASPI_USINGUART,rcv_pointer,len_exdata[receive_raspy_buffer[2]]); 
        }

        //如果没有附加数据
        else{
            //处理该信息


            //处理完成，等待下一次数据
            receive_raspi_pointer = 0;
            HAL_UART_Receive_IT(&RASPI_USINGUART,rcv_pointer,1);        
        }
    }

    // 带附加数据
    else if(receive_raspi_pointer == 3){ 
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
                // 存放底盘运动指令
                // Chassis.Set_add_rad(ahead_distance,left_distance,0);
                Mov_mission_queue[__movepara_queue_phead].ahead = ahead_distance;
                Mov_mission_queue[__movepara_queue_phead].left = left_distance;
                Mov_mission_queue[__movepara_queue_phead].rotate = 0.0f;
                //队头指针更新
                __movepara_queue_phead = (__movepara_queue_phead + 1) % QUEUE_MAX_LEN;

                break;  
            }

            case TO_STM32_CHASSIS_ROTATE:{
                //获取数据
                float rad = ((float)receive_raspy_buffer[3] * 256 + receive_raspy_buffer[4] - 180.0f) /2.0f / PI;
                //TODO:这里有个常数需要整定,底盘旋转角度与轮子旋转角度的换算关系
                rad = rad * CHASSIS_ROTATE_RAD_TO_WHEEL_RAD;
                // 存放底盘运动指令
                // Chassis.Set_add_rad(0,0,rad);
                Mov_mission_queue[__movepara_queue_phead].ahead = 0.0f;
                Mov_mission_queue[__movepara_queue_phead].left = 0.0f;
                Mov_mission_queue[__movepara_queue_phead].rotate = rad;
                //队头指针更新
                __movepara_queue_phead = (__movepara_queue_phead + 1) % QUEUE_MAX_LEN;

                break;
            }

            default:
                break;
        }
        //处理完成，等待下一次数据
        receive_raspi_pointer = 0;
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

void COM_RASPI_TIM_IT(){
    //if(com_raspi_taskid!=-1)
        com_raspi_time_counter++;
}

void COM_RASPI_TASK_SCHEDULE(){
    // task1: 检查底盘是否运动完成,80ms查一次,时刻检查。
    // 先定义一个变量,记一下有没有发送完成信息.0为未发送,1为已发送.
    static char send_msg_tag = 1;
    // task1 start.
    if(com_raspi_time_counter >= 8){
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
        else{ //已达到目标值，小车停止。
            //考虑任务队列中是否有任务。
            if(__movepara_queue_pback != __movepara_queue_phead){ 
                //如果还有任务，则小车执行下一任务
                Chassis.Set_add_rad(Mov_mission_queue[__movepara_queue_pback].ahead,
                                    Mov_mission_queue[__movepara_queue_pback].left,
                                    Mov_mission_queue[__movepara_queue_pback].rotate
                                    );
                //更新队列指针
                __movepara_queue_pback = (__movepara_queue_pback + 1)% QUEUE_MAX_LEN;
                //重置发送完成信息变量
                send_msg_tag = 0; 
            }
            else{ //暂时没任务了
                //发送信息
                if(!send_msg_tag){ //如果本次系列任务没发过
                    //发送
                    send_message_to_raspi(TO_RASPI_MOVE_FINISH);
                    //标记：此次系列任务已发送完成信息给树莓派.
                    send_msg_tag = 1;
                }
            }
            
        }
    }
}
