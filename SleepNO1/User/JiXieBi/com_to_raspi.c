#include "com_to_raspi.h"

//using huart3
uint8_t receive_raspy_buffer[16];
int receive_raspy_pointer = 0;

void com_raspi_Init(){
    HAL_UART_Receive_IT(&RASPI_USINGUART,receive_raspy_buffer + receive_raspy_pointer,1);
}

void handle_received_data(){
    //数据帧格式: 帧头2个0x66 , 随后2个uint8_t数据表示角度  [0x66 0x66 p1 p1]
    if(receive_raspy_pointer == 0){
        if(receive_raspy_buffer[receive_raspy_pointer] == 0x66){
            receive_raspy_pointer++;
        }
        HAL_UART_Receive_IT(&RASPI_USINGUART,receive_raspy_buffer + receive_raspy_pointer,1);   
       
    }
    else if(receive_raspy_pointer == 1){
        if(receive_raspy_buffer[receive_raspy_pointer] == 0x66){
            receive_raspy_pointer = 2;
            HAL_UART_Receive_IT(&RASPI_USINGUART,receive_raspy_buffer + receive_raspy_pointer,2);  
        }
        else{
            receive_raspy_pointer = 0;
            HAL_UART_Receive_IT(&RASPI_USINGUART,receive_raspy_buffer + receive_raspy_pointer,1);     
        }   
    }
    else if(receive_raspy_pointer == 2){ //收到两个数据
        receive_raspy_pointer = 0;
        HAL_UART_Receive_IT(&RASPI_USINGUART,receive_raspy_buffer + receive_raspy_pointer,1); 

        //TODO:set para
        int8_t para = receive_raspy_buffer[2] * 256 + receive_raspy_buffer[3];
        JiXieBi_set_fourth_dj(para);
        //返回收到的数据
        HAL_UART_Transmit(&RASPI_USINGUART,receive_raspy_buffer+2,2,HAL_MAX_DELAY);
    }
}
