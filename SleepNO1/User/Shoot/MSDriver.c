#include "MSDriver.h"


typedef enum{
    DIR_UP=1,
    DIR_DOWN=0
} DIR_status; //定义方向枚举变量，便于编程

// 加速部分使用参数
int acc_cnt = 66, acc_id = 66;
//单位10ms 设置特定加速的时间
int acc_time[11]={2,4,6,8,10,12,14,16,18,20,22};

//functions

//获取丝杆运动方向 （主要与DIR_L引脚电平配合）
DIR_status get_dir(){
    if(HAL_GPIO_ReadPin(MSDriver_DIR_L_GPIO_Port,MSDriver_DIR_L_Pin) == GPIO_PIN_RESET){
        return DIR_UP;
    }
    else return DIR_DOWN;
}

//设置丝杆运动方向
void set_dir(DIR_status dir){
    if(dir == DIR_UP){
        HAL_GPIO_WritePin(MSDriver_DIR_L_GPIO_Port,MSDriver_DIR_L_Pin,GPIO_PIN_RESET);
    }
    else HAL_GPIO_WritePin(MSDriver_DIR_L_GPIO_Port,MSDriver_DIR_L_Pin,GPIO_PIN_SET);
}


void GDM_EXTI(uint16_t GPIO_PIN){
    switch (GPIO_PIN)
    {
    //判断哪一个光电门感应到遮挡。
    case GDM_UP_Pin:
    {
        if(HAL_GPIO_ReadPin(GDM_UP_GPIO_Port,GDM_UP_Pin) == GPIO_PIN_SET && get_dir() == DIR_UP){
            //readpin确认由该光电门被遮挡产生中断且丝杆运动方向需要调转时,停止丝杆且换向。
            //必须写readpin,因为上电时会有电平波动，也会触发该中断
            HAL_GPIO_WritePin(MSDriver_ENA_L_GPIO_Port,MSDriver_ENA_L_Pin,GPIO_PIN_RESET);
            set_dir(DIR_DOWN);   
        }
        break;
    }

    case GDM_DOWN_Pin:
    {
        if(HAL_GPIO_ReadPin(GDM_DOWN_GPIO_Port,GDM_DOWN_Pin) == GPIO_PIN_SET && get_dir() == DIR_DOWN){
            HAL_GPIO_WritePin(MSDriver_ENA_L_GPIO_Port,MSDriver_ENA_L_Pin,GPIO_PIN_RESET);
            set_dir(DIR_UP);   
        }
        break;
    }  

    default:
        break;

    }
}

void set_acc_speed(int speed){
    //speed 只建议取值 1-10 ,虽然可取到15. 暂不考虑speed取浮点数值
    if(speed<1||speed>15)return;
    //计数参数
    int para = 1000/speed ; 
    //para 控制 PWM 周期 , speed = 1 时 para=1000 , 周期1000us=1ms
    __HAL_TIM_SET_AUTORELOAD(&htim4,para); 
    __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_1,para/2); //50%占空比
}

void MS_GO(){
    //如果已经在动 不开启新的go
    if(HAL_GPIO_ReadPin(MSDriver_ENA_L_GPIO_Port,MSDriver_ENA_L_Pin) == GPIO_PIN_SET)return;
    //开始移动，直到中断触发停止。加速过程计数由定时器完成，不占用系统时钟资源
    //
    //HAL_TIM_Base_Start_IT(&htim14);
    //参数初始化
    acc_cnt = 0; acc_id=0;
    //设置初始速度
    set_acc_speed(1);
    //开始走
    HAL_GPIO_WritePin(MSDriver_ENA_L_GPIO_Port,MSDriver_ENA_L_Pin,GPIO_PIN_SET);
}

void MS_GO_UP(){
    if(HAL_GPIO_ReadPin(GDM_UP_GPIO_Port,GDM_UP_Pin) == GPIO_PIN_SET)return;
    set_dir(DIR_UP);
    MS_GO();
}

void MS_GO_DOWN(){
    if(HAL_GPIO_ReadPin(GDM_DOWN_GPIO_Port,GDM_DOWN_Pin) == GPIO_PIN_SET)return;
    set_dir(DIR_DOWN);
    MS_GO();
}

void ACC_TIM14_IT(){
    if(acc_id < 10)acc_cnt++; 
}

void ACC_TASK_SCHEDULE(){
    if(acc_id < 10 && acc_cnt >= acc_time[acc_id]){
        set_acc_speed(acc_id+1);
        acc_id++;   
    }
}

void MS_Init(){
    //初始化。需要将其复位至初始位置
    //引脚初始化 . PUL_L用 TIM4 CH1
    //高电平组只需设高电平即可，不用动。
    HAL_GPIO_WritePin(MSDriver_DIR_H_GPIO_Port,MSDriver_DIR_H_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(MSDriver_ENA_H_GPIO_Port,MSDriver_ENA_H_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(MSDriver_PUL_H_GPIO_Port,MSDriver_PUL_H_Pin,GPIO_PIN_SET);
    //低电平组设置。
    set_dir(DIR_UP);
    HAL_GPIO_WritePin(MSDriver_ENA_L_GPIO_Port,MSDriver_ENA_L_Pin,GPIO_PIN_RESET);
    set_acc_speed(1);
    HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
    //光电门不需要初始化。一切已在配置中。

    //复位:如果不在底端，方向朝下运动到停止。
    if(HAL_GPIO_ReadPin(GDM_DOWN_GPIO_Port,GDM_DOWN_Pin) == GPIO_PIN_RESET){
        HAL_Delay(10);
        MS_GO_DOWN();
        while(HAL_GPIO_ReadPin(GDM_DOWN_GPIO_Port,GDM_DOWN_Pin) == GPIO_PIN_RESET){ //不在底部
            //初始化时因为不在main函数里面 只能这样加速了
            ACC_TASK_SCHEDULE();
        }
    }
}
