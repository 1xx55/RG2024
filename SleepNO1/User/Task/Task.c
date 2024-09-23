#include "Task.h"

#define TIM_10MS htim14

void Task_TIM_Init(){
    HAL_TIM_Base_Start_IT(&TIM_10MS);
}

void Task_10ms_TIM_IT(TIM_HandleTypeDef *htim){
    //配置为10ms定时器 这里是tim14
    if(htim != &TIM_10MS)return;
    SHOOT_TIM_IT();
    ACC_TIM14_IT();
    JiXieBi_TIM_IT();
    COM_RASPI_TIM_IT();
}

// in main
void Task_Schedule(){
    //设置这里的顺序似乎可以调整优先级，但实际上好像不会。
    ACC_TASK_SCHEDULE();
    JiXieBi_TASK_Schedule();
    SHOOT_TASK_Schedule();
    COM_RASPI_TASK_SCHEDULE();
}
