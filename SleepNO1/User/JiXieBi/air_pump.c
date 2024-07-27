#include "air_pump.h"

//Author: 1xx55
//�ļ�˵�������á�

void AIR_PUMP_Init(){
    HAL_GPIO_WritePin(B_ENA1_GPIO_Port,B_ENA1_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(B_ENA2_GPIO_Port,B_ENA2_Pin,GPIO_PIN_RESET);

    // 20240727发现泵的正负极是反的，已调整。
    HAL_GPIO_WritePin(B_INA1_GPIO_Port,B_INA1_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(B_INA2_GPIO_Port,B_INA2_Pin,GPIO_PIN_SET);

    HAL_GPIO_WritePin(B_INB1_GPIO_Port,B_INB1_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(B_INB2_GPIO_Port,B_INB2_Pin,GPIO_PIN_RESET);
    
    AIR_PUMP_CLOSE();
}

void AIR_PUMP_OPEN(){
    //beng
    HAL_GPIO_WritePin(B_ENA1_GPIO_Port,B_ENA1_Pin,GPIO_PIN_SET);
    //famen
    HAL_GPIO_WritePin(B_ENA2_GPIO_Port,B_ENA2_Pin,GPIO_PIN_RESET);
}

void AIR_PUMP_CLOSE(){
    //beng
    HAL_GPIO_WritePin(B_ENA1_GPIO_Port,B_ENA1_Pin,GPIO_PIN_RESET);
    //famen
    HAL_GPIO_WritePin(B_ENA2_GPIO_Port,B_ENA2_Pin,GPIO_PIN_SET);

}

// void AIR_PUMP_CLOSE2(){
//     //只关泵.不关阀门
//     //beng
//     HAL_GPIO_WritePin(B_ENA1_GPIO_Port,B_ENA1_Pin,GPIO_PIN_RESET);
// }