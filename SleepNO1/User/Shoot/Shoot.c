#include "Shoot.h"

//author 1xx55
//��������.��һ����ʱ�����
uint16_t time_counter = 1000;
int8_t taskid = -1;
int speed_para = 750;

void SHOOT_TIM_IT(TIM_HandleTypeDef *htim){
    if(htim!=&SHOOT_TIM) return;
    if(time_counter > 1000) return; //10s
    //���¼�������
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

void SHOOT_START(){ //��0��ʼ���� ��ʼִ������ time_counter = 1000�����������
    time_counter = 0;
    taskid = 0;
}

void SHOOT_ABRUPT(){ //������ֹ,ֹͣ����ģ�顣
    time_counter = 1000; //����ֹͣ
    taskid = -1;
    MOCALUN_stop();      //Ħ����ֹͣ
    FIX_POS_DJ_OPEN();   //�ſ�����
    MS_GO_DOWN();        //˿�������˶����ײ�
}

void SHOOT_set_speed(int speed){
    // speed para: 600�����ɲ���ȥ 700��
    speed_para = speed;
}

void SHOOT_Init(){
    //���������ʼ��
    MS_Init();
    FIX_POS_DJ_Init();
    MOCALUN_Init();
    HAL_TIM_Base_Start_IT(&SHOOT_TIM);
}
