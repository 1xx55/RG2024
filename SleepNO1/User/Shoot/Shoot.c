#include "Shoot.h"

//author 1xx55
//��������.��һ����ʱ�����
int8_t shoot_taskid = -1;
uint16_t time_counter = 1000;
int speed_para = 730;

void SHOOT_TIM_IT(){
    if(shoot_taskid == -1 || shoot_taskid == TASK_ID_READY) return; //
    //���¼�������
    time_counter++;
}

void SHOOT_TASK_Schedule(){
    //put in main while 1
    //检查与夹取的互斥
    if (shoot_taskid == TASK_ID_READY){
        if(!IS_JiXieBi_Busy()) shoot_taskid = 0;
    }

    //mission 1: shoot start
    if      ( time_counter >  0 && shoot_taskid == 0)    {FIX_POS_DJ_CLOSE();shoot_taskid++;}
    else if ( time_counter >= 20 && shoot_taskid == 1)   {MS_GO_UP();shoot_taskid++;}
    else if ( time_counter >= 120 && shoot_taskid == 2)  {//MOCALUN_start(speed_para);
    shoot_taskid++;}
    else if ( time_counter >= 220 && shoot_taskid == 3)  { FIX_POS_DJ_OPEN();shoot_taskid++;}
    else if ( time_counter >= 370 && shoot_taskid == 4)  {
        //MOCALUN_stop();
        shoot_taskid++;
    }
    else if ( time_counter >= 400 && shoot_taskid == 5)  {MS_GO_DOWN();shoot_taskid++;}
    else if ( time_counter >= 600 && shoot_taskid == 6 && IS_Ms_At_Bottom())  {send_message_to_raspi(TO_RASPI_SHOOT_END);shoot_taskid=-1;}
    //mission 1 end

}

void SHOOT_START(){ //��0��ʼ���� ��ʼִ������ time_counter = 1000�����������
    if(shoot_taskid == -1){
        time_counter = 0;
        shoot_taskid = TASK_ID_READY;
    }
}

// void __SHOOT_123(){
//     time_counter = 0;
//     shoot_taskid = 7;
// }

void SHOOT_ABRUPT(){ //������ֹ,ֹͣ����ģ�顣
    time_counter = 1000; //����ֹͣ
    shoot_taskid = -1;
    MOCALUN_stop();      //Ħ����ֹͣ
    FIX_POS_DJ_OPEN();   //�ſ�����
    MS_GO_DOWN();        //˿�������˶����ײ�
}

void SHOOT_set_speed(int speed){
    // speed para: 600�����ɲ���ȥ 700�� 750�е�Զ
    speed_para = speed;
}

void SHOOT_Init(){
    //���������ʼ��
    
    MS_Init();
    FIX_POS_DJ_Init();
    MOCALUN_Init();
}

int IS_SHOOT_BUSY(){
    //if(shoot_taskid != -1) return 1;
    //else return 0;
    if(IS_Ms_At_Bottom() && (shoot_taskid == -1 || shoot_taskid == TASK_ID_READY) ) return 0;
    else return 1; 
}