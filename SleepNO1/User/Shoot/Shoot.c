#include "Shoot.h"

//author 1xx55
//��������.��һ����ʱ�����
uint16_t time_counter = 1000;
int8_t taskid = -1;
int speed_para = 750;

void SHOOT_TIM_IT(){
    if(taskid == -1) return; //
    //���¼�������
    time_counter++;
}

void SHOOT_TASK_Schedule(){
    //put in main while 1
    //mission 1: shoot start
    if      ( time_counter >= 0 && taskid == 0)    {FIX_POS_DJ_CLOSE();taskid++;}
    else if ( time_counter >= 20 && taskid == 1)   {MS_GO_UP();taskid++;}
    else if ( time_counter >= 220 && taskid == 2)  {MOCALUN_start(speed_para);taskid++;}
    else if ( time_counter >= 470 && taskid == 3)  {
        MOCALUN_stop();
        send_message_to_raspi(SHOOT_END);
        taskid++;
        }
    else if ( time_counter >= 520 && taskid == 4)  {MS_GO_DOWN();taskid++;}
    else if ( time_counter >= 520 && taskid == 5)  {FIX_POS_DJ_OPEN();taskid=-1;}
    //mission 1 end

}

void SHOOT_START(){ //��0��ʼ���� ��ʼִ������ time_counter = 1000�����������
    time_counter = 0;
    taskid = 0;
}

void __SHOOT_123(){
    time_counter = 0;
    taskid = 7;
}

void SHOOT_ABRUPT(){ //������ֹ,ֹͣ����ģ�顣
    time_counter = 1000; //����ֹͣ
    taskid = -1;
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
