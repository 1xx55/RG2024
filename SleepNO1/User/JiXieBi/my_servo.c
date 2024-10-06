#include "my_servo.h"

//Author: 1xx55
//os: �ҷ���һ������,��ʱֻ��Ҫʵ��һ��ָ��ķ��͹��ܾͿ����ˡ������õ��Դ������ֲ���
//��ʹ��uart1����

#define GET_LOW_BYTE(A) ((uint8_t)(A))
//�꺯�� ���A�ĵͰ�λ
#define GET_HIGH_BYTE(A) ((uint8_t)((A) >> 8))
//�꺯�� ���A�ĸ߰�λ
#define BYTE_TO_HW(A, B) ((((uint16_t)(A)) << 8) | (uint8_t)(B))
//�꺯�� ���ߵذ�λ�ϳ�Ϊʮ��λ

// ����֡��ʽ: 
//     ֡ͷ     ID��   ���ݳ���   ָ��      ����        У���
//  0x55 0x55   ID     Length    cmd   prm1...prmN   Checksum
// ����˵���� 02+���߶��ͨ��Э��.pdf
uint8_t buf[32];
void SERVOCMD_MOVE_TIME_WRITE(uint8_t servo_id,uint16_t arg_para,uint16_t time_para){
    // ������id���(0-255), �ǶȲ���(0-1000)->0-240��, ʱ�����(0-30000)ms
    // ����: ���Ϊid�Ķ������time_para msʱ���ƶ���arg_paraλ��

    // ��䴫������
    // ֡ͷ
    buf[0] = SERIAL_SERVO_FRAME_HEADER; 
    buf[1] = SERIAL_SERVO_FRAME_HEADER;
    // ID��
    buf[2] = servo_id;
    // ��4������, ���������ݳ���Ϊ7
    buf[3] = 7;
    // ָ��
    buf[4] = SERIAL_SERVO_MOVE_TIME_WRITE;
    // ����, ����Ϊʱ��͡���8λ, �Ƕȵ͡���8λ
    buf[5] = GET_LOW_BYTE(arg_para);
    buf[6] = GET_HIGH_BYTE(arg_para);
    buf[7] = GET_LOW_BYTE(time_para);
    buf[8] = GET_HIGH_BYTE(time_para);
    // У���
    buf[9] = serial_servo_checksum(buf);

    //����
    HAL_UART_Transmit(&huart2, buf, buf[3]+3, HAL_MAX_DELAY);
    //not this
    //HAL_UART_Transmit_IT(&huart2, buf, buf[3]+3);
}
