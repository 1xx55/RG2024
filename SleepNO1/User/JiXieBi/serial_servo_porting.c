// #include "serial_servo.h"
// #include "gpio.h"
// #include "usart.h"
// #include "stm32f4xx_hal.h"
// //#include "cmsis_os2.h"
// #include <string.h>

// /* 全局变量 */
// SerialServoControllerTypeDef serial_servo_controller;
// //extern osSemaphoreId_t serial_servo_rx_completeHandle;

// static int serial_write_and_read(SerialServoControllerTypeDef *self, SerialServoCmdTypeDef *frame, bool tx_only)
// {
//     int ret = 0;
//     /* 进入写模式 */
//     // HAL_GPIO_WritePin(SERIAL_SERVO_RX_EN_GPIO_Port, SERIAL_SERVO_RX_EN_Pin, GPIO_PIN_SET);
//     // HAL_GPIO_WritePin(SERIAL_SERVO_TX_EN_GPIO_Port, SERIAL_SERVO_TX_EN_Pin, GPIO_PIN_RESET);
//     memcpy(&self->tx_frame, frame, sizeof(SerialServoCmdTypeDef));
//     self->tx_byte_index = 0;
//     self->tx_only = tx_only;
//     self->rx_state = SERIAL_SERVO_RECV_STARTBYTE_1;
//     __HAL_UART_CLEAR_FLAG(&huart6, UART_FLAG_RXNE);
// 	__HAL_UART_CLEAR_FLAG(&huart6, UART_FLAG_TC);
// 	__HAL_UART_CLEAR_FLAG(&huart6, UART_FLAG_TXE);
//     __HAL_UART_ENABLE_IT(&huart6, UART_IT_TXE);
// 	__HAL_UART_ENABLE_IT(&huart6, UART_IT_TC);
//     // if(osOK != osSemaphoreAcquire(serial_servo_rx_completeHandle, self->proc_timeout)) {
//     //     ret = -1;
//     // }
//     // HAL_GPIO_WritePin(SERIAL_SERVO_RX_EN_GPIO_Port, SERIAL_SERVO_RX_EN_Pin, GPIO_PIN_SET);
//     // HAL_GPIO_WritePin(SERIAL_SERVO_TX_EN_GPIO_Port, SERIAL_SERVO_TX_EN_Pin, GPIO_PIN_SET);
//     return ret;
// }


// void serial_servo_init(void)
// {
//     serial_servo_controller_object_init(&serial_servo_controller);
//     serial_servo_controller.proc_timeout = 4;
//     serial_servo_controller.serial_write_and_read = serial_write_and_read;
//     //osSemaphoreAcquire(serial_servo_rx_completeHandle, 0);
//     /* 写入模式, 只有在带接收过程的指令才会打开接收 */
//     // HAL_GPIO_WritePin(SERIAL_SERVO_RX_EN_GPIO_Port, SERIAL_SERVO_RX_EN_Pin, GPIO_PIN_SET);
//     // HAL_GPIO_WritePin(SERIAL_SERVO_TX_EN_GPIO_Port, SERIAL_SERVO_TX_EN_Pin, GPIO_PIN_SET);
//     __HAL_UART_CLEAR_FLAG(&huart6, UART_FLAG_TXE);
//     __HAL_UART_CLEAR_FLAG(&huart6, UART_FLAG_TC);
//     __HAL_UART_CLEAR_FLAG(&huart6, UART_FLAG_RXNE);
//     __HAL_UART_ENABLE_IT(&huart6, UART_IT_RXNE);
// }

