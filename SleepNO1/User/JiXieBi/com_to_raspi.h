#ifndef COM_TO_RASPI_H
#define COM_TO_RASPI_H

// includes
#include "main.h"
#include "usart.h"
#include "JiXieBi.h"

// macros
#define RASPI_USINGUART huart3
#define CATCH_START 0x01
#define SHOOT_END 0x02

// function declarations
void com_raspi_Init();
void handle_received_data();
void send_message_to_raspi(uint8_t message);
#endif
