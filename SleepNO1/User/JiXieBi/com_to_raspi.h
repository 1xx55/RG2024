#ifndef COM_TO_RASPI_H
#define COM_TO_RASPI_H

// includes
#include "main.h"
#include "usart.h"
#include "JiXieBi.h"

// macros
#define RASPI_USINGUART huart3
// function declarations
void com_raspi_Init();
void handle_received_data();

#endif
