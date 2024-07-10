#ifndef MO_CA_LUN_H
#define MO_CA_LUN_H

// includes
#include "gpio.h"
#include "main.h"
#include "tim.h"
// macros


// function declarations
void MOCALUN_Init();

void MOCALUN_start(int speed_param);
void MOCALUN_stop();

// specials


#endif
