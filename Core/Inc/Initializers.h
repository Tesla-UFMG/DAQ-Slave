/*
 * initializers.h
 *
 */

#ifndef INITIALIZERS_H_
#define INITIALIZERS_H_


#include "stm32h7xx.h"

void init_CAN();
void debug_temp(uint16_t debug_periodo_ms); //não tinha ela no .h, apesar de estar no .c

#endif /* INITIALIZERS_H_ */
