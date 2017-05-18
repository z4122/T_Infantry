#ifndef DRIVERS_UARTJUDGE_USER_H
#define DRIVERS_UARTJUDGE_USER_H

#include "utilities_iopool.h"

typedef struct{
	uint16_t voltage;
	uint16_t electricity;
	uint16_t remainPower;
}JudgePower_t;
IOPoolDeclare(judgePowerUartIOPool, JudgePower_t);

#endif
