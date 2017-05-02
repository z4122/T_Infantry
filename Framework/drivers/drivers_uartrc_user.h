#ifndef DRIVERS_UARTRC_USER_H
#define DRIVERS_UARTRC_USER_H

#include "utilities_iopool.h"

typedef struct{
	int16_t ch0;
	int16_t ch1;
	int16_t ch2;
	int16_t ch3;
	int8_t s1;
	int8_t s2;
}Remote_t;

typedef struct{
	int16_t x;
	int16_t y;
	int16_t z;
	uint8_t last_press_l;
	uint8_t last_press_r;
	uint8_t press_l;
	uint8_t press_r;
}Mouse_t;	

typedef struct{
	uint16_t v;
}Key_t;

typedef struct{
	Remote_t rc;
	Mouse_t mouse;
	Key_t key;
}RC_CtrlData_t; 


IOPoolDeclare(rcUartIOPool, struct{uint8_t ch[18];});

#endif
