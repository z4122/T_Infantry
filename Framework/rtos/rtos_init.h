#ifndef RTOS_INIT_H
#define RTOS_INIT_H

#include "stdint.h"
extern uint8_t isInited;
void rtos_init(void);
void rtos_addSemaphores(void);
void rtos_addThreads(void);

#endif
