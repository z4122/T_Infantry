#ifndef FRAMEWORK_FREERTOS_INIT_H
#define FRAMEWORK_FREERTOS_INIT_H

#include "stdint.h"
extern uint8_t isInited;
void fw_freertos_init(void);
void fw_freertos_addSemaphores(void);
void fw_freertos_addThreads(void);

#endif
