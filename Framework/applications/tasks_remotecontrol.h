#ifndef FRAMEWORK_TASKS_REMOTECONTROL_H
#define FRAMEWORK_TASKS_REMOTECONTROL_H


#include "cmsis_os.h"
#include "drivers_uartrc_low.h"

void RControlTask(void const * argument);
void RemoteTaskInit(void);
void RemoteDataProcess(uint8_t *pData);
void MouseKeyControlProcess(Mouse *mouse, Key *key);
void RemoteControlProcess(Remote *rc);


#endif
