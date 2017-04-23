#ifndef FRAMEWORK_TASKS_REMOTECONTROL_H
#define FRAMEWORK_TASKS_REMOTECONTROL_H


#include "cmsis_os.h"
#include "framework_drivers_uartremotecontrol.h"

typedef enum
{
	AUTO = 0,
	MANUL = 1,
}Shoot_Mode_e;
Shoot_Mode_e GetShootMode(void);
void SetShootMode(Shoot_Mode_e v);

void RControlTask(void const * argument);
void RemoteTaskInit(void);
void RemoteDataProcess(uint8_t *pData);
void MouseKeyControlProcess(Mouse *mouse, Key *key);
void RemoteControlProcess(Remote *rc);


#endif
