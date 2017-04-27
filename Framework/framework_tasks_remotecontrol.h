#ifndef FRAMEWORK_TASKS_REMOTECONTROL_H
#define FRAMEWORK_TASKS_REMOTECONTROL_H


#include "cmsis_os.h"
#include "framework_drivers_uartremotecontrol.h"

typedef enum
{
	AUTO = 0,
	MANUL = 1,
}Shoot_Mode_e;

typedef enum
{
	NORMAL = 0,
	EMERGENCY = 1,
}Emergency_Flag;

Shoot_Mode_e GetShootMode(void);
void SetShootMode(Shoot_Mode_e v);
Emergency_Flag GetEmergencyFlag(void);
void SetEmergencyFlag(Emergency_Flag v);

void RControlTask(void const * argument);
void RemoteTaskInit(void);
void RemoteDataProcess(uint8_t *pData);
void MouseKeyControlProcess(Mouse *mouse, Key *key);
void RemoteControlProcess(Remote *rc);


#endif
