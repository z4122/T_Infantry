#ifndef RTOS_TASK_H
#define RTOS_TASK_H

#include "cmsis_os.h"

extern osThreadId ledGreenTaskHandle;
extern osThreadId ledRedTaskHandle;
extern osThreadId buzzerTaskHandle;
//IMU
extern osThreadId printIMUTaskHandle;
//UART
extern osThreadId printRcTaskHandle;
extern osThreadId getCtrlUartTaskHandle;
//Motor
extern osThreadId CMGMControlTaskHandle;
extern osThreadId AMControlTaskHandle;

extern osThreadId CMGMCanTransmitTaskHandle;
extern osThreadId AMCanTransmitTaskHandle;

extern osThreadId sonarTaskHandle;

//extern osThreadId testFlashTaskHandle;

#endif
