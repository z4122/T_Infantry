#ifndef FRAMEWORK_FREERTOS_TASK_H
#define FRAMEWORK_FREERTOS_TASK_H

#include "cmsis_os.h"

extern osThreadId ledGTaskHandle;
extern osThreadId ledRTaskHandle;
extern osThreadId RControlTaskHandle;
extern osThreadId canReceiveTaskHandle;
extern osThreadId GMControlTaskHandle;
extern osThreadId CMControlTaskHandle;
extern osThreadId motorCanTransmitTaskHandle;
extern osThreadId printMPU6050TaskHandle;
extern osThreadId readMPU6050TaskHandle;
extern osThreadId CtrlUartTaskHandle;

#endif
