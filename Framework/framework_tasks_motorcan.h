#ifndef FRAMEWORK_TASKS_MOTORCAN_H
#define FRAMEWORK_TASKS_MOTORCAN_H


#include "cmsis_os.h"

void canReceiveTask(void const * argument);
void GMControlTask(void const * argument);
void motorCanTransmitTask(void const * argument);

#endif
