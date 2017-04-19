#ifndef FRAMEWORK_TASKS_MOTORCAN_H
#define FRAMEWORK_TASKS_MOTORCAN_H


#include "cmsis_os.h"

void printMotorTask(void const * argument);
void controlMotorTask(void const * argument);
void motorCanTransmitTask(void const * argument);

#endif
