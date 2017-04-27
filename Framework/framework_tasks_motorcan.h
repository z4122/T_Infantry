#ifndef FRAMEWORK_TASKS_MOTORCAN_H
#define FRAMEWORK_TASKS_MOTORCAN_H


#include "cmsis_os.h"

#define STATE_SWITCH_DELAY_TICK 100000   //mode change delay count in ms

void canReceivelTask(void const * argument);
void GMControlTask(void const * argument);
void motorCanTransmitTask(void const * argument);

#endif
