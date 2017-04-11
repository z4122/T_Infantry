#ifndef FRAMEWORK_TASKS_UPPER_H
#define FRAMEWORK_TASKS_UPPER_H

#include "utilities_iopool.h"

IOPoolDeclare(upperGimbalIOPool, struct{float yawAdd; float pitchAdd;});

void printCtrlUartTask(void const * argument);

#endif
