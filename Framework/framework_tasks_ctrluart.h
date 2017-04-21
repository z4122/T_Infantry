#ifndef FRAMEWORK_TASKS_CTRLUART_H
#define FRAMEWORK_TASKS_CTRLUART_H


#include "cmsis_os.h"
#include "framework_utilities_iopool.h"

IOPoolDeclare(upperGimbalIOPool, struct{float yawAdd; float pitchAdd;});
void printCtrlUartTask(void const * argument);

#endif
