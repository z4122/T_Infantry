#ifndef FRAMEWORK_REMOTECONTROL_H
#define FRAMEWORK_REMOTECONTROL_H

#define rcUart huart1
void rcInit(void);
void printRcTask(void const * argument);
void rcUartRxCpltCallback(void);

#endif
