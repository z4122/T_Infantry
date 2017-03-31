#ifndef FRAMEWORK_DRIVERS_UARTREMOTECONTROL_LOW_H
#define FRAMEWORK_DRIVERS_UARTREMOTECONTROL_LOW_H

#define rcUart huart1
void rcUartRxCpltCallback(void);

void rcInit(void);

#endif
