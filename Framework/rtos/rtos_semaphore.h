#ifndef RTOS_SEMAPHORE_H
#define RTOS_SEMAPHORE_H

#include "cmsis_os.h"


extern osSemaphoreId CMGMCanHaveTransmitSemaphoreHandle;
extern osSemaphoreId ZGYROCanHaveTransmitSemaphoreHandle;

extern osSemaphoreId CMGMCanTransmitSemaphoreHandle;
extern osSemaphoreId ZGYROCanTransmitSemaphoreHandle;

extern osSemaphoreId motorCanReceiveSemaphoreHandle;

extern osSemaphoreId CMGMCanRefreshSemaphoreHandle;
extern osSemaphoreId ZGYROCanRefreshSemaphoreHandle;

extern osSemaphoreId imurefreshGimbalSemaphoreHandle;


//extern osSemaphoreId imuSpiTxRxCpltSemaphoreHandle;
extern osSemaphoreId refreshMPU6500SemaphoreHandle;
extern osSemaphoreId refreshIMUSemaphoreHandle;

extern xSemaphoreHandle xSemaphore_uart;
extern xSemaphoreHandle xSemaphore_rcuart;
extern xSemaphoreHandle motorCanTransmitSemaphore;

#endif
