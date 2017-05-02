#ifndef RTOS_SEMAPHORE_H
#define RTOS_SEMAPHORE_H

#include "cmsis_os.h"


extern osSemaphoreId CMGMCanHaveTransmitSemaphoreHandle;
extern osSemaphoreId AMCanHaveTransmitSemaphoreHandle;

extern osSemaphoreId CMGMCanTransmitSemaphoreHandle;
extern osSemaphoreId AMCanTransmitSemaphoreHandle;

extern osSemaphoreId motorCanReceiveSemaphoreHandle;

extern osSemaphoreId CMGMCanRefreshSemaphoreHandle;
extern osSemaphoreId AMCanRefreshSemaphoreHandle;

extern osSemaphoreId imurefreshGimbalSemaphoreHandle;


//extern osSemaphoreId imuSpiTxRxCpltSemaphoreHandle;
extern osSemaphoreId refreshMPU6500SemaphoreHandle;
extern osSemaphoreId refreshIMUSemaphoreHandle;


#endif
