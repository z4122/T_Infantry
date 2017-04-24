#ifndef FRAMEWORK_FREERTOS_SEMAPHORE_H
#define FRAMEWORK_FREERTOS_SEMAPHORE_H

#include "cmsis_os.h"

extern osSemaphoreId motorCanTransmitSemaphoreHandle;
extern osSemaphoreId readMPU6050SemaphoreHandle;
extern osSemaphoreId refreshMPU6050SemaphoreHandle;
extern osSemaphoreId refreshIMUSemaphoreHandle;
extern xSemaphoreHandle xSemaphore_uart;
extern xSemaphoreHandle xSemaphore_rcuart;
extern xSemaphoreHandle motorCanReceiveSemaphore;
extern xSemaphoreHandle motorCanTransmitSemaphore;
extern EventGroupHandle_t xGMControl;
#define BIT_0 ( 1 << 0 )
#define BIT_3 ( 1 << 3 )
#define BIT_4 ( 1 << 4 )

#endif
