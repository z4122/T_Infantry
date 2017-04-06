#include "framework_freertos_semaphore.h"
#include "framework_freertos_init.h"

osSemaphoreId motorCanTransmitSemaphoreHandle;
osSemaphoreId motorCanReceiveSemaphoreHandle;
osSemaphoreId refreshGimbalSemaphoreHandle;
osSemaphoreId motorCanHaveTransmitSemaphoreHandle;

osSemaphoreId readMPU6050SemaphoreHandle;
osSemaphoreId refreshMPU6050SemaphoreHandle;
osSemaphoreId refreshIMUSemaphoreHandle;

void fw_freertos_addSemaphores(){
	osSemaphoreDef(motorCanTransmitSemaphore);
	motorCanTransmitSemaphoreHandle = osSemaphoreCreate(osSemaphore(motorCanTransmitSemaphore), 1);
	osSemaphoreDef(motorCanReceiveSemaphore);
	motorCanReceiveSemaphoreHandle = osSemaphoreCreate(osSemaphore(motorCanReceiveSemaphore), 1);
	osSemaphoreDef(refreshGimbalSemaphore);
	refreshGimbalSemaphoreHandle = osSemaphoreCreate(osSemaphore(refreshGimbalSemaphore), 1);
	osSemaphoreDef(motorCanHaveTransmitSemaphore);
	motorCanHaveTransmitSemaphoreHandle = osSemaphoreCreate(osSemaphore(motorCanHaveTransmitSemaphore), 1);
	
	osSemaphoreDef(readMPU6050Semaphore);
	readMPU6050SemaphoreHandle = osSemaphoreCreate(osSemaphore(readMPU6050Semaphore), 1);
	osSemaphoreDef(refreshMPU6050Semaphore);
	refreshMPU6050SemaphoreHandle = osSemaphoreCreate(osSemaphore(refreshMPU6050Semaphore), 1);
	osSemaphoreDef(refreshIMUSemaphore);
	refreshIMUSemaphoreHandle = osSemaphoreCreate(osSemaphore(refreshIMUSemaphore), 1);
}
