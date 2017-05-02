#include "rtos_semaphore.h"
#include "rtos_init.h"

osSemaphoreId CMGMCanHaveTransmitSemaphoreHandle;
osSemaphoreId AMCanHaveTransmitSemaphoreHandle;

osSemaphoreId CMGMCanTransmitSemaphoreHandle;
osSemaphoreId AMCanTransmitSemaphoreHandle;

osSemaphoreId motorCanReceiveSemaphoreHandle;

osSemaphoreId CMGMCanRefreshSemaphoreHandle;
osSemaphoreId AMCanRefreshSemaphoreHandle;

osSemaphoreId imurefreshGimbalSemaphoreHandle;

//osSemaphoreId imuSpiTxRxCpltSemaphoreHandle;
osSemaphoreId refreshMPU6500SemaphoreHandle;
osSemaphoreId refreshIMUSemaphoreHandle;

void rtos_addSemaphores(){
	osSemaphoreDef(CMGMCanTransmitSemaphore);
	CMGMCanTransmitSemaphoreHandle = osSemaphoreCreate(osSemaphore(CMGMCanTransmitSemaphore), 1);
	osSemaphoreDef(AMCanTransmitSemaphore);
	AMCanTransmitSemaphoreHandle = osSemaphoreCreate(osSemaphore(AMCanTransmitSemaphore), 1);
	
	osSemaphoreDef(motorCanReceiveSemaphore);
	motorCanReceiveSemaphoreHandle = osSemaphoreCreate(osSemaphore(motorCanReceiveSemaphore), 1);
	
	osSemaphoreDef(CMGMCanHaveTransmitSemaphore);
	CMGMCanHaveTransmitSemaphoreHandle = osSemaphoreCreate(osSemaphore(CMGMCanHaveTransmitSemaphore), 1);
	osSemaphoreDef(AMCanHaveTransmitSemaphore);
	AMCanHaveTransmitSemaphoreHandle = osSemaphoreCreate(osSemaphore(AMCanHaveTransmitSemaphore), 1);

	osSemaphoreDef(CMGMCanRefreshSemaphore);
	CMGMCanRefreshSemaphoreHandle = osSemaphoreCreate(osSemaphore(CMGMCanRefreshSemaphore), 1);
	osSemaphoreDef(AMCanRefreshSemaphore);
	AMCanRefreshSemaphoreHandle = osSemaphoreCreate(osSemaphore(AMCanRefreshSemaphore), 1);
	
	osSemaphoreDef(imurefreshGimbalSemaphore);
	imurefreshGimbalSemaphoreHandle = osSemaphoreCreate(osSemaphore(imurefreshGimbalSemaphore), 1);
	
//	osSemaphoreDef(imuSpiTxRxCpltSemaphore);
//	imuSpiTxRxCpltSemaphoreHandle = osSemaphoreCreate(osSemaphore(imuSpiTxRxCpltSemaphore), 1);
	osSemaphoreDef(refreshMPU6500Semaphore);
	refreshMPU6500SemaphoreHandle = osSemaphoreCreate(osSemaphore(refreshMPU6500Semaphore), 1);
	osSemaphoreDef(refreshIMUSemaphore);
	refreshIMUSemaphoreHandle = osSemaphoreCreate(osSemaphore(refreshIMUSemaphore), 1);
}
