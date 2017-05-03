#include "rtos_semaphore.h"
#include "rtos_init.h"

osSemaphoreId CMGMCanHaveTransmitSemaphoreHandle;
osSemaphoreId ZGYROCanHaveTransmitSemaphoreHandle;

osSemaphoreId CMGMCanTransmitSemaphoreHandle;
osSemaphoreId ZGYROCanTransmitSemaphoreHandle;

osSemaphoreId motorCanReceiveSemaphoreHandle;

osSemaphoreId CMGMCanRefreshSemaphoreHandle;
osSemaphoreId ZGYROCanRefreshSemaphoreHandle;

osSemaphoreId imurefreshGimbalSemaphoreHandle;

//osSemaphoreId imuSpiTxRxCpltSemaphoreHandle;
osSemaphoreId refreshMPU6500SemaphoreHandle;
osSemaphoreId refreshIMUSemaphoreHandle;

xSemaphoreHandle xSemaphore_uart;
xSemaphoreHandle xSemaphore_rcuart;
xSemaphoreHandle motorCanTransmitSemaphore;
void rtos_addSemaphores(){
	osSemaphoreDef(CMGMCanTransmitSemaphore);
	CMGMCanTransmitSemaphoreHandle = osSemaphoreCreate(osSemaphore(CMGMCanTransmitSemaphore), 1);
	osSemaphoreDef(ZGYROCanTransmitSemaphore);
	ZGYROCanTransmitSemaphoreHandle = osSemaphoreCreate(osSemaphore(ZGYROCanTransmitSemaphore), 1);
	
	osSemaphoreDef(motorCanReceiveSemaphore);
	motorCanReceiveSemaphoreHandle = osSemaphoreCreate(osSemaphore(motorCanReceiveSemaphore), 1);
	
	osSemaphoreDef(CMGMCanHaveTransmitSemaphore);
	CMGMCanHaveTransmitSemaphoreHandle = osSemaphoreCreate(osSemaphore(CMGMCanHaveTransmitSemaphore), 1);
	osSemaphoreDef(ZGYROCanHaveTransmitSemaphore);
	ZGYROCanHaveTransmitSemaphoreHandle = osSemaphoreCreate(osSemaphore(ZGYROCanHaveTransmitSemaphore), 1);

	osSemaphoreDef(CMGMCanRefreshSemaphore);
	CMGMCanRefreshSemaphoreHandle = osSemaphoreCreate(osSemaphore(CMGMCanRefreshSemaphore), 1);
	osSemaphoreDef(ZGYROCanRefreshSemaphore);
	ZGYROCanRefreshSemaphoreHandle = osSemaphoreCreate(osSemaphore(ZGYROCanRefreshSemaphore), 1);
	
	osSemaphoreDef(imurefreshGimbalSemaphore);
	imurefreshGimbalSemaphoreHandle = osSemaphoreCreate(osSemaphore(imurefreshGimbalSemaphore), 1);
	
//	osSemaphoreDef(imuSpiTxRxCpltSemaphore);
//	imuSpiTxRxCpltSemaphoreHandle = osSemaphoreCreate(osSemaphore(imuSpiTxRxCpltSemaphore), 1);
	osSemaphoreDef(refreshMPU6500Semaphore);
	refreshMPU6500SemaphoreHandle = osSemaphoreCreate(osSemaphore(refreshMPU6500Semaphore), 1);
	osSemaphoreDef(refreshIMUSemaphore);
	refreshIMUSemaphoreHandle = osSemaphoreCreate(osSemaphore(refreshIMUSemaphore), 1);
	
	vSemaphoreCreateBinary(xSemaphore_uart);
	vSemaphoreCreateBinary(xSemaphore_rcuart);
	motorCanTransmitSemaphore = xSemaphoreCreateCounting(10,0);
}
