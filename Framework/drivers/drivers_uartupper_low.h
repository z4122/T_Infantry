#ifndef DRIVERS_UARTUPPER_LOW_H
#define DRIVERS_UARTUPPER_LOW_H

#include "drivers_uartupper_user.h"

void ctrlUartRxCpltCallback(void);

void ctrlUartInit(void);
void vSendUart(xdata_ctrlUart data);
xdata_ctrlUart xUartprocess(uint8_t *pData);

#endif
