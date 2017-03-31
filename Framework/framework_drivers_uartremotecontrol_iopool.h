#ifndef FRAMEWORK_DRIVERS_UARTREMOTECONTROL_IOPOOL_H
#define FRAMEWORK_DRIVERS_UARTREMOTECONTROL_IOPOOL_H

#include "framework_utilities_iopool.h"

IOPoolDeclare(rcUartIOPool, struct{uint8_t ch[18];});

#endif
