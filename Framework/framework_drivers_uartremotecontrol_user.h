#ifndef FRAMEWORK_DRIVERS_UARTREMOTECONTROL_USER_H
#define FRAMEWORK_DRIVERS_UARTREMOTECONTROL_USER_H

#include "framework_utilities_iopool.h"

IOPoolDeclare(rcUartIOPool, struct{uint8_t ch[18];});

#endif
