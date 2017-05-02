#ifndef DRIVERS_UARTUPPER_USER_H
#define DRIVERS_UARTUPPER_USER_H

#include "utilities_iopool.h"

IOPoolDeclare(ctrlUartIOPool, struct{uint8_t ch[10];});

#endif
