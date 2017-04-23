#ifndef DRIVERS_UARTRC_USER_H
#define DRIVERS_UARTRC_USER_H

#include "utilities_iopool.h"

IOPoolDeclare(rcUartIOPool, struct{uint8_t ch[18];});

#endif
