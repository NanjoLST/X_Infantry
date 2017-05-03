#ifndef TASKS_UPPER_H
#define TASKS_UPPER_H

#include "utilities_iopool.h"

IOPoolDeclare(upperIOPool, struct{float yawAdd; float pitchAdd;});

void getCtrlUartTask(void const * argument);

#endif
