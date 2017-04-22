#ifndef RTOS_TASK_H
#define RTOS_TASK_H

#include "cmsis_os.h"

extern osThreadId ledGTaskHandle;
extern osThreadId ledRTaskHandle;
extern osThreadId printRcTaskHandle;
//extern osThreadId printMotorTaskHandle;
extern osThreadId controlMotorTaskTaskHandle;
extern osThreadId motorCanTransmitTaskHandle;
extern osThreadId printMPU6050TaskHandle;
extern osThreadId readMPU6050TaskHandle;
extern osThreadId printCtrlUartTaskHandle;

extern osThreadId printIMUTaskHandle;

#endif
