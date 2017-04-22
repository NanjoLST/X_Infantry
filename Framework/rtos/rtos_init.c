#include "rtos_init.h"

//#include "utilities_debug.h"
//#include "utilities_iopool.h"
//#include "drivers_led_user.h"
//#include "drivers_canmotor_low.h"
//#include "drivers_mpu6050_low.h"
//#include "drivers_uartupper_low.h"
//#include "drivers_uartremotecontrol_low.h"

#include "drivers_imu_low.h"
#include "utilities_tim.h"

uint8_t isInited = 0;
void rtos_init(){
	//wait for devices
	for(int i=0; i < 3000; i++)
	{
		int a=42000; //at 168MHz 42000 is ok
		while(a--);
	}
	
	fw_userTimeEnable();
	MPU6500_Init();
	IST8310_Init();
//	rcInit();
//	ctrlUartInit();
//	motorInit();
//	mpu6050Init();
//	Init_Quaternion();
//	fw_printfln("init success");
}
