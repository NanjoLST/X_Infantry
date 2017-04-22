#include "drivers_led_user.h"
#include "drivers_led_low.h"

#include "framework_peripheraldefine.h"
#include "cmsis_os.h"
#include "gpio.h"

#define ledGreenOn() HAL_GPIO_WritePin(GREEN_PIN, GREEN_GPIO_PORT, GPIO_PIN_RESET)
#define ledGreenOff() HAL_GPIO_WritePin(GREEN_PIN, GREEN_GPIO_PORT, GPIO_PIN_SET)
#define ledRedOn() HAL_GPIO_WritePin(RED_PIN, RED_GPIO_PORT, GPIO_PIN_RESET)
#define ledRedOff() HAL_GPIO_WritePin(RED_PIN, RED_GPIO_PORT, GPIO_PIN_SET)

LedStatus_t ledGreenStatus = blink, ledRedStatus = blink;

void ledGreenTask(void const * argument){
	while(1){
		if(ledGreenStatus == on){
			ledGreenOn();
		}else if(ledGreenStatus == off){
			ledGreenOff();
		}else if(ledGreenStatus == blink){
			ledGreenOn();
			osDelay(777);
			ledGreenOff();
			osDelay(222);
		}
	}
}

void ledRedTask(void const * argument){
	while(1){
		if(ledRedStatus == on){
			ledRedOn();
		}else if(ledRedStatus == off){
			ledRedOff();
		}else if(ledRedStatus == blink){
			ledRedOn();
			osDelay(66);
			ledRedOff();
			osDelay(88);
		}
	}
}
