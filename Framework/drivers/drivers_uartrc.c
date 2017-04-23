#include "drivers_uartrc_user.h"
#include "drivers_uartrc_low.h"
#include "drivers_led_user.h"
#include "usart.h"

#include "framework_peripheraldefine.h"

NaiveIOPoolDefine(rcUartIOPool, {0});

void rcInit(){
	//遥控器DMA接收开启(一次接收18个字节)
	if(HAL_UART_Receive_DMA(&RC_UART, IOPool_pGetWriteData(rcUartIOPool)->ch, 18) != HAL_OK){
			Error_Handler();
	} 
}

void rcUartRxCpltCallback(){
	//osStatus osMessagePut (osMessageQId queue_id, uint32_t info, uint32_t millisec);
	IOPool_getNextWrite(rcUartIOPool);
	HAL_UART_Receive_DMA(&RC_UART, IOPool_pGetWriteData(rcUartIOPool)->ch, 18);
}
