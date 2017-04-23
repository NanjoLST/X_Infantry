#include "drivers_uart.h"
#include "drivers_uartrc_low.h"
#include "drivers_uartupper_low.h"

#include "framework_peripheraldefine.h"

#include "usart.h"

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	if(UartHandle == &RC_UART){
		rcUartRxCpltCallback();
	}else if(UartHandle == &CTRL_UART){
		ctrlUartRxCpltCallback();
	}
}   
