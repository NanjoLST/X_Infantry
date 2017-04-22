#include "utilities_debug.h"

//调试时开启
#ifdef _FW_DEBUG
	#include "usart.h"
	#include "framework_peripheraldefine.h"
	
	#ifdef __GNUC__
		#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
	#else
		#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
	#endif 
	PUTCHAR_PROTOTYPE
	{
		HAL_UART_Transmit(&DEBUG_UART , (uint8_t *)&ch, 1, 0xFFFF);
		return ch;
	}
#endif
