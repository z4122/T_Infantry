#include "drivers_uart.h"
#include "drivers_uartremotecontrol_low.h"
#include "drivers_uartupper_low.h"

#include "usart.h"

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	if(UartHandle == &rcUart){
		rcUartRxCpltCallback();
	}else if(UartHandle == &ctrlUart){
		ctrlUartRxCpltCallback();
	}
}   
