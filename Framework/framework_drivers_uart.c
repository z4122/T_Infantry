#include "framework_drivers_uart.h"
#include "framework_drivers_uartremotecontrol_low.h"
#include "framework_drivers_uartupper_low.h"

#include "usart.h"

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	if(UartHandle == &rcUart){
		rcUartRxCpltCallback();
	}else if(UartHandle == &ctrlUart){
		ctrlUartRxCpltCallback();
	}
}   
