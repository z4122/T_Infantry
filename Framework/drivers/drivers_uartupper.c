#include "drivers_uartupper_low.h"
#include "drivers_uartupper_user.h"

#include "peripheral_define.h"

#include "usart.h"

NaiveIOPoolDefine(ctrlUartIOPool, {0});

void ctrlUartRxCpltCallback(){
	//osStatus osMessagePut (osMessageQId queue_id, uint32_t info, uint32_t millisec);
	IOPool_getNextWrite(ctrlUartIOPool);
	HAL_UART_Receive_DMA(&CTRL_UART, IOPool_pGetWriteData(ctrlUartIOPool)->ch, 10);
}

void ctrlUartInit(){
	//crtl DMA接收开启(一次接收10个字节)
	if(HAL_UART_Receive_DMA(&CTRL_UART, IOPool_pGetWriteData(ctrlUartIOPool)->ch, 10) != HAL_OK){
			Error_Handler();
	} 
}
