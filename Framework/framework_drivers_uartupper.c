#include "framework_drivers_uartupper_low.h"
#include "framework_drivers_uartupper_user.h"

#include "usart.h"

/*****Begin define ioPool*****/
#define DataPoolInit {0}
#define ReadPoolSize 1
#define ReadPoolMap {0}
#define GetIdFunc 0 
#define ReadPoolInit {0, Empty, 1}

IOPoolDefine(ctrlUartIOPool, DataPoolInit, ReadPoolSize, ReadPoolMap, GetIdFunc, ReadPoolInit);

#undef DataPoolInit 
#undef ReadPoolSize 
#undef ReadPoolMap
#undef GetIdFunc
#undef ReadPoolInit
/*****End define ioPool*****/

void ctrlUartRxCpltCallback(){
	//osStatus osMessagePut (osMessageQId queue_id, uint32_t info, uint32_t millisec);
	IOPool_getNextWrite(ctrlUartIOPool);
	HAL_UART_Receive_DMA(&ctrlUart, IOPool_pGetWriteData(ctrlUartIOPool)->ch, 10);
}

void ctrlUartInit(){
	//crtl DMA接收开启(一次接收10个字节)
	if(HAL_UART_Receive_DMA(&ctrlUart, IOPool_pGetWriteData(ctrlUartIOPool)->ch, 10) != HAL_OK){
			Error_Handler();
	} 
}
