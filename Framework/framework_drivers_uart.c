#include "framework_uart.h"

#include "framework_iopool.h"
#include "framework_remotecontrol.h"
#include "framework_debug.h"
#include "usart.h"

/*****Begin define ioPool*****/
#define IOPoolName0 ctrlUartIOPool 
#define DataType struct{uint8_t ch[10];}
#define DataPoolInit {0}
#define ReadPoolSize 1
#define ReadPoolMap {0}
#define GetIdFunc 0 
#define ReadPoolInit {0, Empty, 1}

DefineIOPool(IOPoolName0, DataType, DataPoolInit, ReadPoolSize, ReadPoolMap, GetIdFunc, ReadPoolInit);
	
#undef DataType
#undef DataPoolInit 
#undef ReadPoolSize 
#undef ReadPoolMap
#undef GetIdFunc
#undef ReadPoolInit
/*****End define ioPool*****/

void ctrlUartInit(){
	//crtl DMA接收开启(一次接收10个字节)
	if(HAL_UART_Receive_DMA(&ctrlUart, IOPool_pGetWriteData(ctrlUartIOPool)->ch, 10) != HAL_OK){
			Error_Handler();
	} 
}

void ctrlUartRxCpltCallback(){
	//osStatus osMessagePut (osMessageQId queue_id, uint32_t info, uint32_t millisec);
	IOPool_getNextWrite(ctrlUartIOPool);
	HAL_UART_Receive_DMA(&ctrlUart, IOPool_pGetWriteData(ctrlUartIOPool)->ch, 10);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	if(UartHandle == &rcUart){
		rcUartRxCpltCallback();
	}else if(UartHandle == &ctrlUart){
		ctrlUartRxCpltCallback();
	}
}   

#include "framework_flash.h"
extern float yawAngleTarget, pitchAngleTarget;
void printCtrlUartTask(void const * argument){
	uint8_t data[10];
	while(1){
		if(IOPool_hasNextRead(ctrlUartIOPool, 0)){
			IOPool_getNextRead(ctrlUartIOPool, 0);
			
			uint8_t *pData = IOPool_pGetReadData(ctrlUartIOPool, 0)->ch;
			for(uint8_t i = 0; i != 10; ++i){
				data[i] = pData[i];
			}			
			if(data[0] == 'L'){
				yawAngleTarget += 15.0f;
			}else if(data[0] == 'R'){
				yawAngleTarget -= 15.0f;
			}else if(data[0] == 'U'){
				pitchAngleTarget += 8.0f;
			}else if(data[0] == 'D'){
				pitchAngleTarget -= 8.0f;
			}else if(data[0] == 'T'){
				fw_printfln("received T: %d", data[1]);
			}
//			else if(data[0] == 'F'){
//				uint32_t temp;
//				if(data[1] == '1'){
//					temp = 1;
//				}else if(data[1] == '0'){
//					temp = 0;
//				}else{
//					temp = 99;
//				}
//				STMFLASH_Write(PARAM_SAVED_START_ADDRESS, &temp, 1);
//				fw_printfln("F: %d", temp);
//			}else if(data[0] == 'X'){
//				uint32_t temp;
//				STMFLASH_Read(PARAM_SAVED_START_ADDRESS, &temp, 1);
//				fw_printfln("Read: %d", temp);
//			}				
			
//			fw_printf("d:");
//			fw_printf("%c|", data[0]);
//			fw_printf("%c|", data[1]);
//			fw_printf("%c|", data[2]);
//			fw_printf("%c|", data[3]);
//			fw_printf("%c|", data[4]);
//			fw_printf("%c|", data[5]);
//			fw_printf("%c|", data[6]);
//			fw_printf("%c|", data[7]);
//			fw_printf("%c|", data[8]);
//			fw_printf("%c\r\n", data[9]);
//			fw_printf("===========\r\n");

		}
	}
}
