#include "framework_tasks_ctrluart.h"
#include "framework_drivers_flash.h"
#include "framework_utilities_debug.h"
#include "framework_drivers_uart.h"
#include "framework_utilities_iopool.h"
#include "tim.h"

extern float yawAngleTarget, pitchAngleTarget;
extern xSemaphoreHandle xSemaphore_uart;
extern xdata_ctrlUart ctrlData; 
extern uint8_t ctrlUartFlag; 
extern uint16_t x;
/*妙算变量处理task*/
void printCtrlUartTask(void const * argument){
	while(1){
		xSemaphoreTake(xSemaphore_uart, osWaitForever);
		fw_printfln("CtrlUartTask processing");
		if(IOPool_hasNextRead(ctrlUartIOPool, 0)){
			IOPool_getNextRead(ctrlUartIOPool, 0);
			
			uint8_t *pData = IOPool_pGetReadData(ctrlUartIOPool, 0)->ch;
			x = (*(pData+1) <<8 )+ *(pData + 2);
				__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, x);
			ctrlData = xUartprocess( pData );
			if( ctrlData.Success == 1) {
				ctrlUartFlag = byte_EOF;
//				printf("dataprocess finished\r\n");
				vSendUart( ctrlData );
			} else {
				ctrlUartFlag = 0;
//				printf("dataprocess error\r\n");
				vSendUart( ctrlData );
				}

		}
	}
}
