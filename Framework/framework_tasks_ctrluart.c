#include "framework_tasks_ctrluart.h"
#include "framework_drivers_flash.h"
#include "framework_utilities_debug.h"
#include "framework_drivers_uart.h"
#include "framework_utilities_iopool.h"
#include "tim.h"

/*****Begin define ioPool*****/
#define DataPoolInit {0}
#define ReadPoolSize 1
#define ReadPoolMap {0}
#define GetIdFunc 0 
#define ReadPoolInit {0, Empty, 1}

IOPoolDefine(upperGimbalIOPool, DataPoolInit, ReadPoolSize, ReadPoolMap, GetIdFunc, ReadPoolInit);
	
#undef DataPoolInit 
#undef ReadPoolSize 
#undef ReadPoolMap
#undef GetIdFunc
#undef ReadPoolInit
/*****End define ioPool*****/

extern float yawAngleTarget, pitchAngleTarget;
extern xSemaphoreHandle xSemaphore_uart;
extern xdata_ctrlUart ctrlData; 
extern uint8_t ctrlUartFlag; 
extern uint16_t x;
/*妙算变量处理task*/
void CtrlUartTask(void const * argument){
	while(1){
		xSemaphoreTake(xSemaphore_uart, osWaitForever);
		fw_printfln("CtrlUartTask processing");
		if(IOPool_hasNextRead(ctrlUartIOPool, 0)){
			IOPool_getNextRead(ctrlUartIOPool, 0);
			
			uint8_t *pData = IOPool_pGetReadData(ctrlUartIOPool, 0)->ch;
			ctrlData = xUartprocess( pData );
			if( ctrlData.Success == 1) {
				ctrlUartFlag = byte_EOF;
				printf("dataprocess finished\r\n");
				vSendUart( ctrlData );
				} else {
				ctrlUartFlag = 0;
				printf("dataprocess error\r\n");
			vSendUart( ctrlData );
				}
				fw_printfln("%x",ctrlData.dev_yaw);
		IOPool_pGetWriteData(upperGimbalIOPool)->yawAdd = ((float)ctrlData.dev_yaw - 9000)/100;
	fw_printfln("%f",IOPool_pGetWriteData(upperGimbalIOPool)->yawAdd);
		IOPool_pGetWriteData(upperGimbalIOPool)->pitchAdd = ((float)ctrlData.dev_pitch - 5000)/100;
		IOPool_getNextWrite(upperGimbalIOPool);

		}
	}

}
