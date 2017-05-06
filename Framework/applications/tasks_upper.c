#include "tasks_upper.h"
#include "drivers_uartupper_user.h"
#include "drivers_uartupper_low.h"
#include "rtos_semaphore.h"
#include "drivers_flash.h"
#include "stdint.h"
#include "utilities_debug.h"
#include "cmsis_os.h"

NaiveIOPoolDefine(upperIOPool, {0});

extern uint16_t yawAngle, pitchAngle;
int forPidDebug = 0;

extern float yawAngleTarget, pitchAngleTarget;
extern xSemaphoreHandle xSemaphore_uart;
extern xdata_ctrlUart ctrlData; 
extern uint16_t x;
uint8_t CReceive = 0;
float yawAdd = 0;
float pitchAdd = 0;

//extern float yawAngleTarget, pitchAngleTarget;
void getCtrlUartTask(void const * argument){
	while(1){
		xSemaphoreTake(xSemaphore_uart, osWaitForever);
//		fw_printfln("CtrlUartTask processing");
		uint8_t *pData = IOPool_pGetReadData(ctrlUartIOPool, 0)->ch;
		ctrlData = xUartprocess( pData );
		if( ctrlData.Success == 1) {
			yawAdd = ((float)ctrlData.dev_yaw - 9000)/100;
			pitchAdd = ((float)ctrlData.dev_pitch - 5000)/100;
//			yawAdd = 0;
//			fw_printfln("yawAdd:%f",yawAdd);
//			fw_printfln("pitchAdd:%f",pitchAdd);
			IOPool_pGetWriteData(upperIOPool)->yawAdd = yawAdd;
			IOPool_pGetWriteData(upperIOPool)->pitchAdd = pitchAdd;
			IOPool_getNextWrite(upperIOPool);
			CReceive = 2;
		} 
		else {
			yawAdd = 0;
			pitchAdd = 0;
			CReceive = 0;
		}
	 }
}

	
