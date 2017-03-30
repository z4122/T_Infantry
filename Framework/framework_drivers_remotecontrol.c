#include "framework_remotecontrol.h"

#include "framework_iopool.h"
#include "framework_led.h"
#include "usart.h"

/*****Begin define ioPool*****/
#define IOPoolName0 rcUartIOPool 
#define DataType struct{uint8_t ch[18];}
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

void rcInit(){
	//遥控器DMA接收开启(一次接收18个字节)
	if(HAL_UART_Receive_DMA(&rcUart, IOPool_pGetWriteData(rcUartIOPool)->ch, 18) != HAL_OK){
			Error_Handler();
	} 
}

void rcUartRxCpltCallback(){
	//osStatus osMessagePut (osMessageQId queue_id, uint32_t info, uint32_t millisec);
	IOPool_getNextWrite(rcUartIOPool);
	HAL_UART_Receive_DMA(&rcUart, IOPool_pGetWriteData(rcUartIOPool)->ch, 18);
}

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
//{
//	if(UartHandle == &rcUart){
//		rcUartRxCpltCallback();
//	}
//}   

typedef struct{
	int16_t ch0;
	int16_t ch1;
	int16_t ch2;
	int16_t ch3;
	int8_t s1;
	int8_t s2;
}Remote;
typedef struct{
	int16_t x;
	int16_t y;
	int16_t z;
	uint8_t last_press_l;
	uint8_t last_press_r;
	uint8_t press_l;
	uint8_t press_r;
}Mouse;	
typedef struct{
	uint16_t v;
}Key;

struct{
	Remote rc;
	Mouse mouse;
	Key key;
}RC_CtrlData; 

void RemoteDataPrcess(uint8_t *pData)
{
    if(pData == NULL)
    {
        return;
    }
    RC_CtrlData.rc.ch0 = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF; 
    RC_CtrlData.rc.ch1 = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5)) & 0x07FF;
    RC_CtrlData.rc.ch2 = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) |
                         ((int16_t)pData[4] << 10)) & 0x07FF;
    RC_CtrlData.rc.ch3 = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5]<<7)) & 0x07FF;
    
    RC_CtrlData.rc.s1 = ((pData[5] >> 4) & 0x000C) >> 2;
    RC_CtrlData.rc.s2 = ((pData[5] >> 4) & 0x0003);

    RC_CtrlData.mouse.x = ((int16_t)pData[6]) | ((int16_t)pData[7] << 8);
    RC_CtrlData.mouse.y = ((int16_t)pData[8]) | ((int16_t)pData[9] << 8);
    RC_CtrlData.mouse.z = ((int16_t)pData[10]) | ((int16_t)pData[11] << 8);    

    RC_CtrlData.mouse.press_l = pData[12];
    RC_CtrlData.mouse.press_r = pData[13];
 
    RC_CtrlData.key.v = ((int16_t)pData[14]);// | ((int16_t)pData[15] << 8);
}

void printRcTask(void const * argument){
	uint8_t data[18];
	while(1){
		if(IOPool_hasNextRead(rcUartIOPool, 0)){
			IOPool_getNextRead(rcUartIOPool, 0);
			
			uint8_t *pData = IOPool_pGetReadData(rcUartIOPool, 0)->ch;
			for(uint8_t i = 0; i != 18; ++i){
				data[i] = pData[i];
			}
		
			RemoteDataPrcess(data);
			if(RC_CtrlData.rc.s1 == 1){
				ledGStatus = on;
			}else if(RC_CtrlData.rc.s1 == 2){
				ledGStatus = blink;
			}else{
				ledGStatus = off;
			}
			if(RC_CtrlData.rc.s2 == 1){
				ledRStatus = on;
			}else if(RC_CtrlData.rc.s2 == 2){
				ledRStatus = blink;
			}else{
				ledRStatus = off;
			}
			
			
//			printf("ch0 = %d | ", RC_CtrlData.rc.ch0);
//			printf("ch1 = %d | ", RC_CtrlData.rc.ch1);
//			printf("ch2 = %d | ", RC_CtrlData.rc.ch2);
//			printf("ch3 = %d \r\n", RC_CtrlData.rc.ch3);
//			
//			printf("s1 = %d | ", RC_CtrlData.rc.s1);
//			printf("s2 = %d \r\n", RC_CtrlData.rc.s2);
//			
//			printf("x = %d | ", RC_CtrlData.mouse.x);
//			printf("y = %d | ", RC_CtrlData.mouse.y);
//			printf("z = %d | ", RC_CtrlData.mouse.z);
//			printf("l = %d | ", RC_CtrlData.mouse.press_l);
//			printf("r = %d \r\n", RC_CtrlData.mouse.press_r);
//			
//			printf("key = %d \r\n", RC_CtrlData.key.v);
//			printf("===========\r\n");

		}
	}
}
