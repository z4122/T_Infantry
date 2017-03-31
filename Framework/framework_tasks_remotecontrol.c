#include "framework_tasks_remotecontrol.h"
#include "framework_drivers_uartremotecontrol_iopool.h"
#include "framework_drivers_led.h"
#include "stdint.h"
#include "stddef.h"

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
