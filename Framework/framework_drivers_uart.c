#include "framework_drivers_uart.h"
#include "framework_freertos_semaphore.h"
#include "FreeRTOS.h"
#include "framework_utilities_iopool.h"
#include "framework_drivers_uartremotecontrol.h"
#include "framework_utilities_debug.h"
#include "usart.h"
#include "semphr.h"
#include "cmsis_os.h"

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

#define ctrlUart huart3
uint8_t ctrlUartFlag = byte_EOF; //帧错误后，等待终止字节
xdata_ctrlUart ctrlData; //妙算接收变量
/*发送协议*/
//插入数组
void vInsert( uint8_t a[ ], uint8_t i, uint8_t n, uint8_t number){
    for (int j=n;j>i;j--){
        a[j]=a[j-1];
        }
        a[i]=number;
    if (i==n)
        a[i]=number;
}
//检查转义符
void vCheck( uint8_t a[] ){
	for(uint8_t i = 1; i <= size_frame - 3; i++)
	{switch ( a[i] ){
		case byte_SOF    : vInsert( a, i, size_frame, byte_ESCAPE);
		                 a[i+1] = 0X00; 
		//fw_printfln("inchange");
		break;
		case byte_EOF    : vInsert( a, i, size_frame, byte_ESCAPE);
		                 a[i+1] = 0X01; //fw_printfln("inchange2");
		break;		
		case byte_ESCAPE : vInsert( a, i, size_frame, byte_ESCAPE);
		                 a[i+1] = 0X02; //fw_printfln("inchange3");
		break;
	}}
}
//发送
void vSendUart(xdata_ctrlUart data){
	uint8_t tempdata[size_frame] = {0};
	tempdata[0] = byte_SOF;
	tempdata[1] = data.dev_yaw >> 8;
	tempdata[2] = data.dev_yaw & 0x00ff;
  tempdata[3] = data.dev_pitch >> 8;
	tempdata[4] = data.dev_pitch & 0x00ff;	
	tempdata[5] = data.target_vl >> 8;
	tempdata[6] = data.target_vl & 0x00ff;
	tempdata[7] = data.target_dis >> 8;
	tempdata[8] = data.target_dis & 0x00ff;
	tempdata[9] = data.DLC;
	tempdata[10] = byte_EOF;
  vCheck( tempdata );
	for( uint8_t i = 0; i <= size_frame - 1; i++){
		fw_printf("%x",tempdata[i]);
	}
	printf("\r\n");
}
/*接收协议*/
//删除转义符
void vDeleteEscape(uint8_t *pData, uint8_t a){
   while ( a <= size_frame - 2 ){
	 *(pData + a) = *(pData + a + 1);
		 a ++;
	 }
}
//检查转义符
void vCheckEscape(uint8_t *pData){
	uint8_t a = 1;
	while( *pData != byte_EOF && a <= size_frame -1){
  if( *(pData + a) == byte_ESCAPE ) {
	//	fw_printfln("in escapecheck");
		switch ( *(pData + a + 1) ){
			case 0x00 : *(pData + a + 1) = byte_SOF; break; 
			case 0x01 : *(pData + a + 1) = byte_EOF; break;
			case 0x02 : *(pData + a + 1) = byte_ESCAPE; break;
		}
		vDeleteEscape( pData, a );
	}
	a++;
  }
}
//接收数据处理 返回 妙算接收变量
xdata_ctrlUart xUartprocess(uint8_t *pData){
	xdata_ctrlUart To_return;
	To_return.Success = 0;
	uint8_t a = 0; //字节索引
		if ( *pData != byte_SOF ) {
      To_return.Success = 0;
			return To_return;
		}
		vCheckEscape( pData );
		for(; a <= size_frame - 1; a++){
			if(*(pData + a) == byte_EOF) break;
		}
		if( *(pData + a ) != byte_EOF || *(pData + a - 1) != a - 2) {
    To_return.Success = 0; 
    return To_return;
		}
		else To_return.Success = 1;
		To_return.dev_yaw    = (*(pData + 1) << 8) + *(pData + 2);
		To_return.dev_pitch  = (*(pData + 3) << 8) + *(pData + 4);
		To_return.target_vl  = (*(pData + 5) << 8) + *(pData + 6);
		To_return.target_dis = (*(pData + 7) << 8) + *(pData + 8);
		To_return.DLC = *(pData + a - 1);
		return To_return;
	}


void ctrlUartInit(){
	//crtl DMA接收开启(一次接收 size_frame 个字节)
	ctrlData.Success = 1;  
	if(HAL_UART_Receive_DMA(&ctrlUart, IOPool_pGetWriteData(ctrlUartIOPool)->ch, size_frame) != HAL_OK){
			Error_Handler();
		printf( "ctrlUartInit error" );
	} 
}

extern xSemaphoreHandle xSemaphore_uart; //妙算串口互斥信号量
void ctrlUartRxCpltCallback(){ //控制串口回调函数
	static portBASE_TYPE xHigherPriorityTaskWoken;
  xHigherPriorityTaskWoken = pdFALSE;
	//osStatus osMessagePut (osMessageQId queue_id, uint32_t info, uint32_t millisec);
	IOPool_getNextWrite(ctrlUartIOPool);
	if( ctrlUartFlag == byte_EOF){   //若ctrlUartFlag 为结尾字节，正常
		if(HAL_UART_Receive_DMA(&ctrlUart, IOPool_pGetWriteData(ctrlUartIOPool)->ch, size_frame) != HAL_OK){
			Error_Handler();
		}
		xSemaphoreGiveFromISR(xSemaphore_uart, &xHigherPriorityTaskWoken);
		uint8_t *pData = IOPool_pGetReadData(ctrlUartIOPool, 0)->ch;
		ctrlData = xUartprocess( pData );
			if( ctrlData.Success == 1) 	ctrlUartFlag = byte_EOF;			 
			else 	ctrlUartFlag = 0;				
	}
	else{//若ctrlUartFlag 不是结尾字节，阻塞读取
    	if(HAL_UART_Receive_DMA(&ctrlUart, &ctrlUartFlag, 1) != HAL_OK){
			Error_Handler();}
	}
//上下文切换	
  if( xHigherPriorityTaskWoken == pdTRUE ){
   portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
	}
}
/*真.串口回调函数*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	if(UartHandle == &rcUart){
		rcUartRxCpltCallback();
	}else if(UartHandle == &ctrlUart){
		ctrlUartRxCpltCallback();
	}
}   

