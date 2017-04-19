#ifndef FRAMEWORK_UART_H
#define FRAMEWORK_UART_H

#define size_frame 14  //定义妙算串口帧长度/字节
#define byte_SOF 0x7d    //起始字节	
#define byte_EOF 0x7e    //终止字节
#define byte_ESCAPE 0xff //转义字节

#include "framework_utilities_iopool.h"

IOPoolDeclare(ctrlUartIOPool, struct{uint8_t ch[size_frame];});

void ctrlUartInit(void);
void ctrlUartRxCpltCallback(void);
void vSendUart(xdata_ctrlUart data);
xdata_ctrlUart xUartprocess(uint8_t *pData);

#endif
