#ifndef FRAMEWORK_UART_H
#define FRAMEWORK_UART_H

#define ctrlUart huart3
void ctrlUartInit(void);
void printCtrlUartTask(void const * argument);
void ctrlUartRxCpltCallback(void);

#endif
