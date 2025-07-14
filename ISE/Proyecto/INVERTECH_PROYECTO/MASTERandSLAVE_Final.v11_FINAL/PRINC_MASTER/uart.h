#ifndef __UART_H
#define __UART_H

#include "stm32f4xx_hal.h"
#include "Driver_USART.h"
#include "cmsis_os2.h"
#include "string.h"
#include "stdio.h"

#define COMIENZO_TRAMA 0x7E  
#define FINAL_TRAMA   0x7F 
#define ERROR_TRAMA 0xFF 

void UART_Init(void);
int32_t UART_SendCommand(uint8_t* command);
int32_t UART_ReceiveData(uint8_t* data);
void UartTx  (void *arg);
void UartRx  (void *arg);	
void procesar_trama (void);


extern osThreadId_t TID_UartTx;
extern osThreadId_t TID_UartRx;
osThreadId_t TID_UartTx;	
osThreadId_t TID_UartRx;

#endif 