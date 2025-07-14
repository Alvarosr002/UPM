/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __COM_H
#define __COM_H

#include "stm32f4xx_hal.h"
#include "Driver_USART.h"
#include "cmsis_os2.h"                  
#include "stdio.h"
#include <stdlib.h>  
#include <string.h>
#include "slave.h"

#define COMIENZO_TRAMA 0x7E  
#define FINAL_TRAMA   0x7F 
#define ERROR_TRAMA 0xFF 

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
  
   #define COM_SLAVE_MSG_QUEUE 10
  
/* Exported functions ------------------------------------------------------- */
	
int Init_ThCom (void);

void UART_Init(void);
void UartRx  (void *arg);
void UartTx  (void *arg);	
void UART_Deinit(void);
  
int32_t UART_ListenCommand(uint8_t* command_buffer);
int32_t UART_SendData(uint8_t* temp_buffer);

extern ARM_DRIVER_USART Driver_USART3;
static ARM_DRIVER_USART *UARTdrv = &Driver_USART3;

extern osThreadId_t TID_UartTx;
extern osThreadId_t TID_UartRx;
extern void Init_Led(void);



#endif /* __COM_H */