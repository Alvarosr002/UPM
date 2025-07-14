#ifndef __PRINCIPAL_MASTER_H
#define __PRINCIPAL_MASTER_H

/** INCLUEDES **/
	#include "stm32f4xx_hal.h"
	#include <stdlib.h>
	#include <string.h> 
	#include <stdio.h>
	#include <stdbool.h>
	#include "cmsis_os2.h"
	#include "RTC.h"
  #include "rfid.h"
  #include "LEDS.h"
  #include "AT24C256.h"
	
	/** MACROS PARA EL COMANDO **/
	#define SLEEP            0xAA
	#define WAKE_UP          0xBB
	#define INIT_WEB         0x01
	#define MEAS             0x01

extern uint8_t comando[3];

typedef enum{
	INIT,
	STANBY,
	PRESENCIAL,
	WEB,
}Estados_Principal_t;

typedef struct{
	float lum;
	float quantity;
	float humidity;
	int air_quality;
	float consumo;
	float temperatura;
}MEDIDAS_t;

extern MEDIDAS_t meas_global;
void UART_Init(void);
extern osMessageQueueId_t mid_Enviar_Master;
extern osThreadId_t TID_UartTx;
extern osThreadId_t TID_PRINC_MASTER;

// FUNCIONES PRINCIPAL MASTER
void automata_PrincipalMaster(void);
extern void I2C_Inicialize(void);
extern void guardarMedidas(void);


#endif