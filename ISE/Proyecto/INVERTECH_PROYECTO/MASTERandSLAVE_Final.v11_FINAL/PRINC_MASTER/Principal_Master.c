#include "Principal_Master.h"
//#include "LEDs.h"
#include "AT24C256.h"
#include "SNTP.h"




// VARIABLES GLOBALES
uint8_t comando[3] = {0x00,0x00,0x00};

//Auxiliares
uint32_t auxiliar_despertar;
uint32_t auxiliarmedida;
uint32_t aux;
bool auxbool=true;

bool auxxd=false;

//Interrupciones NFC and WEB
bool NFC;
bool web=false;
extern bool web;
//PRESENCIAL
osTimerId_t tim_id2;  
static uint32_t exec2;

Estados_Principal_t estadoMaster = INIT;
extern Estados_Principal_t estadoMaster;

// Declaracion de funciones static
static void init_despertar_Slave(void);
void get_Hora (void);
void guardarMedidas(void);

void despertar_Slave(void);
extern void despertar_Slave(void);

static uint32_t dormir_Slave(void);

void automata_PrincipalMaster(void){
	switch(estadoMaster){
		case STANBY:
			//  La Web y el modo Presencial estan inactivos, la placa Slave se duerme
      ledsOFF();
      uint32_t flagAlarma = osThreadFlagsWait(0x70, osFlagsWaitAny, osWaitForever);
		// Esperar Alarma cada x tiempo
      if(flagAlarma == 0x10){
        RTC_Alarm_Config();
        despertar_Slave();
        comando[1] = MEAS;
        osThreadFlagsSet(TID_UartTx, 0xFF);
        // Esperar confirmacion
        auxiliarmedida = osThreadFlagsWait(MEAS, osFlagsWaitAny, osWaitForever);
        dormir_Slave();
      }else if(flagAlarma == 0x20 && insideGreenhouse == true && web == false){
        despertar_Slave();
        HAL_RTC_DeactivateAlarm(&RtcHandle, RTC_ALARM_A);
        estadoMaster = PRESENCIAL;
      }else if(flagAlarma == 0x40 && web == true && insideGreenhouse == false){
        despertar_Slave();
        HAL_RTC_DeactivateAlarm(&RtcHandle, RTC_ALARM_A);
        estadoMaster = WEB;
      }
		break;
		
		case INIT:
			UART_Init();
			I2C_Inicialize();
			RTC_Config();
			init_despertar_Slave();
      Init_Altavoz();
      Init_RFID();
      initMBED_leds(); 
      init_SNTP ();  
			osDelay(5000);   //Tiempo para que se pueda inicializar todo y ademas para coger la hora del servidor SNTP
      printf("INVERT TECH se ha configurado correctamente\n\r");
      dormir_Slave();
			estadoMaster = STANBY;
		break;

		case PRESENCIAL:
			if(insideGreenhouse){
				printf("\n\n MODO PRESENCIAL \n\n\r");
				comando[1] = MEAS;
				osThreadFlagsSet(TID_UartTx, 0xFF);
				ledsON(meas_global.quantity);
				auxiliarmedida = osThreadFlagsWait(MEAS, osFlagsWaitAny, osWaitForever);
				osDelay(1000);
			}else{ 
				if(web){
					estadoMaster = WEB;
				}else{
          RTC_Alarm_Config();
          dormir_Slave();
				  estadoMaster = STANBY;
        }
			}
			
		break;
		
		case WEB:

		  if(!web){
        if(insideGreenhouse == true){
          estadoMaster = PRESENCIAL;
        }else{
          RTC_Alarm_Config();
          dormir_Slave();
          estadoMaster = STANBY;
        }
        
			}else{
        printf("\n\n MODO WEB \n\n\r");
        comando[1]  = INIT_WEB;
        ledsOFF();
        osThreadFlagsSet(TID_UartTx, 0xFF);
        osThreadFlagsWait(INIT_WEB, osFlagsWaitAny, osWaitForever);
        osDelay(1000); 
      }
     
		break;
	}
}

void despertar_Slave(void){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
  osDelay(5000);	// 5 segundos
	comando[1]  = WAKE_UP;
	osThreadFlagsSet(TID_UartTx, 0xFF);
	auxiliar_despertar = osThreadFlagsWait(0x80, osFlagsWaitAll, osWaitForever);	// Cuando recibe 0xBB [Slave despierto]
}
static uint32_t dormir_Slave(void){
	uint32_t aux;
	comando[1]  = SLEEP;
	osThreadFlagsSet(TID_UartTx, 0xFF);
	aux = osThreadFlagsWait(0x08, osFlagsWaitAll, osWaitForever);	// Cuando recibe 0xAA [Slave preparado para dormir]
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
	return aux;
}
static void init_despertar_Slave(void){
	
	// INICIALIZACIÓN DEL GPIO PARA DESPERTAR EL SLAVE
	__HAL_RCC_GPIOB_CLK_ENABLE();
	
	GPIO_InitTypeDef despertar ={
		.Pin = GPIO_PIN_1,
		.Mode = GPIO_MODE_OUTPUT_PP,
//		.Pull = GPIO_PULLDOWN,
		.Speed = GPIO_SPEED_FREQ_VERY_HIGH
	};
	
	HAL_GPIO_Init(GPIOB, &despertar);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);

}
