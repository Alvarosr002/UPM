#include "com.h"

/* Colas de mensajes */
extern osMessageQueueId_t mid_Slave_Com_MsgQueue;
MSGQUEUE_OBJ_MEAS rx_medidas;

/* Functions prototypes */
void Th_Slave_Com (void *argument);                   // thread function
void Th_Com_Slave (void *argument);                   // thread function

/* Variables de medición */
uint16_t v_rtd; 													// Voltaje RTD 
double hum;																// Humedad
float luz;																// Luminosidad 
int aq;																		// Calidad del aire
float aqLvl;															// Nivel de agua
uint16_t v_crnt;													// Voltaje de consumo


static bool uart_initialized = false;
uint8_t commandrx[3];												//Comando que se recibe desde la Placa A(Master)
extern uint8_t commandrx[3];

osThreadId_t TID_UartRx;	
osThreadId_t TID_UartTx;

uint8_t DATATx[14];   //Datos que enviamos al master/Placa A
bool trama_incorrecta = false;

int Init_ThCom (void) {
 
  /* Hilo encargado de enviar info al PC */
  TID_UartTx = osThreadNew(UartTx, NULL, NULL);		// Hilo de transmisión
  if (TID_UartTx == NULL) {
    return(-1);
  }
  
  /* Hilo encargado de recibir info del PC */
  TID_UartRx = osThreadNew(UartRx, NULL, NULL);		// Hilo de recepción
  if (TID_UartRx == NULL) {
    return(-1);
  }
  
  UART_Init();								// Inicializa el periférico UART
  
  return(0);
}



void USART_Callback(uint32_t event) {
    if (event & ARM_USART_EVENT_RECEIVE_COMPLETE) {
				osThreadFlagsSet(TID_UartRx, ARM_USART_EVENT_RECEIVE_COMPLETE);
//        printf("[UART CALLBACK] Comando recibido\n");
    }

    if (event & ARM_USART_EVENT_SEND_COMPLETE) {
				osThreadFlagsSet(TID_UartTx, ARM_USART_EVENT_SEND_COMPLETE);
//        printf("[UART CALLBACK] Respuesta enviada al master\n\n");
    }
}


void UART_Init(void) {
    // 1. Inicialización 
    int32_t status; 
	
		status = UARTdrv->Initialize(USART_Callback);  
    if(status != ARM_DRIVER_OK) {
//        printf("[UART] Error en Initialize: %d\n", status);
        return;
    }
    // 2. Encender el periférico 
    status = UARTdrv->PowerControl(ARM_POWER_FULL);
    if(status != ARM_DRIVER_OK) {
//        printf("[UART] Error en PowerControl: %d\n", status);
        return;
    }
		
		
    // 3. Configurar parámetros UART 
    status = UARTdrv->Control(
        ARM_USART_MODE_ASYNCHRONOUS |  // Modo UART (asíncrono)
        ARM_USART_DATA_BITS_8      |   // 8 bits de datos
        ARM_USART_PARITY_NONE      |   // Sin paridad
        ARM_USART_STOP_BITS_1      |   // 1 bit de parada
        ARM_USART_FLOW_CONTROL_NONE,   // Sin control de flujo
        9600                         // Baudrate 
    );
	
		if(status == ARM_DRIVER_OK) {
//        printf("[UART] Inicializado correctamente a 9600 baudios\n");
        uart_initialized = true;
    } else {
//        printf("[UART] Error en Control: %d\n", status);
    }
		
		// 4. Habilitar TX y RX
    status = UARTdrv->Control(ARM_USART_CONTROL_TX, 1);
    if (status != ARM_DRIVER_OK) {
//        printf("[UART] Error al habilitar TX: %d\n", status);
        return;
    }
			// Habilitar interrupción por recepción usando el callback (no IRQ directa)
    status = UARTdrv->Control(ARM_USART_CONTROL_RX, 1);
    if (status != ARM_DRIVER_OK) {
//        printf("[UART] Error al habilitar RX: %d\n", status);
        return;
    }
}

void UART_Deinit(void){
  
  int32_t status; 
	status = UARTdrv->Uninitialize();
}

int32_t UART_ListenCommand(uint8_t* command_buffer) {
    if (!uart_initialized) {
//        printf("[UART] Error: No inicializado antes de recibir comando\n");
        return ARM_DRIVER_ERROR;
    }
//    printf("[UART] Esperando comando...\n");
		
    int32_t result = UARTdrv->Receive(command_buffer, 3);
    if (result != ARM_DRIVER_OK) {
//      printf("[UART] Error al iniciar recepcion del comando: %d\n", result);
        return result;
    }

		osThreadFlagsWait(ARM_USART_EVENT_RECEIVE_COMPLETE, osFlagsWaitAny, osWaitForever);

    
    return ARM_DRIVER_OK;
}

int32_t UART_SendData(uint8_t* datatx) {
    if (!uart_initialized) {
//        printf("[UART] Error: No inicializado antes de enviar temperatura\n");
        return ARM_DRIVER_ERROR;
    }


    int32_t result = UARTdrv->Send(datatx, 14);
    if (result != ARM_DRIVER_OK) {
//        printf("[UART] Error al enviar temperatura: %d\n", result);
        return result;
    }

		osThreadFlagsWait(ARM_USART_EVENT_SEND_COMPLETE, osFlagsWaitAny, osWaitForever);

    return ARM_DRIVER_OK;
}


/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  Thread 'UARTRx': UARTRx handler for reception                          
 *---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
void UartRx (void *arg) {

	while (1) {
		 // 1. Esperar comando. EL MASTER SOLO ME ENVIA 3 BYTES
		if (UART_ListenCommand(commandrx) == ARM_DRIVER_OK) {
			if(commandrx[0] == COMIENZO_TRAMA){
				if(commandrx[2] == FINAL_TRAMA){
				// 2. Notificar a Tx SOLO cuando el comando esté listo
				osThreadFlagsSet(TID_UartTx,0x01);
				// 3. Esperar confirmación de que Tx ha procesado el comando
				osThreadFlagsWait(0x02, osFlagsWaitAny, osWaitForever);
				}
			} else {
				trama_incorrecta = true;
				osThreadFlagsSet(TID_UartTx,0x01);
				osThreadFlagsWait(0x02, osFlagsWaitAny, osWaitForever);
			}
		} else {
//           printf("Error al recibir comando\n");
		}
	}
}

/*---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  Thread 'UARTTx': UARTtx handler for transmision
 *--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
void UartTx (void *arg) {
	
  while (1) {
		
    // Espera nuevos datos de medición
    osMessageQueueGet(mid_Slave_Com_MsgQueue, &rx_medidas, NULL, 0U);

    // Almacena las mediciones
    v_rtd = rx_medidas.Voltaje_RTD;
    hum = rx_medidas.humidity;
    luz = rx_medidas.luminosity;
    aq = rx_medidas.air_quality;
    aqLvl = rx_medidas.aquaLevel;
    v_crnt = rx_medidas.Voltaje_CONSUMPTION;
    
    // 1. Esperar notificación de comando recibido
    osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever);
    // 2. Borrado de buffer de transmision a enviar al MASTER 
    memset(DATATx, 0x00, sizeof(DATATx));

    // Poner byte de inicio de trama (por ejemplo 0x7E)
		if(!trama_incorrecta)	// De normal va a estar a false trama_incorrecta
			DATATx[0] = COMIENZO_TRAMA;
		else
			DATATx[0] = ERROR_TRAMA;
    // 3. Procesar comando (ya está almacenado en commandrx)
    switch (commandrx[1]) {

			case 0x01: { // Envio de todos los datos
					// Temperatura
					float rtdv_aux = ((float)v_rtd*3.3f)/4096.0f;
					float rtd_V =(rtdv_aux*15.1745)-4.48f;
					rtd_V = rtd_V * 100;
					uint16_t rtd_scaled = (uint16_t)rtd_V;
					DATATx[1] = (uint8_t)(rtd_scaled >> 8)  & 0xFF;	// Byte más significativo (MSB)
					DATATx[2] = (uint8_t)(rtd_scaled & 0xFF);       // Byte menos significativo (LSB)
				
					// Consumo
					float consumption_V = ((((float)v_crnt*3.3f)/4096));
					consumption_V = consumption_V * 100;
					uint16_t consumption_scaled = (uint16_t)consumption_V;
					DATATx[3] = (uint8_t)(consumption_scaled >> 8)  & 0xFF;	// Byte más significativo (MSB)
					DATATx[4] = (uint8_t)(consumption_scaled & 0xFF);       // Byte menos significativo (LSB)
																																	
					// Lux
					uint32_t luz_100x = (uint32_t)(luz * 100.0f); 
					DATATx[5] = (uint8_t)((luz_100x >> 16) & 0xFF);  // Byte más significativo (MSB)
					DATATx[6] = (uint8_t)((luz_100x >> 8)  & 0xFF);  // 
					DATATx[7] = (uint8_t)(luz_100x & 0xFF);          // Byte menos significativo (LSB)
					
					// Calidad del Aire
					DATATx[8] = (uint8_t)aq;	// Byte más significativo (MSB)
					
					// Humedad
					uint16_t humscaled = (uint16_t)(hum * 100);  // Byte más significativo (MSB)
					DATATx[9] = (uint8_t)(humscaled >> 8);       // 
					DATATx[10] = (uint8_t)(humscaled & 0xFF);     // Byte menos significativo (LSB)
					
					// Nivel de agua
					uint16_t level_scaled = (uint16_t)(aqLvl * 100);  // Byte más significativo (MSB)
					DATATx[11] = (uint8_t)(level_scaled >> 8);        // 
					DATATx[12] = (uint8_t)(level_scaled & 0xFF);      // Byte menos significativo (LSB)

//								printf("[UART TX] Datos enviados:\n"
//											"Temperatura: %.2f *C\n"
//											"Consumo: %.2f A\n"
//											"Luminosidad: %.2f lx\n"
//											"Calidad del Aire: %d\n"
//											"Humedad: %.2f %%\n"
//											"Nivel de Agua: %.2f %%\n",
//											rtd_V/100, consumption_V/100, luz, aq, hum, aqLvl);
					break;
			}
			
			case 0xBB: { //COMANDO SLEEP
				DATATx[12] = 0xBB;
				break;
			}
			
			case 0xAA: { //WAKE UP
				DATATx[12] = 0xAA;
				break;
			}

			default:{
				trama_incorrecta = false;
//              printf("[UART TX] Comando desconocido: 0x%02X\n", commandrx);
				break;
			}
		}
		// 4. Enviar datos
			
		// Byte de fin de trama (por ejemplo 0x7F)
		DATATx[13] = FINAL_TRAMA;
	
		if (UART_SendData(DATATx) != ARM_DRIVER_OK) {
//            printf("[UART TX] Error al enviar respuesta\n");
		}
		
    HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_7);	// Indica que se envia info por UART.`Prueba Julián
    
    // 5. Confirmar a Rx que hemos terminado
    osThreadFlagsSet(TID_UartRx, 0x02);
  }
}



