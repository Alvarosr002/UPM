#include "uart.h"
#include "Principal_Master.h"
#include "AT24C256.h"

/////////////UART////////////////////////////////////////////////////////////
extern ARM_DRIVER_USART Driver_USART3;
static ARM_DRIVER_USART *UARTdrv = &Driver_USART3;

/////////////Auxiliares//////////////////////////////////////////////////////
static bool uart_initialized = false;
static bool first_command_send = false;

/////////////Variables de comunicacion UART//////////////////////////////////
static uint8_t rx_buffer[14];  
uint8_t data_rx[12];

/////////////EXTERN's////////////////////////////////////////////////////////
extern Estados_Principal_t estadoMaster;
extern osTimerId_t tim_id1;

extern RTC_HandleTypeDef RtcHandle;
extern RTC_TimeTypeDef rtcTimeConfig;
uint8_t aShowTime[10] = {0};
uint8_t aShowDate[10] = {0}; 
uint8_t hh, mm, ss = 0;

void USART_Callback(uint32_t event) {

	if (event & ARM_USART_EVENT_SEND_COMPLETE){
			osThreadFlagsSet(TID_UartTx, ARM_USART_EVENT_SEND_COMPLETE);
	}
	if (event & ARM_USART_EVENT_RECEIVE_COMPLETE){
			osThreadFlagsSet(TID_UartRx, ARM_USART_EVENT_RECEIVE_COMPLETE);
	}
}

void UART_Init(void) {
	// 1. Inicialización 
	int32_t status; 
	status = UARTdrv->Initialize(USART_Callback);  
	if(status != ARM_DRIVER_OK) {
			return;
	}
	// 2. Encender el periférico 
	status = UARTdrv->PowerControl(ARM_POWER_FULL);
	if(status != ARM_DRIVER_OK) {
			return;
	}
	// 3. Configurar parámetros UART 
	status = UARTdrv->Control(
			ARM_USART_MODE_ASYNCHRONOUS |  // Modo UART (asíncrono)
			ARM_USART_DATA_BITS_8      |   // 8 bits de datos
			ARM_USART_PARITY_NONE      |   // Sin paridad
			ARM_USART_STOP_BITS_1      |   // 1 bit de parada
			ARM_USART_FLOW_CONTROL_NONE,   // Sin control de flujo
			9600                        // Baudrate 
	);

	if(status == ARM_DRIVER_OK) {
			printf("[UART] Inicializado correctamente a 9600 baudios\n");
			uart_initialized = true;
	}
	// 4. Habilitar TX y RX
	status = UARTdrv->Control(ARM_USART_CONTROL_TX, 1);
	if (status != ARM_DRIVER_OK) {
			return;
	}

	status = UARTdrv->Control(ARM_USART_CONTROL_RX, 1);
	if (status != ARM_DRIVER_OK) {
			return;
	}

}

int32_t UART_SendCommand(uint8_t* command) {
	if (!uart_initialized) {
			return ARM_DRIVER_ERROR;
	}
	int32_t result = UARTdrv->Send(command, 3);
	if(result != ARM_DRIVER_OK) {
			return result;
	}
	osThreadFlagsWait(ARM_USART_EVENT_SEND_COMPLETE, osFlagsWaitAny, osWaitForever);
	return ARM_DRIVER_OK;
}


int32_t UART_ReceiveData(uint8_t* data) {
	if(!uart_initialized) {
			return ARM_DRIVER_ERROR;
	}
	return UARTdrv->Receive(data, 14);
}


/*----------------------------------------------------------------------------
  Thread 'UART': UART handler for transmision
 *---------------------------------------------------------------------------*/
void UartTx (void *arg) {

	while (1) {
	 // 1. Esperar señal para iniciar comunicación/enviar comando
	 osThreadFlagsWait(0xFF, osFlagsWaitAll, osWaitForever);
	 comando[0] = COMIENZO_TRAMA;
	 comando[2] = FINAL_TRAMA;
	 // 2. Enviar comando
	 uint32_t timeout = 100;
	 while(timeout-- && UARTdrv->GetStatus().tx_busy) osDelay(1);
	 if (UART_SendCommand(comando) == ARM_DRIVER_OK) {
	 }
	}
}

/*----------------------------------------------------------------------------
  Thread 'UART': UART handler for reception
 *---------------------------------------------------------------------------*/
void UartRx (void *arg) {
		
	while (1) {
	
		// 1. Iniciar recepción
		if (UART_ReceiveData(rx_buffer) != ARM_DRIVER_OK) {
//						printf("[ERROR UART RECEIVE] Rx:Error al preparar recepción\n");
			}
		// 2. Esperar datos (con callback USART)   
		osThreadFlagsWait(ARM_USART_EVENT_RECEIVE_COMPLETE, osFlagsWaitAny, osWaitForever);
		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_7);
		// Procesar los datos recibidos 
		if(rx_buffer[0] == COMIENZO_TRAMA){
			if(rx_buffer[13] == FINAL_TRAMA){
				// Copiar los datos importantes que son 12 bytes(quitando el byte de comienzo de trama y final de trama)
				memcpy(data_rx, &rx_buffer[1], 12);			
				//Procesar trama recibida desde el slave
				procesar_trama();
			} else { 
				// Trama no válida, resetear buffer
				memset(rx_buffer, 0, sizeof(rx_buffer));
				if(comando[1]==0x01){
					osThreadFlagsSet(TID_PRINC_MASTER, comando[1]);
				}else if(comando[1]==0xAA){
					osThreadFlagsSet(TID_PRINC_MASTER, 0x08);
				}else if(comando[1]==0xBB){
					osThreadFlagsSet(TID_PRINC_MASTER, 0x80);
				}
			}	
		}else if(rx_buffer[0] == ERROR_TRAMA){
				if(comando[1]==0x01){
					osThreadFlagsSet(TID_PRINC_MASTER, comando[1]);
				}else if(comando[1]==0xAA){
					osThreadFlagsSet(TID_PRINC_MASTER, 0x08);
				}else if(comando[1]==0xBB){
					osThreadFlagsSet(TID_PRINC_MASTER, 0x80);
				}
		}else { 
			if(comando[1]==0x01){
				osThreadFlagsSet(TID_PRINC_MASTER, comando[1]);
			}else if(comando[1]==0xAA){
				osThreadFlagsSet(TID_PRINC_MASTER, 0x08);
			}else if(comando[1]==0xBB){
				osThreadFlagsSet(TID_PRINC_MASTER, 0x80);
			}
		}
	}
}

void procesar_trama (void){
	
	// 1. Procesar comandos		
	if(comando[1]==0x01){							
/////////// Temperatura
		uint16_t rtd_scaled = ((uint16_t)data_rx[0] << 8) | data_rx[1];
		meas_global.temperatura = ((float)rtd_scaled) / 100.0f;  // Te da el voltaje original con decimales
		printf("[Comando 0x01] Temperatura: 0x%02X 0x%02X ºC(Hexadecimal) || Temperatura: %0.1f ºC \n", data_rx[0],data_rx[1],meas_global.temperatura);

/////////// Consumo
		uint16_t consumption_scaled = ((uint16_t)data_rx[2] << 8) | data_rx[3];
		meas_global.consumo = (((float)consumption_scaled) / 100.0f)/0.0064f;  // Te da el consumo en mA original con decimales
		printf("[Comando 0x01] Consumo: 0x%02X 0x%02X V(Hexadecimal) || Consumo: %0.2f mA\n", data_rx[2],data_rx[3],meas_global.consumo);

/////////// Luminosidad
		uint32_t luz_100x = (data_rx[4] << 16) | (data_rx[5] << 8) | data_rx[6];
		meas_global.lum = (float)luz_100x / 100.0f;
		printf("[Comando 0x01] Luminosidad: 0x%02X 0x%02X 0x%02X lx(Hexadecimal) || Luminosidad: %.2f lx\n", data_rx[4],data_rx[5],data_rx[6],meas_global.lum);
		
/////////// Calidad del aire
		meas_global.air_quality = (int)data_rx[7];
		printf("[Comando 0x01] Calidad de aire: 0x%02X (Hexadecimal) || Calidad de aire: %d IAQ\n",data_rx[7],meas_global.air_quality);
	
/////////// Humedad
		uint16_t hum_scaled = ((uint16_t)data_rx[8] << 8) | data_rx[9];
		meas_global.humidity =(float)hum_scaled/ 100.0f;
		printf("[Comando 0x01] Humedad: 0x%02X 0x%02X || Humedad: %.2f %%\n", data_rx[8],data_rx[9],meas_global.humidity);
		
/////////// Nivel de agua 
		uint16_t nivel_agua_recibido = ((uint16_t)data_rx[10] << 8) | (uint16_t)data_rx[11];
		meas_global.quantity = (float)nivel_agua_recibido / 100.0; // Convertir a porcentaje
		printf("[Comando 0x01] Nivel de agua: %02X %02X %%|| Nivel de agua: %0.2f %%\n\n", data_rx[10],data_rx[11], meas_global.quantity);
		
		//[EEPROM] Guardar en la memoria solo en el modo STANDBY
		if(estadoMaster == STANBY){
			guardar_temperatura(rtd_scaled);
			guardar_humedad(hum_scaled);
			guardar_luminosidad(luz_100x);
			guardar_calidad_aire(meas_global.air_quality);
			guardar_nivel_agua(nivel_agua_recibido);
			
			RTC_Hora_Fecha(aShowTime, aShowDate);
				
			hh = stimestructure.Hours;
			mm = stimestructure.Minutes;
			ss = stimestructure.Seconds;
			guardar_hora(hh, mm, ss);
		}
/////////// Flag para hilo principal_master	avisando de que ya recibi los datos y han sido procesados																
			osThreadFlagsSet(TID_PRINC_MASTER, comando[1]);							
	}else if(comando[1]==0xAA){
		if(data_rx[11]==0xAA){
			osThreadFlagsSet(TID_PRINC_MASTER, 0x08);
			printf("[Comando %02X]ACK de confirmacion dormir al SLAVE\n\n", comando[1]);
		}
		
	}else if(comando[1]==0xBB){
		if(data_rx[11]==0xBB){
			osThreadFlagsSet(TID_PRINC_MASTER, 0x80);
			printf("[Comando %02X]ACK de confirmacion despertar al SLAVE\n\n", comando[1]);
		}
	}
}





