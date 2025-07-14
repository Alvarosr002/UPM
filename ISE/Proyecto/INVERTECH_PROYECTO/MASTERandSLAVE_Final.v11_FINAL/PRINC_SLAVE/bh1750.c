#include "bh1750.h"
#include "Driver_I2C.h"

/* I2C driver instance */
extern ARM_DRIVER_I2C            Driver_I2C1;	// Instacio el controlador I2C específico que Keil genera automáticamente para I2C1
static ARM_DRIVER_I2C *I2Cdrv = &Driver_I2C1;	// Me creo un puntero a una estructura I2C al que le asigno la dir del driver I2C1

/* Thread */
osThreadId_t tid_ThBH1750;												// ID del Hilo

/* Queue */
osMessageQueueId_t mid_MsgQueueBH1750;
MSGQUEUE_BH1750_t tx_BH1750;

/* Functions prototypes */
int Queue_BH1750 (void);
void ThBH1750 (void *argument);										// Funcion principal del Hilo
void Init_I2C (void);
void Init_BH1750(void);
float Brightness_Reading (void);

/* Variables */
static volatile uint32_t I2C_Event;

int Init_ThBH1750 (void) {

  tid_ThBH1750 = osThreadNew(ThBH1750, NULL, NULL);
  if (tid_ThBH1750 == NULL) {
    return(-1);
  }
  
  mid_MsgQueueBH1750 = osMessageQueueNew(MSGQUEUE_BH1750, sizeof(MSGQUEUE_BH1750_t), NULL);
	if (mid_MsgQueueBH1750 == NULL) {
    return(-1);
  }
  
  return(0);
}

void ThBH1750 (void *argument){
	
	Init_I2C();
	Init_BH1750();
	
  tx_BH1750.lum = 0.0;
  
  while(1){
    // Insert thread code here...
		
		tx_BH1750.lum = Brightness_Reading();
		osMessageQueuePut(mid_MsgQueueBH1750, &tx_BH1750, NULL, 0U);
		
		osDelay(1000);
		
    //osThreadYield();															// suspend thread
    
  }

}

/* I2C Signal Event function callback */
void I2C_SignalEvent (uint32_t event) {
 
  /* Save received events */
  I2C_Event |= event;
	
  if (event & ARM_I2C_EVENT_TRANSFER_DONE) {
    /* Transfer or receive is finished */
		osThreadFlagsSet(tid_ThBH1750, TRANSF_DONE_BH1750);	// Flag para indicar que la transferencia o recepcion de informacion ha terminado correctamente
  }
	
}

/* Initialize I2C connected EEPROM */
void Init_I2C (void) {
	
	/* Initialize I2C peripheral */
  I2Cdrv->Initialize (I2C_SignalEvent);	// Inicializa el controlador I2C
	
	/* Power-on I2C peripheral */
  I2Cdrv->PowerControl (ARM_POWER_FULL);	// Activa la alimentación del I2C
	
	/* Configure I2C bus */
  I2Cdrv->Control      (ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_FAST);	// Configura velocidad rapida (400 kHz)
  I2Cdrv->Control      (ARM_I2C_BUS_CLEAR, 0);	// Limpia el bus I2C si está bloqueado
	
}

void DeInit_I2C (void){
  I2Cdrv->Uninitialize();
  
}

/* Inicializacion del sensor BH1750*/
void Init_BH1750(void){
	
	// Comando para encender el sensor
	uint8_t encender_sensor = 0;
	
	encender_sensor = BH1750_POWER_ON;
	
	I2Cdrv->MasterTransmit(BH1750_ADDR, &encender_sensor, 1, false);			// Enviamos a la dirección del sensor el comando correspondiente para despertarlo
	osThreadFlagsWait(TRANSF_DONE_BH1750, osFlagsWaitAny,osWaitForever);	// Esperamos al flag que no sindica que la comunicacion se ha completado
	
	// Comando para establecer la resolucion
	uint8_t data_transmit = 0;
	
	data_transmit = CONTINUOSLY_H_RESOLUTION;
	
	I2Cdrv->MasterTransmit(BH1750_ADDR, &data_transmit, 1, true);				// Enviamos a la dirección del sensor el comando correspondiente para establecer el tipo de resolucion
	osThreadFlagsWait(TRANSF_DONE_BH1750, osFlagsWaitAny,osWaitForever);	// Esperamos al flag que no sindica que la comunicacion se ha completado
	
}

/* Funcion para leer la medida de luminosidad*/

float Brightness_Reading (void){
	
	uint16_t lux = 0;
	uint8_t brightness[2] = {0};
  float temp = 0.0;
	
	I2Cdrv->MasterReceive(BH1750_ADDR, brightness, 2, false);							// Recibimos del sensor la medida y la almacenamos en la variable brightness
	osThreadFlagsWait(TRANSF_DONE_BH1750, osFlagsWaitAny,osWaitForever);	// Esperamos al flag que no sindica que la comunicacion se ha completado
	
	// Validar datos antes de convertir
	if (brightness[0] == 0xFF && brightness[1] == 0xFF) {
		return 0;
	}
	
	lux = ((brightness[0] << 8) | brightness[1]); // Convertir a 16 bits. La medida la obtenemos en big endian, luego los bits mas significativos se encontraran en la posicion 0 del array
	
  temp = (float) lux / 1.2;
  
	return temp;  // Conversión a lux según el datasheet
	
}
