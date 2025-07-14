#include "slave.h"
 
/*----------------------------------------------------------------------------
 *      Thread 1 'Thread_Name': Sample thread
 *---------------------------------------------------------------------------*/

/* Threads */
osThreadId_t tid_slave;                        // thread id

/* Colas de medidas */
MSGQUEUE_ADCs_t rx_ADCs;
MSGQUEUE_BME680_t rx_BME680;
MSGQUEUE_BH1750_t rx_BH1750;
MSGQUEUE_HCSR04_t rx_HCSR04;

/* Queue Objects*/

MSGQUEUE_OBJ_MEAS tx_medidas = {
  .Voltaje_RTD = 0.0,
  .humidity = 0.0,
  .luminosity = 0.0,
  .air_quality = 0,
  .aquaLevel = 0.0,
  .Voltaje_CONSUMPTION = 0
};

osMessageQueueId_t mid_Slave_Com_MsgQueue;
extern osMessageQueueId_t mid_Slave_Com_MsgQueue;

/* Functions prototypes */
int Init_Slave_Com_Queue (void);
void ThSlave (void *argument);                   // thread function
void InitializeSensors (void);
void processCommand (uint8_t cmd);
void RefreshMeas(void);

/*FUNCIONES AUXILIARES*/
void PrintSensorValues(void);

/* Variables */
float waterLevel = 0.0;
float luminosity = 0.0;
uint32_t RTD_Voltaje = 0;
uint32_t CONSUMPTION_Voltaje = 0;
extern uint8_t commandrx[3];

/* Structure */
typedef enum {
  INITIALIZE,
  WAKE_UP,
  LOW_POWER,
  ALL_MEAS
}StatesPrincipal_t;

StatesPrincipal_t currentState = LOW_POWER; //Por defecto

void readGPIO_Input(StatesPrincipal_t estado);

/* Externs */
extern void Init_I2C (void);
extern void Init_I2C_MASTER(void);
extern void RTD_pin_F429ZI_config();
extern void CONSUMPTION_pin_F429ZI_config();
extern void GPIO_HCSR04 (void);


int Init_Slave (void) {
 
  tid_slave = osThreadNew(ThSlave, NULL, NULL);
  if (tid_slave == NULL) {
    return(-1);
  }
  
  Init_Slave_Com_Queue();
  
  return(0);
}

int Init_Slave_Com_Queue (void){
  
  mid_Slave_Com_MsgQueue = osMessageQueueNew(SLAVE_COM_MSG_QUEUE, sizeof(MSGQUEUE_OBJ_MEAS), NULL);
  if(mid_Slave_Com_MsgQueue == NULL){
    return -1;
  }
  
  return (0);
  
}
 
void ThSlave (void *argument) {
  
  InitializeSensors();
  Init_PB1_WAKEUP();
  while (1) {
    // Insert thread code here...
    /* Crear una función que se llame wait for command y dependiendo del comando se vaya a un estado u otro */
//		PrintSensorValues();
//		osDelay(1000);
    processCommand(commandrx[1]);
    
    switch (currentState){
      
      //Here we will initialize the COM Thread
      case INITIALIZE:	// NO SE USA PARA NADA
        InitializeSensors();
      break;
      
      case WAKE_UP:
        readGPIO_Input(currentState);
//        currentState = INITIALIZE;
      break;
      
      case LOW_POWER:
        readGPIO_Input(currentState);
      break;
      
      case ALL_MEAS:
        osMessageQueueGet(mid_MsgQueueADCs, &rx_ADCs, NULL, 0u);
        osMessageQueueGet(mid_MsgQueueBME680, &rx_BME680, NULL, 0u);
        osMessageQueueGet(mid_MsgQueueBH1750, &rx_BH1750, NULL, 0u);
        osMessageQueueGet(mid_MsgQueueHCSR04, &rx_HCSR04, NULL, 0u);
        
        tx_medidas.Voltaje_RTD = rx_ADCs.RTD_Vol;
        tx_medidas.Voltaje_CONSUMPTION = rx_ADCs.CONSUMPTION_Vol;
        tx_medidas.humidity = rx_BME680.humidity;
        tx_medidas.air_quality = rx_BME680.air_quality;
        tx_medidas.luminosity = rx_BH1750.lum;
        tx_medidas.aquaLevel = rx_HCSR04.quantity;
      
        osMessageQueuePut(mid_Slave_Com_MsgQueue, &tx_medidas, NULL, 100);
        
      break;
      
    }
    //osThreadYield();                            // suspend thread
  }
}

void InitializeSensors (void){
  Init_HCSR04 ();
  Init_ThBH1750();
  Init_ThADCs();
  Init_Thbme680();
  Init_LEDs();
}

void PrintSensorValues(void) {
//    printf("----- Medidas de los sensores -----\n");
//    printf("Voltaje RTD: %u mV\n", tx_medidas.Voltaje_RTD);
//    printf("Humedad: %.2f %%\n", tx_medidas.humidity);
//    printf("Luminosidad: %.2f lux\n", tx_medidas.luminosity);
//    printf("Calidad del aire (indice): %d\n", tx_medidas.air_quality);
//    printf("Nivel de agua: %.2f cm\n", tx_medidas.aquaLevel);
//    printf("Voltaje de consumo: %u mV\n", tx_medidas.Voltaje_CONSUMPTION);
//    printf("-----------------------------------\n");
}


void processCommand (uint8_t cmd){
  
  switch(cmd){
    case 0x01: //All meas
      currentState = ALL_MEAS;
      break;
    
    case 0xAA: //Sleep
      currentState = LOW_POWER;
      break;
    
    case 0xBB: //Wake-Up
      currentState = WAKE_UP;
      break;
  }
}

void readGPIO_Input(StatesPrincipal_t estado){
  
  if(estado == WAKE_UP){
    if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_SET){ //Pin del pulsador
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
			osDelay(10);
    }
  }
  
  if(estado == LOW_POWER){
    if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_RESET){
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
      Deinit_I2C_BME680();
      DeInit_I2C();
      NVIC_DisableIRQ(USART3_IRQn);
      Enter_SleepMode ();
      NVIC_EnableIRQ(USART3_IRQn);
      GPIO_HCSR04();
      CONSUMPTION_pin_F429ZI_config();
      RTD_pin_F429ZI_config();
      Init_LEDs();
      Init_I2C();
      Init_I2C_MASTER();
    }
    
    osDelay(10);
  }
  
}

void Init_LEDs(void){
	
	GPIO_InitTypeDef GPIO_InitStruct;
	
	__HAL_RCC_GPIOB_CLK_ENABLE();	// Habilitar el reloj asociado al puerto de los LEDs. En este caso el reloj del puerto B
	
	// Configuracion de los LEDs VERDE|AZUL|ROJO  = PIN 0|7|14
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_7 | GPIO_PIN_14;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
};

