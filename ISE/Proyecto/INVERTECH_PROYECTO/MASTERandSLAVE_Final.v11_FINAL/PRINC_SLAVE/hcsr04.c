/*
  Autor: Aitor Casado de la Fuente
  Grupo: M04M05M06A
*/

/* ****************************************************************************************
   * Sensor functionality: this sensor work as a proximity sensor, with a supply          *
   * of 3V3 to 5V. It has two pins: TRIGGER & ECHO. The 'TRIG' pin is the responsable     *
   * for ending the ultrasonic burst. This pin should be set to HIGH at least for 10 us.  *
   * After that, the sensor will send out an eight cycle sonic burst at 40 kHz.           *
   * As it names sais, will eject a trigger wavellength. The 'ECHO' pin, after the sonic  *
   * burst has been sent, this pin will go HIGH. Its main uses is for data, taking data   *
   * measurments. It will stay high until an ultrasonic burst us detected back, at        *
   * which point it will go LOW.                                                          *
   *                                                                                      *
   * Use in the project: we will could calculate the distance between the object          *
   * and the sensor measuring the time the 'ECHO' pin is on HIGH level. The time that     *
   * the pin os at HIGH level is the time the burst is travelling.                        *
   **************************************************************************************** */

/*----------------------------------------------------------------------------
 *  OPERATING CONCEPT:                                                        
                                                                               
 *  1. It is necessary to make a 10 us pulse (at least) to the pin TRG in
    order to start the measurment.
    2. Follow-up, the sensor will send out an eight cycle sonic burst at 
    40 kHz, setting the ECHO pin at HIGH level.
    3. This pin (ECHO) will stand out at HIGH level until we receive the
    'echo' pin bounce from the object.
    4. Finally, in order to stablish a correct distance, we only have to
    measure the duration at HIGH level of the pin ECHO, using the formule
    cm = us/58 = us*0.01724.
    
    *WARNING NOTES*:
 *---------------------------------------------------------------------------*/
 
/*----------------------------------------------------------------------------
 *      Thread 1 'Thread_HCSR04': Sample thread
 *---------------------------------------------------------------------------*/
#include "hcsr04.h"

/* Cambiar dependiendo de la altura del depósito */
#define ALTURA_DEPO_CM 11.8f 

 /* Thread */
osThreadId_t tid_hcsr04;                        // thread id

/* Queue */
osMessageQueueId_t mid_MsgQueueHCSR04;
MSGQUEUE_HCSR04_t tx_HCSR04;
MSGQUEUE_HCSR04_t tx_HCSR04 = { .quantity = 0.0};

 /* Timer */
TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim4;
TIM_HandleTypeDef tim7;

 /* Functions prototypes */
void Thread_HCSR04 (void *argument);                   // thread function
int Queue_HCSR04 (void);
void GPIO_HCSR04 (void);
void IC_TIM4_Initialization (void);
void Init_Sensor (void);
void delay(uint32_t n_microsegundos);
float getMeasure (void);

int queue = 0;

 
int Init_HCSR04 (void) {
 
  tid_hcsr04 = osThreadNew(Thread_HCSR04, NULL, NULL);
  if (tid_hcsr04 == NULL) {
    return(-1);
  }
  
  mid_MsgQueueHCSR04 = osMessageQueueNew(MSGQUEUE_HCSR04, sizeof(MSGQUEUE_HCSR04_t), NULL);
	if (mid_MsgQueueHCSR04 == NULL) {
    return(-1);
  }
 
  return(0);
}

void Thread_HCSR04 (void *argument) {
  
  GPIO_HCSR04 ();
  IC_TIM4_Initialization ();
  osDelay(100);
  
  while (1) {
    // Insert thread code here...
    tx_HCSR04.quantity = getMeasure();
    
    osMessageQueuePut(mid_MsgQueueHCSR04, &tx_HCSR04, NULL, 0U);
    
    osDelay(1000);
    
    //osThreadYield();                            // suspend thread
  }
}

/*Here we will generate the Trigger pulse every time
  we want to do a measure. This pulse has to be, at least,
  10 us long. Less than that time we won't get a
  response from the sensor.*/
void Init_Sensor (void){
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);  // TRIG LOW
  delay(2);  // Espera breve antes de enviar pulso
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);    // TRIG HIGH
  delay(10);  // Pulso de 10 µs
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);  // TRIG LOW
}

/*This both functions are unique from this module. They are designed to
  generate/implement high-precision delays using the Data Watchpoint
  and Trace (DWT), specifically for microsecond delays. We cannot use 
  osDelay() due to it receives arguments in ms, and it is impossible
  to introduce decimal numbers as arguments.*/

void delay(uint32_t n_microsegundos) {
    __HAL_RCC_TIM7_CLK_ENABLE();

    tim7.Instance = TIM7;
    tim7.Init.Prescaler = 83;                 // 84 MHz / (83+1) = 1 MHz ? 1 µs por cuenta
    tim7.Init.Period = n_microsegundos - 1;
    tim7.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    tim7.Init.CounterMode = TIM_COUNTERMODE_UP;

    HAL_TIM_Base_Init(&tim7);
    HAL_TIM_Base_Start(&tim7);

    while (__HAL_TIM_GET_COUNTER(&tim7) < (n_microsegundos - 1));

    HAL_TIM_Base_Stop(&tim7);
    HAL_TIM_Base_DeInit(&tim7);
}

float getMeasure(void) {
    uint32_t start = 0, end = 0, timeout = 30000;
    uint32_t counter = 0;
    float width = 0.0f, distance = 0.0f, waterLevel = 0.0f;

    __HAL_TIM_SET_COUNTER(&htim4, 0);
    HAL_TIM_Base_Start(&htim4);

    Init_Sensor ();  // Generar pulso

    // Esperar flanco de subida en ECHO
    while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_RESET) {
        counter = __HAL_TIM_GET_COUNTER(&htim4);
        if (counter > timeout) {
            HAL_TIM_Base_Stop(&htim4);
            return -1.0f;  // Timeout esperando HIGH
        }
    }

    start = __HAL_TIM_GET_COUNTER(&htim4);

    // Esperar flanco de bajada
    while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_SET) {
        counter = __HAL_TIM_GET_COUNTER(&htim4);
        if (counter - start > timeout) {
            HAL_TIM_Base_Stop(&htim4);
            return -1.0f;  // Timeout esperando LOW
        }
    }

    end = __HAL_TIM_GET_COUNTER(&htim4);
    HAL_TIM_Base_Stop(&htim4);

    // Calcular duración
    width = (end >= start) ? (end - start) : (0xFFFF - start + end);
    distance = width / 58.0f;
    waterLevel = ALTURA_DEPO_CM - distance;

    if (waterLevel < 0.0f) return 0.0f;
    else if (waterLevel > ALTURA_DEPO_CM) return 100.0f;
    else return (waterLevel / ALTURA_DEPO_CM) * 100.0f;
}
   
  
/* ****************************************************************************************
   * ECHO: this pin will be used as an Input Capture TIMER in order to could receive the  *
   * bounced pulse. We are using PB6 pin configurated in AF to generate a TIMER (TIM4)    *
   * working as an Input Capture Timer. The counter of the timer is set to 1 us with a    *
   * prescaler of 83 (84) and with a period of 65535. This is the maximum value that this *
   * timer can reach because it is a 16 bit timer. This gives a measuring window util     *
   * 65.535 us, enough to measure objects from 11 metres from the sensor. The IC timer    *
   * is configured to capture both edges of the wave (up & down) using the maximum        *
   * digital filter we can afford.                                                        *
   * TRIGGER: we will use the PB2 pin to generate a 10 us HIGH level pulse using an IT    *
   * from TIM7, configurated with to generate a 10 us pulse.                              *
   **************************************************************************************** */
void GPIO_HCSR04 (void){
  
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  /* Trigger Pulse Initialization */
  /* This pin will go HIGH level for at least 10 us */
  
  /* PB2 Configuration */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
  /*Echo Pulse Initialization */
  /*This pin will receive the bounced wavelength */
  
  /* PB6 Configuration as AF */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void IC_TIM4_Initialization (void){
  
  TIM_IC_InitTypeDef sConfigIC;
  
  /* TIM4 Configuration as IC */
  __HAL_RCC_TIM4_CLK_ENABLE();
  
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 83;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0xFFFF;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  
  HAL_TIM_IC_Init(&htim4);
  
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0x0;
  
  HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1);
  HAL_TIM_IC_Start(&htim4, TIM_CHANNEL_1);
}