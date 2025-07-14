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
   * the pin is at HIGH level is the time the burst is travelling.                        *
   **************************************************************************************** */
   
  
#include "hcsr04.h"

TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim4;

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
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

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

/* Cambiar dependiendo de la altura del depósito */
#define ALTURA_DEPO_CM 11.8f 

void Init_HCSR04 (void);
void DWT_Delay_Init(void);
void DWT_Delay_us(uint32_t us);
float getMeasure (void);

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

/*Here we will generate the Trigger pulse every time
  we want to do a measure. This pulse has to be, at least,
  10 us long. Less than that time we won't get a
  response from the sensor.*/
void Init_HCSR04 (void){
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
  DWT_Delay_us (2); //Wait for 2 us
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
  DWT_Delay_us (10); //Wait for 10 us
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
}

/*This both functions are unique from this module. They are designed to
  generate/implement high-precision delays using the Data Watchpoint
  and Trace (DWT), specifically for microsecond delays. We cannot use 
  osDelay() due to it receives arguments in ms, and it is impossible
  to introduce decimal numbers as arguments.*/

/*Initializes the DWT cycle counter to measure delays based on CPU cycles.*/
void DWT_Delay_Init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; //Enable DWT access setting bit TRCENA in register DEMCR
    DWT->CYCCNT = 0;  // Resets the cycle counter (CYCCNT) of the DWT to zero
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; // Enable the cycle counter (CYCCNT) to start counting
}

/*It generates a delay in microseconds (µs) with high precision.*/
void DWT_Delay_us(uint32_t us) {
    uint32_t start = DWT->CYCCNT; //Saves the current value of the cycle counter (CYCCNT) as the starting point.
    uint32_t ticks = (HAL_RCC_GetHCLKFreq() / 1000000) * us; // Calculates the number of CPU cycles required for the desired delay
    while ((DWT->CYCCNT - start) < ticks); // Wait in a loop until the cycle counter (CYCCNT) has incremented the calculated amount (ticks).
}

float getMeasure (void){
  
  uint32_t start = 0; uint32_t end = 0;
  uint32_t timeout = SystemCoreClock/1000;
  float width = 0.0; float distance = 0.0;
  float waterLevel = 0.0;
  
  //Reset TIM4
  __HAL_TIM_SET_COUNTER(&htim4, 0);
  HAL_TIM_Base_Start(&htim4);
  
  /* Generate a 10us pulse -> TRIGGER*/
  Init_HCSR04 ();
  
  /* Wait for rising edge */
  while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_RESET){
    if (__HAL_TIM_GET_COUNTER(&htim4) > timeout){
      return -1.0f;
    }
  }
  
  start = __HAL_TIM_GET_COUNTER(&htim4);
  
  /* Wait for falling edge */
  while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_SET){
    if (__HAL_TIM_GET_COUNTER(&htim4) > (start + timeout)){
      HAL_TIM_Base_Stop(&htim4);
      return -1.0f;
    }
  }
  
  end = __HAL_TIM_GET_COUNTER(&htim4);
  HAL_TIM_Base_Stop(&htim4);
  
  /* Get distance*/
  if(end > start){
    width = (end - start);
    distance = width / 58.0f;
    
    /* Wait to calculate the height of the tank */
    waterLevel = ALTURA_DEPO_CM - distance;
    
    if(waterLevel < 0.0f){
      return 0.0f; //Empty tank
    }else if (waterLevel > ALTURA_DEPO_CM){
      return 100.0f; //Full tank (saturated)
    }else{
      return (waterLevel / ALTURA_DEPO_CM) * 100.0f; // Percentage
    }
    
  }else{
    return -1.0f;
  }
}

/* This function initialize the MBED leds*/
void initMBED_leds (void){
  GPIO_InitTypeDef GPIO_InitStruct; 
  
  /*Enable clock to GPIOD*/
  __HAL_RCC_GPIOD_CLK_ENABLE();
  
  /*Set GPIOD pin */
  GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;  

  /*Init GPIOB Pins*/
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
  
  /* Common cathode. They're off with '1'*/
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);  //VERDE
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);  //AZUL
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET); //ROJO
}

/* This function will set a color depending on the quantity*/
void ledsON (float percentage){
  
  if(percentage >= 0.0f && percentage <= 20.0f){ //RED
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET); //VERDE
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET); //AZUL
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET); //ROJO
  }
  
  if(percentage > 20.0f && percentage <=  70.0f){ //YELLOW
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET); //VERDE
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET); //AZUL
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET); //ROJO
  }
  
  if(percentage > 70.0f && percentage <= 99.9f){ //GREEN
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET); //VERDE
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET); //AZUL
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET); //ROJO
  }
}
