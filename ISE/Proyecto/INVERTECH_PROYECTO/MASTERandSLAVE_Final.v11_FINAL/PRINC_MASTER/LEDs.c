#include "LEDS.h"

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
	
	
	__HAL_RCC_GPIOB_CLK_ENABLE();	// Habilitar el reloj asociado al puerto de los LEDs. En este caso el reloj del puerto B
	
	// Configuracion de los LEDs VERDE|AZUL|ROJO  = PIN 0|7|14
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_7 | GPIO_PIN_14;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
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

void ledsOFF (void){
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);  //VERDE
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);  //AZUL
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET); //ROJO
}

