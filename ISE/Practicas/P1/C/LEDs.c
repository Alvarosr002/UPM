#include "LEDs.h"

void Init_LEDs(void){
	
	GPIO_InitTypeDef GPIO_InitStruct;
	
	__HAL_RCC_GPIOB_CLK_ENABLE();	// Habilitar el reloj asociado al puerto de los LEDs de la Nucleo. En este caso el reloj del puerto B
	__HAL_RCC_GPIOD_CLK_ENABLE();	// Habilitar el reloj asociado al puerto de los LEDs de la Mbed. En este caso el reloj del puerto D
	
	// Configuracion de los LEDs VERDE|AZUL|ROJO  = PIN 0|7|14 de la Nucleo
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_7 | GPIO_PIN_14;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	// APAGAR LOS LEDS

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
	
	// Configuracion de los LEDs VERDE|AZUL|ROJO de la Mbed
	GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_11 | GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
	
	// APAGAR LOS LEDS

	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);

};

void Encender_Apagar_LEDs(uint8_t LED){
	
	if((LED & 0x00) == 0x00){	// Apagado de todos los LEDs
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
	}
	
	//PB0
	if((LED & 0x01) == 0x01){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);	// Enciendo el LED Verde Nucleo
	}else{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);	// Apago el LED Verde Nucleo
	}
	
	//PB7
	if((LED & 0x02) == 0x02){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);	// Enciendo el LED Azul Nucleo
	}else{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);	// Apago el LED Azul Nucleo
	}
	
	//PB14
	if((LED & 0x04) == 0x04){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);	// Enciendo el LED Rojo Nucleo
	}else{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);	// Apago el LED Rojo Nucleo
	}
	
	//PD11
	if((LED & 0x08) == 0x08){
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);	// Enciendo el LED Verde Mbed
	}else{
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);	// Apago el LED Verde Mbed
	}
	
	//PB0
	if((LED & 0x10) == 0x10){
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);	// Enciendo el LED Azul Mbed
	}else{
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);	// Apago el LED Azul Mbed
	}
	
	//PB0
	if((LED & 0x20) == 0x20){
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);	// Enciendo el LED Rojo Mbed
	}else{
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);	// Apago el LED Rojo Mbed
	}
	
}
