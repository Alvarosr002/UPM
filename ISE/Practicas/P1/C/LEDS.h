#ifndef __LEDS_H
#define __LEDS_H

#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"                          // CMSIS RTOS header file

	void Init_LEDs (void);
	void Encender_Apagar_LEDs(uint8_t LED);
	
#endif /* __LEDS_H */
