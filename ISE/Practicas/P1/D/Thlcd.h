#ifndef __THLCD_H
#define __THLCD_H

	#include "stm32f4xx_hal.h"
	#include "Driver_SPI.h"
	#include "stdio.h"
	#include "string.h"
	#include "cmsis_os2.h"                          // CMSIS RTOS header file
	
	#define S_TRANS_DONE_SPI  0x01
	
  void LCD_Init (void);
  void LCD_Update (void);
  void LCD_Reset (void);
  void Init_PinesGPIO (void);
	void cleanLCD(void);
	void cleanLine (void);
	void write_lcd(char cadena[], int linea, int longitudCadena);

//	int Init_ThLCD (void);

#endif /* __THLCD_H */
