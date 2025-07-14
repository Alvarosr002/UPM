#ifndef __THLCD_H
#define __THLCD_H

#include "Driver_SPI.h"
#include "stdio.h"
#include "string.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"
#include "rfid.h"
#include "Principal_Master.h"

#define S_TRANS_DONE_SPI  0x01

int Init_LCD (void);

extern void LCD_Update (void);
extern void LCD_Clean (void);
extern void LCD_Initialize (void);
extern void escrituraLCD (uint8_t linea, const char frase[20]);
extern void escrituraLCD_V2 (uint8_t linea, char *texto);
extern void limpiar_L1 (void);
extern void limpiar_L2 (void);

#endif /* __THLCD_H */
