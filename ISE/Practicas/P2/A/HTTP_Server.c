/*------------------------------------------------------------------------------
 * MDK Middleware - Component ::Network
 * Copyright (c) 2004-2019 Arm Limited (or its affiliates). All rights reserved.
 *------------------------------------------------------------------------------
 * Name:    HTTP_Server.c
 * Purpose: HTTP Server example
 *----------------------------------------------------------------------------*/

#include <stdio.h>

#include "main.h"

#include "rl_net.h"                     // Keil.MDK-Pro::Network:CORE

#include "stm32f4xx_hal.h"              // Keil::Device:STM32Cube HAL:Common
//#include "Board_LED.h"                  // ::Board Support:LED
#include "LEDs.h"                  			// ::Board Support:LED
#include "Board_Buttons.h"              // ::Board Support:Buttons
//#include "Board_ADC.h"                  // ::Board Support:A/D Converter
#include "adc.h"
//#include "Board_GLCD.h"                 // ::Board Support:Graphic LCD
#include "Thlcd.h"                      // LCD
//#include "GLCD_Config.h"                // Keil.MCBSTM32F400::Board Support:Graphic LCD
#include "RTC.h"

// Main stack size must be multiple of 8 Bytes
#define APP_MAIN_STK_SZ (1024U)
uint64_t app_main_stk[APP_MAIN_STK_SZ / 8];
const osThreadAttr_t app_main_attr = {
  .stack_mem  = &app_main_stk[0],
  .stack_size = sizeof(app_main_stk)
};

//extern GLCD_FONT GLCD_Font_6x8;
//extern GLCD_FONT GLCD_Font_16x24;

extern uint16_t AD_in          (uint32_t ch);
extern uint8_t  get_button     (void);
extern void     netDHCP_Notify (uint32_t if_num, uint8_t option, const uint8_t *val, uint32_t len);

extern bool LEDrun;
extern char lcd_text[2][20+1];

extern osThreadId_t TID_Display;
extern osThreadId_t TID_Led;
extern osThreadId_t TID_RTC;
extern osThreadId_t TID_ALARM;

bool LEDrun;
char lcd_text[2][20+1] = { "LCD line 1",
                           "LCD line 2" };

ADC_HandleTypeDef adchandle; //handler definition
													 
/* Thread IDs */
osThreadId_t TID_Display;
osThreadId_t TID_Led;
osThreadId_t TID_RTC;
osThreadId_t TID_ALARM;

/* Thread declarations */
static void BlinkLed (void *arg);
static void Display  (void *arg);
static void Date_Time_RTC (void *arg);
static void Alarm(void *arg);

/* Buffers used for displaying Time and Date */
uint8_t aShowTime[10] = {0};
uint8_t aShowDate[10] = {0};
													 
__NO_RETURN void app_main (void *arg);

/* Read analog inputs */
uint16_t AD_in (uint32_t ch) {
  int32_t val = 0;

  if (ch == 0) {
		val = ADC_getVoltage(&adchandle , 13 ); //get values from channel 13->ADC123_IN13
  }
  return ((uint16_t)val);
}

/* Read digital inputs */
uint8_t get_button (void) {
  return ((uint8_t)Buttons_GetState ());
}

/* IP address change notification */
void netDHCP_Notify (uint32_t if_num, uint8_t option, const uint8_t *val, uint32_t len) {

  (void)if_num;
  (void)val;
  (void)len;

  if (option == NET_DHCP_OPTION_IP_ADDRESS) {
    /* IP address change, trigger LCD update */
    osThreadFlagsSet (TID_Display, 0x01);
  }
}

/*----------------------------------------------------------------------------
  Thread 'ALARM': Funcion para gestionar los booleanos de las alarmas
 *---------------------------------------------------------------------------*/

static __NO_RETURN void Alarm(void *arg){
	
	(void)arg;
	
	while(1){
		
		if(Parpadear_LED == true){
			Parpadear_LED = false;
			
			for(uint8_t i = 0; i < 5; i++){
				
				HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
				osDelay(1000);
				
			}
		}
	}
}

/*----------------------------------------------------------------------------
  Thread 'RTC': Funcion para obtener dia y fecha del RTC
 *---------------------------------------------------------------------------*/
static __NO_RETURN void Date_Time_RTC (void *arg) {
	
	(void)arg;
	
	while(1){
		RTC_Hora_Fecha(aShowTime, aShowDate);
		
		// Copiar el contenido de aShowTime en lcd_text[0], para poder tenerlo asi en un array de caracteres
    memcpy(lcd_text[0], aShowTime, sizeof(aShowTime));
    lcd_text[0][sizeof(aShowTime)] = '\0';  // Asegurar terminación
		
    // Copiar el contenido de aShowDate en lcd_text[1]
    memcpy(lcd_text[1], aShowDate, sizeof(aShowDate));
    lcd_text[1][sizeof(aShowDate)] = '\0';  // Asegurar terminación
		
		osThreadFlagsSet (TID_Display, 0x01);			// Envio flag al LCD de que se quiere escribir algo
	}
	
}


/*----------------------------------------------------------------------------
  Thread 'Display': LCD display handler
 *---------------------------------------------------------------------------*/
static __NO_RETURN void Display (void *arg) {

  (void)arg;
  
	//Inicializacion del LCD
  Init_PinesGPIO();
  LCD_Reset();
  LCD_Init();
	cleanLCD();
  LCD_Update();

  while(1) {
    /* Wait for signal from DHCP */
    osThreadFlagsWait (0x01U, osFlagsWaitAll, osWaitForever);
		
		cleanLCD();

    /* Display user text line 1 */
		write_lcd(lcd_text[0],1, strlen(lcd_text[0]));
		/* Display user text line 2 */
		write_lcd(lcd_text[1],2, strlen(lcd_text[1]));
		
//    LCD_Update();
    
  }
}

/*----------------------------------------------------------------------------
  Thread 'BlinkLed': Blink the LEDs on an eval board
 *---------------------------------------------------------------------------*/
static __NO_RETURN void BlinkLed (void *arg) {	// Función que hace que los leds vayan parpadeando
  const uint8_t led_val[16] = { 0x48,0x88,0x84,0x44,0x42,0x22,0x21,0x11,
                                0x12,0x0A,0x0C,0x14,0x18,0x28,0x30,0x50 };
  uint32_t cnt = 0U;

  (void)arg;

  LEDrun = false;
  while(1) {
    /* Every 100 ms */
    if (LEDrun == true) {
//      LED_SetOut (led_val[cnt]);
			Encender_Apagar_LEDs(led_val[cnt]);
      if (++cnt >= sizeof(led_val)) {
        cnt = 0U;
      }
    }
    osDelay (100);
  }
}

/*----------------------------------------------------------------------------
  Main Thread 'main': Run Network
 *---------------------------------------------------------------------------*/
__NO_RETURN void app_main (void *arg) {
  (void)arg;

	// Inicializacion de los LEDs
  Init_LEDs();
	
	// Inicializacion de los ADCs
	ADC1_pins_F429ZI_config(); //specific PINS configuration
	ADC_Init_Single_Conversion(&adchandle , ADC1); //ADC1 configuration
	
	// Inicializacion del RTC
	RTC_Config();
	
  netInitialize ();			// Inicializa un monton de cosas del Ethernet

  TID_Led     = osThreadNew (BlinkLed, NULL, NULL);
  TID_Display = osThreadNew (Display,  NULL, NULL);
	TID_RTC     = osThreadNew (Date_Time_RTC, NULL, NULL);
	TID_ALARM   = osThreadNew (Alarm, NULL, NULL);
	
  osThreadExit();
}
