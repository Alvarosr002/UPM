#include <stdio.h>

#include "main.h"

#include "rl_net.h"                     // Keil.MDK-Pro::Network:CORE

#include "stm32f4xx_hal.h"              // Keil::Device:STM32Cube HAL:Common
#include "LEDs.h"                  			// ::Board Support:LED
#include "Board_Buttons.h"              // ::Board Support:Buttons
#include "adc.h"
#include "Thlcd.h"                      // LCD
#include "hcsr04.h"
#include "Principal_Master.h"
#include "AT24C256.h"
#include "Thlcd.h"
#include "SNTP.h"


// Main stack size must be multiple of 8 Bytes
#define APP_MAIN_STK_SZ (1024U)
uint64_t app_main_stk[APP_MAIN_STK_SZ / 8];
const osThreadAttr_t app_main_attr = {
  .stack_mem  = &app_main_stk[0],
  .stack_size = sizeof(app_main_stk)
};

extern uint16_t AD_in          (uint32_t ch);
extern uint8_t  get_button     (void);
extern void     netDHCP_Notify (uint32_t if_num, uint8_t option, const uint8_t *val, uint32_t len);

extern bool LEDrun;

extern MEDIDAS_t meas_global;


extern osThreadId_t TID_Led;
extern osThreadId_t TID_PRINC_MASTER;

extern void UartTx  (void *arg);
extern void UartRx  (void *arg);	

extern void despertar_Slave(void);
extern void guardar_Memory(void);

bool LEDrun;
													 
MEDIDAS_t meas_global;
													 

/* Thread IDs */
osThreadId_t TID_RTC;
osThreadId_t TID_Alarm;
osThreadId_t TID_PRINC_MASTER;
osThreadId_t TID_Display;
extern osThreadId_t TID_UartTx;
extern osThreadId_t TID_UartRx;

uint32_t flagAlarma = 0x0000;

__NO_RETURN void app_main (void *arg);

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
  }
}

static __NO_RETURN void Alarma (void *arg){
	
	(void)arg;

/*----------------------------------------------------------------------------
  Alarm Thread : Alarm thread
 *---------------------------------------------------------------------------*/  
	
	while(1){
    // Insert thread code here.
    if(osThreadFlagsWait (0x01U, osFlagsWaitAll, 0) == 0x01){
      RTC_Alarm_Config();
    }   
    
    osDelay(1000);
    
  }
	
}

/*----------------------------------------------------------------------------
  Main Thread 'main': Run Network
 *---------------------------------------------------------------------------*/
__NO_RETURN void principal_Master (void *arg) {
  (void)arg;
	while(1){
		
    automata_PrincipalMaster();
	}
}
/*----------------------------------------------------------------------------
  Main Thread 'main': Run Network
 *---------------------------------------------------------------------------*/
__NO_RETURN void app_main (void *arg) {
  (void)arg;
  netInitialize ();			// Inicializa un monton de cosas del Ethernet
	TID_PRINC_MASTER = osThreadNew(principal_Master, NULL, NULL);
	Init_LCD ();
  
//  TID_Alarm = osThreadNew (Alarma,  NULL, NULL);
  TID_UartTx     = osThreadNew (UartTx, NULL, NULL);
	TID_UartRx     = osThreadNew (UartRx, NULL, NULL);
  osThreadExit();
}
