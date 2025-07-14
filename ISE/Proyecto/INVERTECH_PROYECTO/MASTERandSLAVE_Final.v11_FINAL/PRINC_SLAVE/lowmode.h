#ifndef __LOWMODE_H
#define __LOWMODE_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"                          // CMSIS RTOS header file
#include "stdio.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
  /* Exported thread functions,  
  Example: extern void app_main (void *arg); */
  
  void Enter_SleepMode (void);
  void Init_PB1_WAKEUP (void);

#endif /* __LOWMODE_H */
