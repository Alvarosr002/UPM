#ifndef __HCSR04_H
#define __HCSR04_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"                          // CMSIS RTOS header file
#include "hcsr04.h"
#include "stdio.h"

/* Exported types ------------------------------------------------------------*/

typedef struct{
  float quantity;
}MSGQUEUE_HCSR04_t;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

  #define MSGQUEUE_HCSR04 1

/* Exported functions ------------------------------------------------------- */
  
  extern osMessageQueueId_t mid_MsgQueueHCSR04;
  
  int Init_HCSR04 (void);

#endif /* __HCSR04_H */