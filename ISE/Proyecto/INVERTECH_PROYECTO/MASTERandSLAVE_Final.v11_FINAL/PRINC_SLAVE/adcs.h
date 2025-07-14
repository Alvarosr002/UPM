/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ADCS_H
#define __ADCS_H

/* Includes ------------------------------------------------------------------*/
	
	#include "stm32f4xx_hal.h"
	#include "cmsis_os2.h"                  // ::CMSIS:RTOS2

/* Exported types ------------------------------------------------------------*/

typedef struct{
  uint16_t RTD_Vol;
  uint16_t CONSUMPTION_Vol;
}MSGQUEUE_ADCs_t;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

  #define MSGQUEUE_ADCs 1

/* Exported functions ------------------------------------------------------- */
	
  extern osMessageQueueId_t mid_MsgQueueADCs;

	int Init_ThADCs (void);

#endif /* __ADCS_H */
