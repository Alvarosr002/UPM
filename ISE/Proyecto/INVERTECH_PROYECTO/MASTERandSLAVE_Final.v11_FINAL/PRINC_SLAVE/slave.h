#ifndef __SLAVE_H
#define __SLAVE_H

/* Includes ------------------------------------------------------------------*/

  #include "stm32f4xx_hal.h"
  #include "cmsis_os2.h"                          // CMSIS RTOS header file
  #include "hcsr04.h"
  #include "bh1750.h"
  #include "adcs.h"
  #include "bme680.h"
  #include "com.h"
  #include "lowmode.h"
//  #include "sensor.h"
  
/* Exported types ------------------------------------------------------------*/

  typedef struct{
    uint32_t Voltaje_RTD;
    float humidity;
    float luminosity;
    int air_quality;
    float aquaLevel;
    uint32_t Voltaje_CONSUMPTION;
  } MSGQUEUE_OBJ_MEAS;
  
  extern MSGQUEUE_OBJ_MEAS tx_medidas;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
  
  #define SLAVE_COM_MSG_QUEUE 10
  
/* Exported functions ------------------------------------------------------- */

  int Init_Slave (void);
  void Init_LEDs(void);

#endif /* __SLAVE_H */