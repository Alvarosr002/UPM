#ifndef __LEDS_H
#define __LEDS_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void initMBED_leds (void);
void ledsON (float percentage);
void ledsOFF (void);

#endif /* __LEDS_H */