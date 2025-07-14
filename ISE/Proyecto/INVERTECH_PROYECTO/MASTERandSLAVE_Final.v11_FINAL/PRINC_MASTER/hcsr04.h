#ifndef __HCSR04_H
#define __HCSR04_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void GPIO_HCSR04 (void);
void IC_TIM4_Initialization (void);
float getMeasure (void);
void initMBED_leds (void);
void ledsON (float percentage);

#endif /* __HCSR04_H */
