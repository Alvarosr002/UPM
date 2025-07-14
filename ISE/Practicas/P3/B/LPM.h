#ifndef __LPM_H
#define __LPM_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "LEDs.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

#define SLEEP_MODE
#define LAN8742A_PHY_ADDRESS            0x00U

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void SleepMode_Measure(void);

#endif /* __LPM_H */
