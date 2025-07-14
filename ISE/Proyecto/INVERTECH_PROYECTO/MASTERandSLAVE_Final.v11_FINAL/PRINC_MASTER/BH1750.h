/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BH1750_H
#define __BH1750_H

/* Includes ------------------------------------------------------------------*/
	#include "stm32f4xx_hal.h"
	#include "cmsis_os2.h"                  // ::CMSIS:RTOS2

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

	#define BH1750_ADDR 0x23								// Direccion I2C del BH1750
	#define BH1750_POWER_ON 0x01        		// Comando para encender el sensor
	#define CONTINUOSLY_H_RESOLUTION 0x10		// Comando para establecer el modo de medicion continuo con alta resolucion
	#define CONTINUOSLY_L_RESOLUTION 0x13		// Comando para establecer el modo de medicion continuo con baja resolucion
	#define TRANSF_DONE_BH1750 0x01					// Flag para indicar que la comunicacion se ha realizado correctamente
	#define BRIGHT_MEASURE 0x02							// Flag para indicar que se realice una medida
	
/* Exported functions ------------------------------------------------------- */
	
	void Init_I2C (void);
	void Init_BH1750(void);
	float Brightness_Reading (void);

#endif /* __BH1750_H */
