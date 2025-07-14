#ifndef __RTC_H
#define __RTC_H

	#include "stm32f4xx_hal.h"
	#include "cmsis_os2.h"                  // ::CMSIS:RTOS2
	#include "stdbool.h"
	#include "stdio.h"
	
	/* Defines related to Clock configuration */
	#define RTC_ASYNCH_PREDIV  0x7F   /* LSE as RTC clock */
	#define RTC_SYNCH_PREDIV   0x00FF /* LSE as RTC clock */
	
	extern volatile bool Parpadear_LED ;
	
	extern RTC_HandleTypeDef RtcHandle;
	
	void RTC_Config(void);
	void RTC_Hora_Fecha(uint8_t *showtime, uint8_t *showdate);
	void RTC_Alarm_Config(void);
	
#endif /* __RTC_H */
