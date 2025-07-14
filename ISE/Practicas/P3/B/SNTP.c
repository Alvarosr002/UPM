#include "SNTP.h"

netStatus SNTPOK;
struct tm SNTP_Date_Hour;

void init_SNTP(void);
void time_callback(uint32_t seconds, uint32_t seconds_fraction);

void init_SNTP(void){
	
	SNTPOK = netSNTPc_GetTime(NULL, time_callback);
	if(SNTPOK != netOK){
		while(1);
	}
	
}

void time_callback(uint32_t seconds, uint32_t seconds_fraction){
	
	if(seconds != 0){
		
		time_t Segundos_RTC = (time_t) seconds;	// Cambio el formato de la variable que almacena los segundos
																						// transcurridos desde 1970 obtenidos por el SNTP a tipo
																						// time_t, para asi luego pasarselo por parametro a la funcion
																						// localtime
		
		SNTP_Date_Hour = *localtime(&Segundos_RTC);	// Obtenemos los segundos desde 1970 del servidor SNTP
																								// y los almacenamos en la estructura
		
		/*##-1- Configure the Date of the SNTP #################################################*/
		sdatestructure.Year = SNTP_Date_Hour.tm_year - 100;	// Restamos 100 al valor del año, ya que
																												// en la estructura tm el valor de los años
																												// se represetna desde 1900, y luego en la funcion
																												// RTC_Hora_Fecha sumaremos 2000 a dicho valor
		
		sdatestructure.Month = SNTP_Date_Hour.tm_mon + 1;	// Sumamos 1 al valor de la estructura, ya que
																											// enero corresponde al valor 0 y diciembre al 11
		
		sdatestructure.Date = SNTP_Date_Hour.tm_mday;	// Valor del dia a configurar en formato BCD
		sdatestructure.WeekDay = SNTP_Date_Hour.tm_wday;	// Macro en la capa HALL que indica que es martes
		
		if(HAL_RTC_SetDate(&RtcHandle,&sdatestructure,RTC_FORMAT_BIN) != HAL_OK)	// Se configura la fecha empeando la funcion HAL_RTC_SetDate(), donde en el tercer parametro especificamos que los
																																							// valores se encuentran en BCD
		{
    /* Initialization Error */
			while(1);	// Error en la configuracion
		}
		
		/*##-2- Configure the Time of the SNTP #################################################*/
		
		stimestructure.Hours = (SNTP_Date_Hour.tm_hour + 1) % 24;	// Establece el valor de las horas del SNTP
		stimestructure.Minutes = SNTP_Date_Hour.tm_min;	// Establece el valor de los minutos del SNTP
		stimestructure.Seconds = SNTP_Date_Hour.tm_sec;	// Establece el valor de los segundos del SNTP
		stimestructure.TimeFormat = RTC_HOURFORMAT_24;	// Indica que el formato es 24 horas
		stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE ;	// Desactiva el ajuste automatico de horario de verano
		stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;	// No se almacenainformacion adicional en la memoria del RTC

		if (HAL_RTC_SetTime(&RtcHandle, &stimestructure, RTC_FORMAT_BIN) != HAL_OK)	// Establece la hora en el RTC empleando la funcion HAL_RTC_SetTime()
		{
			/* Initialization Error */
			while(1);	// Error en la configuracion
		}
		
		/*##-3- Writes a data in a RTC Backup data Register1 #######################*/
		HAL_RTCEx_BKUPWrite(&RtcHandle, RTC_BKP_DR1, 0x32F2); // Escribe en el resgitro de BackUp el valor 0x32F2 = 0011 0010 1111 0010
		
	}
		
}

void Pulsador_B1(void){
	
	GPIO_InitTypeDef GPIO_InitStruct;
	
	__HAL_RCC_GPIOC_CLK_ENABLE();	// Habilitamos el reloj del puerto asociado al pulsador. Puerto C
	
	// Configuracion del Pulsador de usuario B1
	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);	// Habilitamos las interrupciones de todos los pines del 10 al 15 de todos los puertos
	
};
