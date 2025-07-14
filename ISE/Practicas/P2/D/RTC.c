#include "RTC.h"

/* RTC handler declaration */
RTC_HandleTypeDef RtcHandle;

RTC_DateTypeDef sdatestructure;	// Declaracion de la estructura empleada para configurar la fecha del RTC
RTC_TimeTypeDef stimestructure;	// Declaracion de la estructura empleada para configurar la hora del RTC
RTC_AlarmTypeDef alarmstrcut;		// Declaracion de la esctructura de la alarma

/* RTC variables declaration */

//Fecha
uint8_t RTC_Anno = 0x25;
uint8_t RTC_Mes = RTC_MONTH_MARCH;
uint8_t RTC_Num_Dia = 0x05;
uint8_t RTC_Dia_Semana = RTC_WEEKDAY_MONDAY;

//Hora
uint8_t RTC_Hora = 0x12;
uint8_t RTC_Min = 0x59;
uint8_t RTC_Seg = 0x00;

/* Private function prototypes -----------------------------------------------*/
void RTC_Config(void);
void RTC_DateConfig(uint8_t RTC_Anno, uint8_t RTC_Mes, uint8_t RTC_Num_Dia, uint8_t RTC_Dia_Semana);
void RTC_TimeConfig(uint8_t RTC_Hora, uint8_t RTC_Min, uint8_t RTC_Seg);
void RTC_Hora_Fecha(uint8_t *showtime, uint8_t *showdate);
void RTC_Alarm_Config(void);
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc);

void RTC_Config(void){
	
	// Habilitar LSE y esperar a que esté listo
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE;
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
    while(1);	// Error en la configuracion
	}

	// Seleccionar LSE como fuente de reloj para el RTC
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
	PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
	{
		while(1);	// Error en la configuracion
	}

	// Habilitar el RTC
	__HAL_RCC_RTC_ENABLE();
		
	/*##-1- Configure the RTC peripheral #######################################*/
  /* Configure RTC prescaler and RTC data registers */
  /* RTC configured as follows:
      - Hour Format    = Format 24
      - Asynch Prediv  = Value according to source clock
      - Synch Prediv   = Value according to source clock
      - OutPut         = Output Disable
      - OutPutPolarity = High Polarity
      - OutPutType     = Open Drain */
	
  RtcHandle.Instance = RTC;	// Asignacion del periferico RTC al manejador RtcHandle
  RtcHandle.Init.HourFormat = RTC_HOURFORMAT_24;	// Configuracion en el formato 24 h
  RtcHandle.Init.AsynchPrediv = RTC_ASYNCH_PREDIV;	// Se establece el valor del prescaler asincrono para que el RTC cuente correctamente el tiempo en base a la frecuencia del reloj de entrada
  RtcHandle.Init.SynchPrediv = RTC_SYNCH_PREDIV;	// Se establece el valor del prescaler sincrono para que el RTC cuente correctamente el tiempo en base a la frecuencia del reloj de entrada
  RtcHandle.Init.OutPut = RTC_OUTPUT_DISABLE;	// Deshabilitacion de la señal de salida del RTC
  RtcHandle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;	// Configuracion de la polaridad de la slaida a nivel alto
  RtcHandle.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;	// Salida en Open Drain, lo que significa que el pin puede drenar corriente a tierra y necesita resistencia para llevarlo a nivel alto
  __HAL_RTC_RESET_HANDLE_STATE(&RtcHandle);	// Reseteo del estado del manejador del RTC, asegurando un estado limpio antes de inicializarlo
  if (HAL_RTC_Init(&RtcHandle) != HAL_OK)	// Se lama a la funcion HAL_RTC_Init(), que configura al RTC con los parametros definidos anteriormente	
  {
    /* Initialization Error */
		while(1);	// Error en la configuracion
  }

  /*##-2- Check if Data stored in BackUp register1: No Need to reconfigure RTC#*/
  /* Read the Back Up Register 1 Data */
  if (HAL_RTCEx_BKUPRead(&RtcHandle, RTC_BKP_DR1) != 0x32F2)	// Comprueban si el RTC ha sido configurado previamente o ha perdido dicha configuracion por reinicio o corte de alimentacion
																															// y si es asi, lo vuelve a configurar
  {
    /* Configure RTC Calendar */
//		RTC_DateConfig(RTC_Anno, RTC_Mes, RTC_Num_Dia, RTC_Dia_Semana);
//		RTC_TimeConfig(RTC_Hora, RTC_Min, RTC_Seg);
//		RTC_Alarm_Config();
  }
  else
  {
    /* Clear source Reset Flag */
    __HAL_RCC_CLEAR_RESET_FLAGS();	// 
  }
	
}

void RTC_DateConfig(uint8_t RTC_Anno, uint8_t RTC_Mes, uint8_t RTC_Num_Dia, uint8_t RTC_Dia_Semana){
	
	/*##-1- Configure the Date #################################################*/
  /* Set Date: Tuesday February 18th 2014 */
  sdatestructure.Year = RTC_Anno;	// Valor del año a configurar en formato BCD
  sdatestructure.Month = RTC_Mes;	// Macro definida en la HALL que indica el mes de febrero
  sdatestructure.Date = RTC_Num_Dia;	// Valor del dia a configurar en formato BCD
  sdatestructure.WeekDay = RTC_Dia_Semana;	// Macro en la capa HALL que indica que es martes
  
  if(HAL_RTC_SetDate(&RtcHandle,&sdatestructure,RTC_FORMAT_BCD) != HAL_OK)	// Se configura la fecha empeando la funcion HAL_RTC_SetDate(), donde en el tercer parametro especificamos que los
																																						// valores se encuentran en BCD
  {
    /* Initialization Error */
		while(1);	// Error en la configuracion
  }
	
}

void RTC_TimeConfig(uint8_t RTC_Hora, uint8_t RTC_Min, uint8_t RTC_Seg){
	
	/*##-2- Configure the Time #################################################*/
  /* Set Time: 02:00:00 */
  stimestructure.Hours = RTC_Hora;	// Establece el valor de las horas en formato BCD
  stimestructure.Minutes = RTC_Min;	// Establece el valor de los minutos en formato BCD
  stimestructure.Seconds = RTC_Seg;	// Establece el valor de los segundos en formato BCD
  stimestructure.TimeFormat = RTC_HOURFORMAT_24;	// Indica que el formato es 24 horas
  stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE ;	// Desactiva el ajuste automatico de horario de verano
  stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;	// No se almacenainformacion adicional en la memoria del RTC

  if (HAL_RTC_SetTime(&RtcHandle, &stimestructure, RTC_FORMAT_BCD) != HAL_OK)	// Establece la hora en el RTC empleando la funcion HAL_RTC_SetTime()
  {
    /* Initialization Error */
		while(1);	// Error en la configuracion
  }
	
	/*##-3- Writes a data in a RTC Backup data Register1 #######################*/
  HAL_RTCEx_BKUPWrite(&RtcHandle, RTC_BKP_DR1, 0x32F2); // Escribe en el resgitro de BackUp el valor 0x32F2 = 0011 0010 1111 0010
	
}

/**
  * @brief  Display the current time and date.
  * @param  showtime : pointer to buffer
  * @param  showdate : pointer to buffer
  * @retval None
  */

void RTC_Hora_Fecha(uint8_t *showtime, uint8_t *showdate)
{

  /* Get the RTC current Time */
  HAL_RTC_GetTime(&RtcHandle, &stimestructure, RTC_FORMAT_BIN);	// Recoge la informacion relacioanda con la hora actual del RTC
  /* Get the RTC current Date */
  HAL_RTC_GetDate(&RtcHandle, &sdatestructure, RTC_FORMAT_BIN);	// Recoge la informacion relacionada con la fecha actual del RTC
  /* Display time Format : hh:mm:ss */
  sprintf((char *)showtime, "%02d:%02d:%02d", stimestructure.Hours, stimestructure.Minutes, stimestructure.Seconds);
  /* Display date Format : mm-dd-yy */
  sprintf((char *)showdate, "%02d-%02d-%02d", sdatestructure.Date, sdatestructure.Month, 2000 + sdatestructure.Year);
}

void RTC_Alarm_Config(void){
	
	HAL_RTC_GetTime(&RtcHandle, &stimestructure, RTC_FORMAT_BIN);	// Recoge la informacion relacioanda con la hora actual del RTC
	
	alarmstrcut.AlarmTime.Hours = stimestructure.Hours;	// Configuro la hora a la que va a sonar la alarma, igualandola a la hora del RTC
	alarmstrcut.AlarmTime.Minutes = stimestructure.Minutes;	// Configuro el minuto en el que va a sonar la alarma,
																																			// que corresponderá al resto de dividir el minuto actual del RTC +1 entre 60, ya que si solo ponemos
																																			// stimestructure.Minutes + 1, cuando nos encontremos en el minuto 59 configuraremos la alarma para que sale en el minuto 60
																																			// que será un caso que nunca cumpliremos.
	
	alarmstrcut.AlarmTime.Seconds = (stimestructure.Seconds + 15) % 60;	// Configuro el segundo en el que quiero que salte la alarma, que será cuando al comienzo de un minuto nuevo
	alarmstrcut.AlarmTime.TimeFormat = RTC_HOURFORMAT_24;	// Indica que el formato es 24 horas
	alarmstrcut.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE ;	// Desactiva el ajuste automatico de horario de verano
  alarmstrcut.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;	// No se almacenainformacion adicional en la memoria del RTC
	
	alarmstrcut.AlarmMask = RTC_ALARMMASK_HOURS 
												| RTC_ALARMMASK_MINUTES 
												| RTC_ALARMMASK_DATEWEEKDAY;	// Si pones RTC_ALARMMASK_ALL, se cuelga el programa.
																											// En este parametro de la alarma, le indicaremos todo aquello que no queremos tener en cuenta a la hora de determinar cuando ha de saltar
																											// esta, por lo que lo enmascararemos.
	
	alarmstrcut.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;	// Configuro la alarma para que no tenga en cuenta los subsegundos y se active cuando coincidan horas, minutos y segundos. 
	alarmstrcut.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  alarmstrcut.AlarmDateWeekDay    = 1; // se ignora con la Mask
	alarmstrcut.Alarm = RTC_ALARM_A;
	
	// Configurar la alarma con interrupción
  HAL_RTC_SetAlarm_IT(&RtcHandle, &alarmstrcut, RTC_FORMAT_BIN);
	
	// Habilitamos la interrupcion de la alarma A y B
	HAL_NVIC_EnableIRQ(RTC_Alarm_IRQn);
	
}
