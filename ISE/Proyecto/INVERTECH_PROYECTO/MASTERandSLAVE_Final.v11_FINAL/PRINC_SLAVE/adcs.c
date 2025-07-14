#include "slave.h"

#define RESOLUTION_12B 4096U
#define VREF 3.3f

/* ADC */
ADC_HandleTypeDef adchandle_adcs; //handler definition

/* Thread */
osThreadId_t tid_ThADCs;                        // thread id

/* Queue */
osMessageQueueId_t mid_MsgQueueADCs;
MSGQUEUE_ADCs_t tx_ADCs;

int muestras;
  uint16_t Vrtd;
  uint16_t Vcons;

/* Functions prototypes */
int Queue_RTD (void);
int Queue_CONSUMPTION (void);
void ThADCs (void *argument);                   // thread function
void RTD_pin_F429ZI_config();
void CONSUMPTION_pin_F429ZI_config();
void filtro_adc (uint16_t rtd, uint16_t cons, int *i);
int ADCs_Init_Single_Conversion(ADC_HandleTypeDef *, ADC_TypeDef  *);
uint32_t ADCs_getVoltage(ADC_HandleTypeDef * , uint32_t );

int Init_ThADCs (void) {
 
  tid_ThADCs = osThreadNew(ThADCs, NULL, NULL);
  if (tid_ThADCs == NULL) {
    return(-1);
  }
  
  mid_MsgQueueADCs = osMessageQueueNew(MSGQUEUE_ADCs, sizeof(MSGQUEUE_ADCs_t), NULL);
	if (mid_MsgQueueADCs == NULL) {
    return(-1);
  }
	
  return(0);
}

void ThADCs (void *argument) {
  
  
	
	RTD_pin_F429ZI_config();
	CONSUMPTION_pin_F429ZI_config();
	ADCs_Init_Single_Conversion(&adchandle_adcs , ADC1); //ADC1 configuration
	
  tx_ADCs.RTD_Vol = 0;
  tx_ADCs.CONSUMPTION_Vol = 0;
  
  while (1) {
    // Insert thread code here...
    Vrtd =(uint16_t) ADCs_getVoltage(&adchandle_adcs , 13); //get values from channel 13->ADC123_IN13
    Vcons = (uint16_t)ADCs_getVoltage(&adchandle_adcs , 10); //get values from channel 13->ADC123_IN13
    filtro_adc(Vrtd, Vcons, &muestras);
    //osMessageQueuePut(mid_MsgQueueADCs, &tx_ADCs, NULL, 0U);
		//muestras++;
		osDelay(250);
		//osThreadYield();                            // suspend thread
  }
}

// Cogemos 5 medidas y hacemos la media de las cinco consiguiendo un promedio
// que se asemeje a la medida real
void filtro_adc (uint16_t rtd, uint16_t cons, int *muestra){
  static uint32_t cons_aux = 0;
  static uint32_t rtd_aux = 0;
	
	cons_aux += cons;
  rtd_aux += rtd;
  (*muestra)++;
  
  if (*muestra == 10){
//    tx_ADCs.RTD_Vol = (uint16_t)(rtd_aux + rtd)/(*muestra);
    tx_ADCs.CONSUMPTION_Vol = (uint16_t)cons_aux/10;
		tx_ADCs.RTD_Vol = (uint16_t)(rtd_aux /10);
    osMessageQueuePut(mid_MsgQueueADCs, &tx_ADCs, NULL, 0U);
    *muestra = 0;
		cons_aux = 0;
    rtd_aux = 0;
  }
  
}

/**
  * @brief config the use of analog inputs ADC123_IN10 and ADC123_IN13 and enable ADC1 clock
  * @param None
  * @retval None
  */
void RTD_pin_F429ZI_config(){
	  GPIO_InitTypeDef GPIO_InitStruct = {0};
	__HAL_RCC_ADC1_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	
	/* PC3 ------> ADC1_IN13 */

  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	
}

void CONSUMPTION_pin_F429ZI_config(){
	  GPIO_InitTypeDef GPIO_InitStruct = {0};
	__HAL_RCC_ADC1_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
		
	/* PC0 ------> ADC1_IN10 */

  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	
}

/**
  * @brief Initialize the ADC to work with single conversions. 12 bits resolution, software start, 1 conversion
  * @param ADC handle
	* @param ADC instance
  * @retval HAL_StatusTypeDef HAL_ADC_Init
  */
int ADCs_Init_Single_Conversion(ADC_HandleTypeDef *hadc, ADC_TypeDef  *ADC_Instance)
{
	
   /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc->Instance = ADC_Instance;
  hadc->Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc->Init.Resolution = ADC_RESOLUTION_12B;
  hadc->Init.ScanConvMode = DISABLE;
  hadc->Init.ContinuousConvMode = DISABLE;
  hadc->Init.DiscontinuousConvMode = DISABLE;
  hadc->Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc->Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc->Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc->Init.NbrOfConversion = 1;
  hadc->Init.DMAContinuousRequests = DISABLE;
	hadc->Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(hadc) != HAL_OK)
  {
    return -1;
  }
 
	return 0;

}

/**
  * @brief Configure a specific channels ang gets the voltage in float type. This funtion calls to  HAL_ADC_PollForConversion that needs HAL_GetTick()
  * @param ADC_HandleTypeDef
	* @param channel number
	* @retval voltage in float (resolution 12 bits and VRFE 3.3
  */
uint32_t ADCs_getVoltage(ADC_HandleTypeDef *hadc, uint32_t Channel)
	{
		ADC_ChannelConfTypeDef sConfig = {0};
		HAL_StatusTypeDef status;

		uint32_t raw = 0;
//		float voltage = 0;
		 /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = Channel;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
  {
    return -1;
  }
		
		HAL_ADC_Start(hadc);
		
		do (
			status = HAL_ADC_PollForConversion(hadc, 0)); //This funtions uses the HAL_GetTick(), then it only can be executed wehn the OS is running
		while(status != HAL_OK);
		
		raw = HAL_ADC_GetValue(hadc);
		
//		voltage = raw*VREF/RESOLUTION_12B; 
		
		return raw;

}
