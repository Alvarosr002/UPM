#include "speaker.h"
 
/*----------------------------------------------------------------------------
 *      Thread 1 'Thread_Name': Sample thread
 *---------------------------------------------------------------------------*/
 
osThreadId_t tid_speaker;                        // thread id
extern osThreadId_t tid_speaker;                        // thread id

TIM_HandleTypeDef htim2;
TIM_OC_InitTypeDef TIM_Channel_InitStruct;

void speaker (void *argument);                   // thread function
void initTimer2_OC (void);
static void correctID (void);
static void wrongID (void);

int Init_Altavoz (void) {
 
  tid_speaker = osThreadNew(speaker, NULL, NULL);
  if (tid_speaker == NULL) {
    return(-1);
  }
 
  return(0);
}
 
void speaker (void *argument) {
  
  uint32_t spkFlags = 0x0000;
  initTimer2_OC ();
 
  while (1) {
    // Insert thread code here...

    spkFlags = osThreadFlagsWait(SPK_ON | SPK_OFF, osFlagsWaitAny, osWaitForever);
    
    if(spkFlags == SPK_ON){
      correctID ();
    }else if(spkFlags == SPK_OFF){
      wrongID ();
    }
    //osThreadYield();                            // suspend thread
  }
}

void initTimer2_OC (void){
  
  GPIO_InitTypeDef GPIO_InitStruct;
  
  //Timer normal en FA
  __HAL_RCC_GPIOA_CLK_ENABLE ();
  
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  //Timer OC
  __HAL_RCC_TIM2_CLK_ENABLE();
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 4199;
  htim2.Init.Period = 19;
  HAL_TIM_OC_Init(&htim2);
  
  TIM_Channel_InitStruct.OCMode = TIM_OCMODE_TOGGLE;
  TIM_Channel_InitStruct.OCPolarity = TIM_OCPOLARITY_LOW;
  TIM_Channel_InitStruct.OCFastMode = TIM_OCFAST_DISABLE;
  
  HAL_TIM_OC_ConfigChannel(&htim2, &TIM_Channel_InitStruct, TIM_CHANNEL_1);
}

void test_speaker (void){
  osDelay(1000);
  osThreadFlagsSet(tid_speaker, SPK_ON);
  osDelay(1000);
  osThreadFlagsSet(tid_speaker, SPK_OFF);
  osDelay(1000);
}

static void correctID (void){
  HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_1);
  osDelay(500);
  HAL_TIM_OC_Stop(&htim2, TIM_CHANNEL_1);
  osDelay(500);
}

static void wrongID (void){
  HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_1);
  osDelay(1500);
  HAL_TIM_OC_Stop(&htim2, TIM_CHANNEL_1);
}
