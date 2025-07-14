#include "cmsis_os2.h"                          // CMSIS RTOS header file
#include "slave.h"
 
/*----------------------------------------------------------------------------
 *      Thread 1 'Thread_Name': Sample thread
 *---------------------------------------------------------------------------*/
 
 extern osThreadId_t tid_Thread;
osThreadId_t tid_Thread;                        // thread id
 
 extern int Init_Thread();
 
void Thledp (void *argument);                   // thread function
 
int Init_Thread (void) {
 
  tid_Thread = osThreadNew(Thledp, NULL, NULL);
  if (tid_Thread == NULL) {
    return(-1);
  }
 
  return(0);
}
 
void Thledp (void *argument) {
 
  while (1) {
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0); // Insert thread code here...
    osDelay(1000);
    osThreadYield();                            // suspend thread
  }
}
