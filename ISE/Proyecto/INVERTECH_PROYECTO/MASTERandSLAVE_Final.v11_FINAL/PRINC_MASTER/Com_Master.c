#include "cmsis_os2.h"                          // CMSIS RTOS header file
#include "Principal_Master.h"
#include <time.h>
 
/*----------------------------------------------------------------------------
 *      Thread 1 'Thread_Name': Sample thread
 *---------------------------------------------------------------------------*/
 
osThreadId_t tid_Thread;                        // thread id
//osMessageQueueId_t mid_Recibir_Master;
osMessageQueueId_t mid_Enviar_Master;

Medidas medidas;
void medir_sensores(void);
 
void Thread (void *argument);                   // thread function
 
int Init_Thread (void) {
 
  tid_Thread = osThreadNew(Thread, NULL, NULL);
  if (tid_Thread == NULL) {
    return(-1);
  }
	
	//mid_Recibir_Master = osMessageQueueNew(10, sizeof(comando), NULL);
	mid_Enviar_Master  = osMessageQueueNew(10, sizeof(medidas), NULL);
 
  return(0);
}
 
void Thread (void *argument) {
	// Inicializador de numeros aleatorios
	static int init = 0;
    if (!init) {
        srand(time(NULL));
        init = 1;
    }
 
  while (1) {
		osThreadFlagsWait(0xFF, osFlagsWaitAny, osWaitForever);
		
		switch(comando){
			case SLEEP:
				osThreadFlagsSet(TID_PRINC_MASTER, 0xFF);
				
			break;
			
			case MEAS:
				osThreadFlagsSet(TID_PRINC_MASTER, 0xFF);
				medir_sensores();
		    osMessageQueuePut(mid_Enviar_Master, &medidas, NULL, NULL);
			break;
			
			case INIT_PRESENCIAL:
				osThreadFlagsSet(TID_PRINC_MASTER, 0xFF);
			break;
			
			case INIT_WEB:
				osThreadFlagsSet(TID_PRINC_MASTER, 0xFF);
			break;
		}
    osThreadYield();                            // suspend thread
  }
}

void medir_sensores(void){
	// Inicializar el generador de números aleatorios una sola vez

    medidas.temp = (float)rand() / RAND_MAX * 3.2f;               // 0.0 - 3.2 V
    medidas.hum  = 20.0 + (rand() % 8000) / 100.0;               // 20.0 - 100.0 %
    medidas.lum  = (float)(rand() % 10000);                      // 0 - 10,000 lux (estimado)
    medidas.iaq  = rand() % 501;                                 // 0 - 500
    medidas.consumo  = (float)rand() / RAND_MAX * 3.2f;            // 0.0 - 3.2 V
    medidas.quantity  = (float)(rand() % 1001) / 10.0f;          // 0.0 - 100.0
}