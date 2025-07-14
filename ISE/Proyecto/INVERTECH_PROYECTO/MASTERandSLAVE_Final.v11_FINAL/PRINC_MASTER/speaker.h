
#ifndef __SPEAKER_H
#define __SPEAKER_H

#include "cmsis_os2.h"                          // CMSIS RTOS header file
#include "stm32f4xx_hal.h"
#include "stm32f4xx_it.h"
#include "stdbool.h"
#include "stdio.h"

#define SPK_ON  0x01
#define SPK_OFF 0x02

int Init_Altavoz (void);
void test_speaker (void);



#endif /* __SPEAKER_H */
