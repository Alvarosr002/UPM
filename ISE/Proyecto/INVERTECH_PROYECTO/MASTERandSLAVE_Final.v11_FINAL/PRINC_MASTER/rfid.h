/**
 * MF RC522 Default pinout
 *
 *      MFRC522     STM32F4XX    DESCRIPTION
 *      CS (SDA)    PB12         Chip select for SPI
 *      SCK         PB13         Serial Clock for SPI
 *      MISO        PC2          Master In Slave Out for SPI
 *      MOSI        PB15         Master Out Slave In for SPI
 *      GND         GND          Ground
 *      VCC         3.3V         3.3V power
 *      RST         PB4          Reset pin
 *
*/

#ifndef __RFID_H
#define __RFID_H

#include "Driver_SPI.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"                          // CMSIS RTOS header file
#include "stdio.h"
#include "speaker.h"
#include "string.h"
#include "LEDS.h"

typedef enum {
  MI_OK = 0,
  MI_NOTAGERR,
  MI_ERR
} TM_MFRC522_Status;

#define MFRC522_MAX_LEN 16

/* Thread flags */
#define ENTER_INV 0x01
#define EXIT_INV  0x02

#define READID 0x01
#define NUM_VALID_CARDS 2
#define CARD_ID_SIZE    4

/* MFRC522 Data commands for CommandReg*/
#define PCD_IDLE        0x00  //NO action; Cancel the current command
#define PCD_CALCCRC     0x03  //CRC Calculate
#define PCD_TRANSMIT    0x04  //Transmit data
#define PCD_RECEIVE     0x08  //Receive Data
#define PCD_TRANSCEIVE  0x0C  //Transmit and receive data
#define PCD_AUTHENT     0x0E  //Authentication Key
#define PCD_RESETPHASE  0x0F  //Reset

/* Mifare_One card command word*/
#define PICC_REQIDL      0x26  //Find the antenna area does not enter hibernation
#define PICC_ANTICOLL    0x93  //Anti-collision
#define PICC_SELECT_TAG  0x93  //Election card
#define PICC_READ        0x30  //Read block
#define PICC_WRITE       0xA0  //Write block
#define PICC_HALT        0x50  //Sleep

/* MFRC522 Registers Bank*/

//PAGE 0: Command and Status
#define MFRC522_REG_COMMAND      0x01
#define MFRC522_REG_COMM_IE_N    0x02
#define MFRC522_REG_CMD_IRQ      0x04
#define MFRC522_REG_DIV_IRQ      0x05
#define MFRC522_REG_ERROR        0x06 
#define MFRC522_REG_FIFO_DATA    0x09
#define MFRC522_REG_FIFO_LEVEL   0x0A
#define MFRC522_REG_CONROL       0x0C
#define MFRC522_REG_BIT_FRAMING  0x0D

//PAGE 1: Command
#define MFRC522_REG_MODE        0x11
#define MFRC522_REG_TX_CONTROL  0x14
#define MFRC522_REG_TX_AUTO     0x15

//PAGE 2: Configuration
#define MFRC522_REG_CRC_RESULT_M   0x21
#define MFRC522_REG_CRC_RESULT_L   0x22
#define MFRC522_REG_GAIN_CFG       0x26
#define MFRC522_REG_TIM_MODE       0x2A
#define MFRC522_REG_TIM_PRESCALER  0x2B
#define MFRC522_REG_TIM_RELOAD_H   0x2C
#define MFRC522_REG_TIM_RELOAD_L   0x2D

//PAGE 3: Test (Not used)

/* Extern functions */
int Init_RFID (void);
extern bool insideGreenhouse;

#endif /* __RFID_H */