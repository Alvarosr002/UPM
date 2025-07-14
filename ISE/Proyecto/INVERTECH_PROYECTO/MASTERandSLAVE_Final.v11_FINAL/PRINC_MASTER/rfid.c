
/**
 * |----------------------------------------------------------------------
 * | Most of the code used in this module has been taken from a free
 * | repository (GitHub), since, as the author himself indicates
 * | (MaJerle), it is free to use and it's implementation depends
 * | on the autor's skill.
 * |----------------------------------------------------------------------
 */
 
#include "rfid.h"
#include "Principal_Master.h"

extern Estados_Principal_t estadoMaster;

extern osThreadId_t TID_PRINC_MASTER;
extern osThreadId_t tid_Display;

/* Thread */
static osThreadId_t TID_RFID;
static void Th_RFID (void *argument);

/* Timer */
static osTimerId_t id_timMeas;
static void timMeas_Callback (void *arg);
static void initTim_Meas (void);
static void take_meas (void);
static void stop_meas (void);

/* Module SPI variables */
extern ARM_DRIVER_SPI Driver_SPI2;
static ARM_DRIVER_SPI* SPIDriver = &Driver_SPI2;

/* Module Private Functions */
static void TM_MFRC522_SPI_Init (void);
static void SPI2_Callback (uint32_t event);
static void TM_MFRC522_WriteRegister (uint8_t addr, uint8_t val);
static uint8_t TM_MFRC522_ReadRegister (uint8_t addr);
static void TM_MFRC522_Init (void);
static void TM_MFRC522_Reset (void);
static void TM_MFRC522_AntennaOn (void);
static void TM_MFRC522_AntennaOff (void);
static void TM_MFRC522_SetBitMask (uint8_t reg, uint8_t mask);
static void TM_MFRC522_ClearBitMask (uint8_t reg, uint8_t mask);
static TM_MFRC522_Status TM_MFRC522_Request (uint8_t reqMode, uint8_t* tagType);
static TM_MFRC522_Status TM_MFRC522_AntiColl (uint8_t* serNum);
static void TM_MFRC522_CalculateCRC (uint8_t* pInData, uint8_t len, uint8_t* pOutData);
static void TM_MFRC522_Hiber (void);
static TM_MFRC522_Status TM_MFRC522_Check (uint8_t* id);
static TM_MFRC522_Status TM_MFRC522_ToCard(uint8_t cmd, uint8_t* txData, uint8_t txLen, uint8_t* backData, uint16_t* backLen);
bool AuthID (uint8_t* id);
static void initMBED_leds (void);

extern osThreadId_t tid_speaker;                        // thread id
//extern osThreadId_t tid_Display;                        // thread id

uint8_t id[4] = {0};
const uint8_t valid_cards[NUM_VALID_CARDS][CARD_ID_SIZE] = {
  {0x64, 0xFF, 0xFF, 0x03},
  {0xE3, 0xB8, 0xB0, 0x14}
};
bool compare = false;
bool insideGreenhouse = false;
extern bool insideGreenhouse;

/**
  * @brief Initialize and configure the SPI bus
  */
int Init_RFID (void) {
 
  TID_RFID = osThreadNew(Th_RFID, NULL, NULL);
  if (TID_RFID == NULL) {
    return(-1);
  }
  
  return(0);
}

static void Th_RFID (void *argument){
  
  uint32_t rfidFlags = 0x0000;
  uint8_t cardID[5];
  TM_MFRC522_Status status;
  
  /* Initialization */
  initMBED_leds ();
  TM_MFRC522_Init ();
  initTim_Meas (); //Send flag to start reading
  take_meas ();  /* Start 100 ms timer*/
  
  while(1){
//    test_speaker ();
    rfidFlags = osThreadFlagsWait(READID, osFlagsWaitAll, osWaitForever);
    if(rfidFlags == READID){ /* From timer */
      status = TM_MFRC522_Check(cardID);
      if(status == MI_OK){
        osDelay(200);
        memcpy(id, cardID, CARD_ID_SIZE);
        //printf("Tarjeta detectada con ID: %02X:%02X:%02X:%02X\n", cardID[0], cardID[1], cardID[2], cardID[3]);
        //Do a specific sound and turn on green led
        compare = AuthID(id);
        if(compare){
          osThreadFlagsSet(tid_speaker, SPK_ON);
          
          if(!insideGreenhouse){
            if(estadoMaster != WEB){
              /* First time at the greenhouse: ENTER */
              osThreadFlagsSet(tid_Display, ENTER_INV);
              ledsOFF ();
              HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET); //Green
              osDelay(500);
              HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET); //Green
              osThreadFlagsSet(TID_PRINC_MASTER, 0x20);
              insideGreenhouse = true;
            }else
              insideGreenhouse = false;
            
            //printf("Entrada al invernadero.\n");
          }else{
            /* Check out: EXIT */
            osThreadFlagsSet(tid_Display, EXIT_INV);
            ledsOFF ();
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET); //Green
            osDelay(500);
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET); //Green
            insideGreenhouse = false;
            //printf("Salida del invernadero.\n");
          }
          HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);   //Red
          
        }else{
          //printf("Tarjeta erronea.\n");
          HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);   //Green
          HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET); //Red
          osDelay(500);
          HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET); //Red
        }
        
      }else if(status == MI_ERR){
        //Do a specific sound and turn on red led
//        osThreadFlagsSet(tid_speaker, SPK_OFF);
        //printf("Tarjeta no detectada.\n");
//        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET); //Green
//        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET); //Red
      }
    }
  }
}

/**
  * @brief Initialize and configure the SPI bus
  */
static void TM_MFRC522_SPI_Init (void){
  
  __HAL_RCC_GPIOA_CLK_ENABLE();
  
  GPIO_InitTypeDef GPIO_InitStruct;
  
  /* Chip Select (CS) & RESET */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  
  HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);
  HAL_GPIO_WritePin (GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  
  /* SPI Interface */
  SPIDriver -> Initialize (SPI2_Callback);
  SPIDriver -> PowerControl (ARM_POWER_FULL);
  SPIDriver -> Control (ARM_SPI_MODE_MASTER | ARM_SPI_CPOL1_CPHA1 | ARM_SPI_MSB_LSB
                        | ARM_SPI_DATA_BITS (8), 20000000);
  
//  /* PB4: PIN RESET. Low level: pg. 71*/
//  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET); 
//  osDelay(1);
//  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
//  osDelay(1000);
}

/**
  * @brief SPI Callback.
  * @param event: Event that triggered the callback.
*/
static void SPI2_Callback (uint32_t event){
  
  switch(event){
    case ARM_SPI_EVENT_TRANSFER_COMPLETE:
      //Enviar flag cuando tenga el hilo hecho
      osThreadFlagsSet(TID_RFID, ARM_SPI_EVENT_TRANSFER_COMPLETE);
      break;
    
    case ARM_SPI_EVENT_DATA_LOST:
      //printf("Datos perdidos en la transferencia.\n");
      break;
    
    case ARM_SPI_EVENT_MODE_FAULT:
      //printf("Slave desactivado.\n");
      break;
  }
}

/**
  * @brief  Function to write data at the specific register
  * @params  addr: register address; val: value to write
*/
static void TM_MFRC522_WriteRegister (uint8_t addr, uint8_t val){
  
  static uint8_t tx_data[2]; //[0] for register addres; [1] for data in that register
  
  /* Low level CS: is active at low level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  
  /* Send operation to transfer address and data */
  /* Al hacer el & con 0111110 nos aseguramos que se reserve MSB y LSB. 
     El MSB depende su valor del tipo de operación (W/R), y el LSB 
     siempre tiene que estar a 0 ya que así lo indica el datasheet. Por
     lo tanto, al desplazar, forzamos el MSB a 0 (escritura).*/
  tx_data[0] = (addr << 1) & 0x7E; //Mejorar con código de Deepseek
  
  tx_data[1] = val;
  
  SPIDriver -> Send (tx_data, 2); //Indica que se envía dos bytes: primero drección, después datos
  
  //Verificar la transferencia con flag
  osThreadFlagsWait(ARM_SPI_EVENT_TRANSFER_COMPLETE, osFlagsWaitAny, osWaitForever);
  
  /* High level CS*/
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

/**
  * @brief  This function allow to read a byte from the specified register
  * @param  addr: address of the register to read
  * @retval rx_data: read byte from the register
*/
static uint8_t TM_MFRC522_ReadRegister (uint8_t addr){
  
  uint8_t rx_data; //Checkear en depuración si recibo 2 bytes y si se solapan
  uint8_t tx_data;
  
  /* Low level CS: is active at low level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  
  /* Adress to read */
  tx_data = ((addr << 1) & 0x7E) | 0x80; // Make sure to reserve de MSB = 1 (R) and LSB = 0
  
  /* Send  */
  SPIDriver -> Send(&tx_data, 1);
  osThreadFlagsWait(ARM_SPI_EVENT_TRANSFER_COMPLETE, osFlagsWaitAny, osWaitForever);
  SPIDriver -> Receive(&rx_data, 1);
  osThreadFlagsWait(ARM_SPI_EVENT_TRANSFER_COMPLETE, osFlagsWaitAny, osWaitForever);
  
  /* High level CS*/
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  
  return rx_data; 
}

/**
  * @brief This function initialize the sensor RFID MFRC522 with the next steps:
  *        1º) Initialize the SPI communication
  *        2º) Reset the sensor in order to clear registers and do a factory reset
  *        3º) Configure some registers to set the time, gain and transmission mode
  *        4º) Power on the antenna
  */
static void TM_MFRC522_Init (void){
  
  /* Initialize SPI interface*/
  TM_MFRC522_SPI_Init ();
  
  /* Reset the RFID before an operation */
  TM_MFRC522_Reset ();
  
  /* Timer configuration: registers 0x2A-0x2D*/
  TM_MFRC522_WriteRegister(MFRC522_REG_TIM_MODE, 0x8D); //10001101
  TM_MFRC522_WriteRegister(MFRC522_REG_TIM_PRESCALER, 0x3E); //62 (0x8D[3:0]<<8 | 0x3E) = 13<<8 | 62 = 3582 -> 6.78MHz/3582 = 1.89 kHz
  TM_MFRC522_WriteRegister(MFRC522_REG_TIM_RELOAD_L, 30); //Valor de recarga 30 (0x1E)
  TM_MFRC522_WriteRegister(MFRC522_REG_TIM_RELOAD_H, 0); //Cuando llega a 0, recarga dicho valor -> Genera IRQ
  
  /* Gain configuration: Set to 48 dB*/
  TM_MFRC522_WriteRegister(MFRC522_REG_GAIN_CFG, 0x70); //01110000 -> Bit 6-4: RxGain = 111 (Pg. 35)
  
  /* Transmission configuration */
  TM_MFRC522_WriteRegister(MFRC522_REG_TX_AUTO, 0x40); //Bit 6: Force100ASK = 1 (modulación ASK al 100%)
  
  /* Mode configuration */
  TM_MFRC522_WriteRegister(MFRC522_REG_MODE, 0x3D); //00111101 -> Bit 7: MSBFirst = 0 (CRC check LSB first)
                                                    //         -> Bit 5: TXWaitRF = 1 (Transmisor only starts if exist RF)
                                                    //         -> Bit 3: PolMfin  = 1 (MFIN high active)
                                                    //         -> Bits [1:0]: CRCPreset = 01 (Initial value CRC = 6363h) -> Is the standard for RFID communication
                                                    /* Check page 25 */
                                                    
   /* Activate the antenna setting to '1' bits 0 and 1 of the register 0x14*/
   TM_MFRC522_AntennaOn ();
}

/**
  * @brief Reset the MFRC522 Module
*/
static void TM_MFRC522_Reset (void){
  TM_MFRC522_WriteRegister(MFRC522_REG_COMMAND, PCD_RESETPHASE);
}

/**
  * @brief Set the antenna of the sensor MFRC522
  */
static void TM_MFRC522_AntennaOn (void){
  
  uint8_t temp = 0;
  
  /* Read from control TX register: this register (0x14) 
     control the logical behavior of the antenna -> Pins Tx1 and Tx2 */
  temp = TM_MFRC522_ReadRegister(MFRC522_REG_TX_CONTROL); //Bit 0 [Tx1RFEn]: enable driver TX1
                                                          //Bit 1 [Tx2RFEn]: enable driver TX2
                                                          /* Check table 47 */
  
  /* Check if the drivers are off */
  if(!(temp & 0x03)){ //Check both bits (00000011) -> If both are 0 -> Antenna is disabled
    //Set bit mask
    TM_MFRC522_SetBitMask(MFRC522_REG_TX_CONTROL, 0x03); //Set to '1' the bits 0 & 1 of the register 0x14
  }
}

/**
  * @brief Power off the antenna
  */
static void TM_MFRC522_AntennaOff (void){
  //Clear bit mask
  TM_MFRC522_ClearBitMask(MFRC522_REG_TX_CONTROL, 0x03); //Deactivate the antenna, setting the bits 0 & 1 to '0' of the register 0x14
}

/**
  * @brief  Set to '1' the desired bits from the specified register
  * @params  reg: register to write; mask: mask to set to '1'
*/
static void TM_MFRC522_SetBitMask (uint8_t reg, uint8_t mask){
  /* Set to '1' the bits specified in mask, without modifing the other bits */
  TM_MFRC522_WriteRegister(reg, TM_MFRC522_ReadRegister(reg) | mask); //Read the register value with 'reg' and apply logic OR to activate the desired bits
                                                                      //Finally, writes the new value in the register
}

/**
  * @brief  Set to '0' the desired bits from the specified register
  * @params  reg: register to write; mask: mask to set to '0'
*/
static void TM_MFRC522_ClearBitMask (uint8_t reg, uint8_t mask){
  /* Set to '0' the bits specified in mask, without modifing the other bits */
  TM_MFRC522_WriteRegister(reg, TM_MFRC522_ReadRegister(reg) & (~mask)); //Reverse the mask to create an antimask, reading the actual value of the register
                                                                         //Apply logic AND with the antimask to deactivate the desired bits. Finally, writes
                                                                         //again the new value in the register
}

/**
  * @brief  Find RFID cards and give back the type of card detected
  * @params  reqMode: command; tagType: pointer to save the type of detected card
  * @return  TM_MFRC522_Status (MI_OK, MI_ERR, MI_NOTAGERR)
*/
static TM_MFRC522_Status TM_MFRC522_Request (uint8_t reqMode, uint8_t* tagType){
  
  TM_MFRC522_Status status;
  uint16_t backBits = 0; /* Received bits */
  
  /* Configure bitFramming register to handle communication. This register is the key for communication
       - TxLastBits[2:0]: number of last valid bits in the last transmitted byte 
       - RxAlign[5:3]: alignment of received bits (normally 0 is used for default alignment).*/
  TM_MFRC522_WriteRegister(MFRC522_REG_BIT_FRAMING, 0x07); //0x0D -> 0x07: set bits 0,1, 2 to '1' (TxLastBits defines the number of bits of the last byte that shall
                                                           //be transmitted. The RC522 will only process the first 7 bytes
  
  /* Send request command and wait for answer*/
  tagType[0] = reqMode;
  status = TM_MFRC522_ToCard(PCD_TRANSCEIVE, tagType, 1, tagType, &backBits);
  
  if((status != MI_OK) || (backBits != 0x10)){
    status = MI_ERR;
  }
  
  return status;
}

/**
  * @brief  Enable to make cards anti-collision
  * @params  serNum: 8 bits array with card ID
  * @return  TM_MFRC522_Status (MI_OK, MI_ERR, MI_NOTAGERR) depending of ID
*/
static TM_MFRC522_Status TM_MFRC522_AntiColl (uint8_t* serNum){
  
  TM_MFRC522_Status status = 0;
  uint8_t i = 0; uint8_t serNumChk = 0;
  uint16_t length = 0;
  
  /* Register that controls how bits are transmitted*/
  TM_MFRC522_WriteRegister(MFRC522_REG_BIT_FRAMING, 0x00); //0x0D ->0x00: TxLastBits = 000 -> No partials bits
  
  serNum[0] = PICC_ANTICOLL; //Command for anti-collision
  serNum[1] = 0x20;          //Anti-collision level (0x20 = Level 1 for 4, 7 and 10 bytes)
  
  /* Send this commands to card */
  status = TM_MFRC522_ToCard(PCD_TRANSCEIVE, serNum, 2, serNum, &length);
  
  /* Checksum XOR required from standard ISO/IEC 14443-3 */
  if(status == MI_OK){
    for(i = 0; i < 4; i++){
      serNumChk ^= serNum[i]; /* XOR operation to calculate checksum of UID only for the first four bytes*/
    }                         /* the result must match with the four byte. If not, error message.*/
    if(serNumChk != serNum[i]){
      status = MI_ERR;
    }
  }
  
  return status;
  
}

/**
  * @brief  Calculate CRC (Cyclic Redundacy Check) for input data
  * @params  pInData: pointer to input data to calculate CRC; len: data length in bytes; 
             pOutData: array to store CRC calculated
*/
static void TM_MFRC522_CalculateCRC (uint8_t* pInData, uint8_t len, uint8_t* pOutData){
  
  uint8_t i = 0; uint8_t n = 0;
  
  TM_MFRC522_ClearBitMask(MFRC522_REG_DIV_IRQ, 0x04); /* Register 0x05: Bit 2 [CRCIRq] = '0' to not do the CRC */
  TM_MFRC522_SetBitMask(MFRC522_REG_FIFO_LEVEL, 0x80); /* Register 0x0A: Bit 7 [FlushBuffer] = '1' to clean the FIFO.
                                                          Clean both buffers, read and write pointer. */
  
  /* Itera el array con los datos de entrada del CRC para guardarlo en la FIFO y después decirle al sensor
     que se ha guardado en la FIFO. */
  for(i = 0; i < len; i++){
    TM_MFRC522_WriteRegister(MFRC522_REG_FIFO_DATA, *(pInData + i));
  }
  TM_MFRC522_WriteRegister(MFRC522_REG_COMMAND, PCD_CALCCRC); //Como se ha guardado en la FIFO, le dices que ya se puede calcular el CRC
  
  /* Esperamos a que se haya calculado completamente el CRC */
  i = 0xFF; //Valor al máximo posible para asegurarte que se calcule el CRC
  do{
    n = TM_MFRC522_ReadRegister(MFRC522_REG_DIV_IRQ); //Devuelve
    i--;
  }while ((i != 0) && !(n & 0x04)); //Que no se agote la FIFO y se haya calculado el CRC porque de normal esa condición va a ser true siempre (while(1))
                                    //Cuando n = 0x04 quiere decir que CRCIrq = 1, por lo que sale del do-while
  
  //Guardamos el valor del CRC leído de ambos registros donde se alamcena su valor, en dos partes.
  pOutData[0] = TM_MFRC522_ReadRegister(MFRC522_REG_CRC_RESULT_L);
  pOutData[1] = TM_MFRC522_ReadRegister(MFRC522_REG_CRC_RESULT_M);
  
}

/**
  * @brief  Put a specific card into hibernation to stop data transfer from the same card
            After writing or reading a specific block
*/
static void TM_MFRC522_Hiber (void){
  
  uint16_t length = 0;
  uint8_t buff[4] = {0};
  
  buff[0] = PICC_HALT;
  buff[1] = 0;
  TM_MFRC522_CalculateCRC(buff, 2, &buff[2]);
  
  TM_MFRC522_ToCard(PCD_TRANSCEIVE, buff, 4, buff, &length);
}

/**
  * @brief  Function toverify the existnce of a RFID card at the reading field of the MFRC522
            saving the ID at th specified pointer
  * @params  id: pointer to a memory area of 5 bytes where the ID of the detected card is stored
*/
static TM_MFRC522_Status TM_MFRC522_Check (uint8_t* id){
  
  TM_MFRC522_Status status = 0;
  
  /* Start to find cards, return status if find or not*/
  status = TM_MFRC522_Request(PICC_REQIDL, id);
  
  /* If card is previously detected... */
  if(status == MI_OK){
    /* Check for anti-collision, return card serial number 4 bytes */
    status = TM_MFRC522_AntiColl(id);
  }
  
  /* Send card to hibernation, ready for future operations */
  TM_MFRC522_Hiber();
  
  return status;
}

static TM_MFRC522_Status TM_MFRC522_ToCard(uint8_t cmd, uint8_t* txData, uint8_t txLen, uint8_t* backData, uint16_t* backLen){
  
  TM_MFRC522_Status status;
  uint8_t irqEn = 0x00; uint8_t waitIRQ = 0x00;
  uint8_t lastBits = 0; uint8_t n = 0;
  uint16_t i = 0;
  
  if(cmd == PCD_TRANSCEIVE){
    irqEn = 0x77; //Enable all the relevant interruptions
    waitIRQ = 0x30; //Wait for RxIRq and IdleIRq interruptions
  }
  
  TM_MFRC522_WriteRegister(MFRC522_REG_COMM_IE_N, irqEn | 0x80); //Send to register 0x02 -> 0x77 + 0x80 = 0xF7 -> Activates all IRQs except IdleEn
  TM_MFRC522_ClearBitMask(MFRC522_REG_CMD_IRQ, 0x80); //Reg 0x04 -> 0x80: Set1 = '0' -> Marked bits in CommIrqReg are cleared
  TM_MFRC522_SetBitMask(MFRC522_REG_FIFO_LEVEL, 0x80); /* Register 0x0A: Bit 7 [FlushBuffer] = '1' to clean the FIFO.
                                                          Clean both buffers, read and write pointer. */
  
  TM_MFRC522_WriteRegister(MFRC522_REG_COMMAND, PCD_IDLE); //No action; cancels current command execution
  
  /* Write data at FIFO */
  for(i = 0; i < txLen; i++){
    TM_MFRC522_WriteRegister(MFRC522_REG_FIFO_DATA, txData[i]);
  }
  
  /* Execute the command and start transmission */
  TM_MFRC522_WriteRegister(MFRC522_REG_COMMAND, cmd); /*Reg 0x0D: PCD_TRANSCEIVE (0x0C: command to transmits from FIFO buffer to the antenna and activates
                                                                                        automatically the receiver after transmission */
  if (cmd == PCD_TRANSCEIVE){ //If this command is 0x0C
    TM_MFRC522_SetBitMask(MFRC522_REG_BIT_FRAMING, 0x80); /* Reg 0x0D -> 0x80: StartSend = '1', data transmission starts */
  }
  
  /* Wait for data transmission*/
  i = 2000; /* This value is according to the clock freq adjustment. Operaor M1 card maximum waiting time is 25 ms*/
  do{
    n = TM_MFRC522_ReadRegister(MFRC522_REG_CMD_IRQ); //Reg 0x04 [CommIrqReg[7:0] = [Set1 TxIrq RxIrq IdleIrq HiAlerIrq LoAlertIrq ErrIrq TimerIrq]
    i--;
  }while((i != 0) && !(n & 0x01) && !(n & waitIRQ)); // n = 0x01 -> TimerIrq = '1': means when value of downcounter timer is 0
                                                     // n = waitIRQ = 0x30 -> RxIrq = '1': when receiver detects the end of a valid data stream
                                                     //                    -> IdleIrq = '1': when a command terminates by itself
  
  //Se sale del do-while cuando n = 0x30 significa que los datos recibidos son válidos
  
  TM_MFRC522_ClearBitMask(MFRC522_REG_BIT_FRAMING, 0x80); /* Reg 0x0D -> 0x80: StartSend = '0', data transmission finish */
  
  if(i != 0){
    if(!(TM_MFRC522_ReadRegister(MFRC522_REG_ERROR) & 0x1B)){ /* Read from errors register (0x06) & 0x1B (0001 1011): BufferOvfl, ColErr, ParityErr, ProtocolErr = '1' -> ENTER IF NO ERROR OF THIS FOUR*/
      status = MI_OK; //No error
      
      /* Only enter the if when n = 0x01 */
      if(n & irqEn & 0x01){ //If irqEn = '1' -> TimerIrq = '1': when the value of downcounter timer is 0
        status = MI_NOTAGERR;
      }
      
      if(cmd == PCD_TRANSCEIVE){
        n = TM_MFRC522_ReadRegister(MFRC522_REG_FIFO_LEVEL); //Check the number of bytes at FIFO
        lastBits = TM_MFRC522_ReadRegister(MFRC522_REG_CONROL) & 0x07; // Reg 0x0C [ControlReg] & 0x07 -> Bits[2:0] = RxLastBits (Show the number of valid bits in the last receive byte. If 0, whole byte valid
        
        /* Check if al the data stored in the FIFO is completed */
        if(lastBits){
          *backLen = (n - 1) * 8 + lastBits; //Calculate the total of the received bytes
          //Resta 1 al numero total de bytes recibidos y multiplica por 8 para obtener el número de bits totales.
          //Después suma los últimos bits recibidos (lastBits) que no son un byte completo
        }else{
          *backLen = n * 8; //If lastBits = 0 -> Whole byte is valid, and multiply by 8
        }
        
        /* If 0 bytes are received, n = 1 to avoid posterior errors */
        if(n == 0){
          n = 1;
        }
        
        /* Avoid buffer overflow */
        if(n > MFRC522_MAX_LEN){
          n = MFRC522_MAX_LEN;
        }
        
        /* Reading the received data from the FIFO */
        for(i = 0; i < n; i++){
          backData[i] = TM_MFRC522_ReadRegister(MFRC522_REG_FIFO_DATA);
        }
      }
      
    }else{
      status = MI_ERR; //An error exists
    }
  }
  
  return status;
}

bool AuthID (uint8_t* id){
  for(int i = 0; i < NUM_VALID_CARDS; i++){
    if(memcmp(id, valid_cards[i], CARD_ID_SIZE) == 0){
      return true;
    }
  }
  return false;
}

static void initMBED_leds (void){
  GPIO_InitTypeDef GPIO_InitStruct; 
  
  /*Enable clock to GPIOD*/
  __HAL_RCC_GPIOD_CLK_ENABLE();
  
  /*Set GPIOD pin */
  GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;  

  /*Init GPIOB Pins*/
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
  
  /* Common cathode. They're off with '1'*/
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);  //VERDE
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);  //AZUL
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);  //ROJO
}

static void initTim_Meas (void){
  id_timMeas = osTimerNew(timMeas_Callback, osTimerPeriodic, NULL, NULL);
}

static void timMeas_Callback (void *arg){
  osThreadFlagsSet(TID_RFID, READID);
}

static void take_meas (void){
  osTimerStart(id_timMeas, 1000U);
}

static void stop_meas (void){
  osTimerStop(id_timMeas);
}
