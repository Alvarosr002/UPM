/*
  Autor: Aitor Casado de la Fuente
  Grupo: M04M05M06A
*/

#include "Thlcd.h"
#include "Arial12x12.h"

/*
  int sprintf(char *str, const char *format)
  
  str: puntero a una matriz de elementos char conde se almacena la cadena C resultante
  format: cadena que contien el texto que se escribirá en el buffer
Esta ultima, puede contener etiquetas de formato

ETIQUETAS:

- %c: Utilizada para formatear un carácter.
- %s: Utilizada para formatear una cadena de caracteres (string).
- %d o %i: Utilizada para formatear un número entero con signo.
- %u: Utilizada para formatear un número entero sin signo.
- %x o %X: Utilizada para formatear un número entero en hexadecimal.
- %o: Utilizada para formatear un número entero en octal.
- %f: Utilizada para formatear un número en punto flotante (decimal).
- %e o %E: Utilizada para formatear un número en notación científica.
- %p: Utilizada para formatear un puntero.
- %%: Utilizada para imprimir el carácter '%' literalmente.

*/

extern osThreadId_t tid_Display;

/* Thread ID*/
osThreadId_t tid_Display;                        // thread id
extern osThreadId_t tid_Display;                        // thread id

/* Local functions */
void Thread (void *argument);                   // thread function
static void Set_Timers (void);

extern ARM_DRIVER_SPI Driver_SPI1;
ARM_DRIVER_SPI *SPIdrv = &Driver_SPI1; //n debe ser el numero del interface seleccionado en el fichero de configuracion RTE_device.h
ARM_SPI_STATUS stat;
TIM_HandleTypeDef tim7;

/*Variables internas*/
unsigned char buffer[512]; //Cada página tiene 128 columnas y hay 4 páginas: 128x4 = 512 columnas en total
uint16_t positionL1;
uint16_t positionL2;
uint8_t centrado1 = 30;
uint8_t centrado2 = 35;
uint8_t line = 0;

uint32_t v1 = 1234;
float v2 = 3.14159;

//Las frases ocupan parte del array, por lo que hemos de conocer que posiciones del array
//van a ocupar para especificar que escriba de cierto punto a cierto punto
char linea1[19];
char linea2[23];
int i = 0;

char cadenaLCD[100];
uint32_t tipoEntero = 1234;
uint32_t longitudCadena;

char lcd_text[2][20+1] = {0};
bool exitLoop = false;
uint32_t flagNFC = 0x0000;

extern MEDIDAS_t meas_global;

/* Private function prototypes -----------------------------------------------*/
void SPI_Callback (uint32_t event);
void delay (uint32_t n_microsegundos);
void LCD_wr_data (unsigned char data);
void LCD_wr_cmd (unsigned char cmd);
void symbolToLocalBuffer(uint8_t line, uint8_t symbol);
void Prueba_L1 (void);
void Prueba_L2 (void);
void pruebaTexto (void);
void L1(void);
void L2(void);
void write_lcd(char cadena[], int linea, int longitudCadena);
void cleanLine (void);
void LCD_Clean(void);
void LCD_Initialize (void);
void escrituraLCD (uint8_t linea, const char frase[20]);
void cleanBuffer (uint8_t line);

/* Thread ---------------------------------------------------------*/
int Init_LCD (void) {
 
  tid_Display = osThreadNew(Thread, NULL, NULL);
  if (tid_Display == NULL) {
    return(-1);
  }
 
  return(0);
}
 
void Thread (void *argument) {
  
  /* LCD Initialization */
  LCD_Initialize ();
  
  /* Timer Initialization */
//  osTimerStart(id_tim_1s, 1000U);
 
  while (1) {
    
    /* 1º) Primer panel: modo bajo consumo. */
    strcpy(lcd_text[0], "  Consumo actual BC:");
    sprintf(lcd_text[1], "       %.1f  mA   ", 150.6f);
    escrituraLCD_V2(1, lcd_text[0]);
    escrituraLCD_V2(2, lcd_text[1]);
    LCD_Update();
    
    flagNFC = osThreadFlagsWait(ENTER_INV, osFlagsWaitAll, osWaitForever);
    
    if((flagNFC & ENTER_INV) == ENTER_INV){
      flagNFC = 0x0000; //Flag reset
      
      /* 2º) Segundo panel: NFC detectado. */
      LCD_Clean();
      strcpy(lcd_text[0], "      Bienvenido    ");
      strcpy(lcd_text[1], "    al invernadero!  ");
      escrituraLCD_V2(1, lcd_text[0]);
      escrituraLCD_V2(2, lcd_text[1]);
      LCD_Update();
      osDelay(5000);
      
      exitLoop = false;
      
      while(!exitLoop){
        /* 3º) Consumo actual de la tarjeta. */
        LCD_Clean();
        strcpy(lcd_text[0], "   Consumo actual:   ");
        sprintf(lcd_text[1], "       %.1f mA    ", meas_global.consumo);
        escrituraLCD_V2(1, lcd_text[0]);
        escrituraLCD_V2(2, lcd_text[1]);
        LCD_Update();
        
        flagNFC = osThreadFlagsWait(EXIT_INV, osFlagsWaitAll, 2500U);
        if(flagNFC == EXIT_INV){ //Devuelve 0xFFFFFFFE, por eso entraba en el if porque hacía & con 0x02
          exitLoop = true;
          break;
        }
        
        /* 4º) Primeras medidas. */
        LCD_Clean();
        sprintf(lcd_text[0], "L:%.1f lx w:%.1f %%", meas_global.lum, meas_global.quantity);
        sprintf(lcd_text[1], "   CA:%d PM   ", meas_global.air_quality);
        escrituraLCD_V2(1, lcd_text[0]);
        escrituraLCD_V2(2, lcd_text[1]);
        LCD_Update();
        
        flagNFC = osThreadFlagsWait(EXIT_INV, osFlagsWaitAll, 2500);
        if(flagNFC == EXIT_INV){ //Devuelve 0xFFFFFFFE, por eso entraba en el if porque hacía & con 0x02
          exitLoop = true;
          break;
        }
        
        /* 5º) Segundas medidas. */
        LCD_Clean();
        sprintf(lcd_text[0], "    T:%.1f *C    ", meas_global.temperatura);
        sprintf(lcd_text[1], "     H:%.1f %%      ", meas_global.humidity);
        escrituraLCD_V2(1, lcd_text[0]);
        escrituraLCD_V2(2, lcd_text[1]);
        LCD_Update();
        
        flagNFC = osThreadFlagsWait(EXIT_INV, osFlagsWaitAll, 2500U);
        if(flagNFC == EXIT_INV){ //Devuelve 0xFFFFFFFE, por eso entraba en el if porque hacía & con 0x02
          exitLoop = true;
          break;
        }
      }
    }
    
    /* 6º) NFC de salida detectado (mismo ID). */
    LCD_Clean();
    strcpy(lcd_text[0], "    Hasta luego!   ");
    strcpy(lcd_text[1], "   Vuelve pronto!   ");
    escrituraLCD_V2(1, lcd_text[0]);
    escrituraLCD_V2(2, lcd_text[1]);
    LCD_Update();
    osDelay(5000);
    
//    osThreadYield();                            // suspend thread
  }
}

/* Private functions ---------------------------------------------------------*/
void SPI_Callback (uint32_t event)
{
    switch (event)
    {
    case ARM_SPI_EVENT_TRANSFER_COMPLETE:
        /* Success: Wakeup Thread */
        osThreadFlagsSet(tid_Display, S_TRANS_DONE_SPI); 
        break;
    
    case ARM_SPI_EVENT_DATA_LOST:
        /*  Occurs in slave mode when data is requested/sent by master
            but send/receive/transfer operation has not been started
            and indicates that data is lost. Occurs also in master mode
            when driver cannot transfer data fast enough. */
        break;
    
    case ARM_SPI_EVENT_MODE_FAULT:
        /*  Occurs in master mode when Slave Select is deactivated and
            indicates Master Mode Fault. */
        break;
    }
}

//Función que genera el retraso del reset.
void delay (uint32_t n_microsegundos){
  
  __HAL_RCC_TIM7_CLK_ENABLE();
  
  tim7.Instance = TIM7;
  tim7.Init.Prescaler = 83; //84 Mhz / 84 = 1 MHz
  tim7.Init.Period = n_microsegundos - 1; //Si queremos 1 us, le tendremos que pasar 1: 1 MHz / 1 = 1 MHz (1 us), valor period = 0
                                          //Si queremos 1 ms, le tendremos que pasar 1000: 1 MHz / 1000 = 1000 Hz (1 ms), valor period = 999
  HAL_TIM_Base_Init(&tim7);
  HAL_TIM_Base_Start(&tim7);
  
  while((TIM7->CNT) < (n_microsegundos-1)){}  //No esperamos flags
  
  HAL_TIM_Base_Stop(&tim7);
  HAL_TIM_Base_DeInit(&tim7);
}

//Función que inicializa pines de salida GPIO: RESET (PA6), A0 (PF13) y CS (PD14).
void Init_PinesGPIO (void){
  
  GPIO_InitTypeDef GPIO_InitStruct;
  
  //Inicialización RESET: PA6
  __HAL_RCC_GPIOA_CLK_ENABLE();
  
  GPIO_InitStruct.Pin = GPIO_PIN_6; //PA6
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; //Push-Pull Salida
  GPIO_InitStruct.Pull = GPIO_PULLUP; //Activo Nivel Alto
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; //Alta frecuencia
  
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
  
  //Inicialización A0: PF13
  __HAL_RCC_GPIOF_CLK_ENABLE();
  
  GPIO_InitStruct.Pin = GPIO_PIN_13; //PF13
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; //Push-Pull Salida
  GPIO_InitStruct.Pull = GPIO_PULLUP; //Activo Nivel Alto
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; //Alta frecuencia
  
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
  
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, GPIO_PIN_SET);
  
    //Inicialización CS: PD14
  __HAL_RCC_GPIOD_CLK_ENABLE();
  
  GPIO_InitStruct.Pin = GPIO_PIN_14; //PD14
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; //Push-Pull Salida
  GPIO_InitStruct.Pull = GPIO_PULLUP; //Activo Nivel Alto
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; //Alta frecuencia
  
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
  
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
}

//Función que inicialzia al bus SPI
void LCD_Reset (void){
  /* Initialize the SPI driver */
  SPIdrv->Initialize(SPI_Callback);
  /* Power up the SPI peripheral */
  SPIdrv->PowerControl(ARM_POWER_FULL);
  /* Configure the SPI to Master, 8-bit mode @10000 kBits/sec */
  SPIdrv->Control(ARM_SPI_MODE_MASTER | ARM_SPI_CPOL1_CPHA1 | ARM_SPI_MSB_LSB | ARM_SPI_DATA_BITS(8), 20000000);
  
  
  delay(1); //Generar pulso a nivel bajo de 1 us
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET); 
  delay(5); //Tiempo después del reset
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
  delay(1001); //Generar pulso a nivel alto de 1 ms
}

//Función que escribe un dato en el LCD
void LCD_wr_data (unsigned char data){
  
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET); //CS = 0
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, GPIO_PIN_SET);   //A0 = 1
  
  //Escribimos un dato enviandolo al SPI con ->Send
  SPIdrv->Send(&data, sizeof(data));
  
  //Esperamos a que se libere el bus SPI: Diapositiva 10 de CMSIS Driver
  osThreadFlagsWait(S_TRANS_DONE_SPI, osFlagsWaitAny, osWaitForever); //NO BLOQUEANTE
  
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);   //CS = 1
}

//Función que recibe un comando en el LCD
void LCD_wr_cmd (unsigned char cmd){
  
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET); //CS = 0
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, GPIO_PIN_RESET);   //A0 = 0
  
    //Escribimos un dato enviandolo al SPI con ->Send
  SPIdrv->Send(&cmd, sizeof(cmd));
  
  //Esperamos a que se libere el bus SPI: Diapositiva 10 de CMSIS Driver
  osThreadFlagsWait(S_TRANS_DONE_SPI, osFlagsWaitAny, osWaitForever); //NO BLOQUEANTE
  
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);   //CS = 1
}

void LCD_Init (void){
  
  LCD_wr_cmd(0xAE); //Display off
  LCD_wr_cmd(0xA2); //Fija el valor de la relaci n de la tensi n de polarizaci n del LCD a 1/9
  LCD_wr_cmd(0xA0); //El direccionamiento de la RAM de datos del display es la normal
  LCD_wr_cmd(0xC8); //El scan en las salidas COM es el normal
  LCD_wr_cmd(0x22); //Fija la relaci n de resistencias interna a 2
  LCD_wr_cmd(0x2F); //Power on
  LCD_wr_cmd(0x40); //Display empieza en la l nea 0
  LCD_wr_cmd(0xAF); //Display ON
  LCD_wr_cmd(0x81); //Contraste
  LCD_wr_cmd(0x17); //Valor Contraste -> Página 46 del Display Controller
  LCD_wr_cmd(0xA4); //Display all points normal
  LCD_wr_cmd(0xA6); //LCD Display normal
}

void LCD_Update (void) {
  
  int i; 
  LCD_wr_cmd(0x00);      // 4 bits de la parte baja de la dirección a 0 
  LCD_wr_cmd(0x10);      // 4 bits de la parte alta de la dirección a 0 
  LCD_wr_cmd(0xB0);      // Página 0 
  
  for(i=0;i<128;i++){ 
    LCD_wr_data(buffer[i]); 
  } 

   
  LCD_wr_cmd(0x00);      // 4 bits de la parte baja de la dirección a 0 
  LCD_wr_cmd(0x10);      // 4 bits de la parte alta de la dirección a 0 
  LCD_wr_cmd(0xB1);      // Página 1 
   
  for(i=128;i<256;i++){ 
    LCD_wr_data(buffer[i]); 
  } 
  
  LCD_wr_cmd(0x00);       
  LCD_wr_cmd(0x10);      
  LCD_wr_cmd(0xB2);      //Página 2 


  for(i=256;i<384;i++){ 
    LCD_wr_data(buffer[i]); 
  } 
  
  LCD_wr_cmd(0x00);       
  LCD_wr_cmd(0x10);       
  LCD_wr_cmd(0xB3);      // Pagina 3 
   
   
  for(i=384;i<512;i++){ 
    LCD_wr_data(buffer[i]); 
  } 
}

//La variable centrado es la que hace que se centre el texto
void symbolToLocalBuffer_L1(uint8_t symbol){ 
  
  uint8_t i, value1, value2;
  uint16_t offset = 0;
  
  offset = 25*(symbol - ' '); 
  
  for (i = 0; i < 12; i++){
    
    value1 = Arial12x12[offset+i*2+1];
    value2 = Arial12x12[offset+i*2+2];
    
    buffer[i+0+positionL1] = value1; 
    buffer[i+128+positionL1] = value2;
    
  }
  
  positionL1 = positionL1+Arial12x12[offset];
}

//La variable centrado es la que hace que se centre el texto
void symbolToLocalBuffer_L2(uint8_t symbol){ 
  
  uint8_t i, value1, value2;
  uint16_t offset = 0;
  
  offset = 25*(symbol - ' ');
  
  for (i = 0; i < 12; i++){
    
    value1 = Arial12x12[offset+i*2+1];
    value2 = Arial12x12[offset+i*2+2];
    
    buffer[i+256+positionL2] = value1;
    buffer[i+384+positionL2] = value2;
  }
  
  positionL2 = positionL2+Arial12x12[offset];
}

void symbolToLocalBuffer(uint8_t line, uint8_t symbol){
  
  if(line == 1){
    symbolToLocalBuffer_L1(symbol);
  }
  if(line == 2){
    symbolToLocalBuffer_L2(symbol);
  }
}

void L1(void){
  sprintf(linea1,"Prueba valor1: %d", v1);
  for(i=0; i<19; i++){ //Aqui especificamos todo lo que tiene que escribir
    symbolToLocalBuffer(1, linea1[i]);
  }
}

void L2(void){
  sprintf(linea2,"Prueba valor2: %.5f", v2);
  for(i=0; i<22; i++){ //Especificamos cuánto va a escribir
    symbolToLocalBuffer(2, linea2[i]);
  }
}

void write_lcd(char cadena[], int linea, int longitudCadena){
  for(int j=0; j<longitudCadena; j++){
    symbolToLocalBuffer(linea,cadena[j]);
  }
  LCD_Update();
}

//Función optimizada ya que resetea la posicion
void escrituraLCD_V2 (uint8_t linea, char *texto){
  
  switch(linea){
    case 1:
      limpiar_L1 ();
      for(uint8_t j = 0; j < texto[j] != '\0'; j++){
        if(positionL1 <= 127){
          symbolToLocalBuffer_L1(texto[j]);
        }
      }
    break;
    
    case 2:
      limpiar_L2();
      for(uint8_t i = 0; i < texto[i] != '\0';  i++){
        if(positionL2 <= 127){
          symbolToLocalBuffer_L2(texto[i]);
        }
      }
    break;
  }
}

void escrituraLCD (uint8_t linea, const char frase[20]){
  switch(linea){
    case 1:
      for(uint8_t j = 0; j < frase[j] != '\0';  j++){
        symbolToLocalBuffer_L1(frase[j]);
      }
    break;
      
    case 2:
      for(uint8_t i = 0; i < frase[i] != '\0';  i++){
        symbolToLocalBuffer_L2(frase[i]);
      }
    break;
  }
//  LCD_Update ();
}

void cleanLine (void){
  positionL1 = 0;
  positionL2 = 0;
}

void cleanBuffer (uint8_t line){
  if(line == 1)
  {
    for(int i = 0; i<256; i++)
    {
      buffer[i] = 0x00;
    }
  }else if(line == 2)
  {
    for(int i = 256; i<512; i++)
    {
      buffer[i] = 0x00;
    }
  }
}

void LCD_Clean(void)
{
  memset(buffer, 0 , 512u); //implica añadir la libreria: #include "string.h"
  //LCD_Update();
}

void LCD_Initialize (void){
  Init_PinesGPIO ();
  LCD_Reset ();
  LCD_Init ();
  LCD_Clean ();
  LCD_Update ();
}

void limpiar_L1 (void){
  //Reseteamos la posicion de la linea para que empieze a escribir desde el principio de esta
  positionL1=0;
  
  //Borramos el contenido del buffer de esta linea
  for(unsigned int i = 0; i < 256; i++){
    buffer[i] = NULL;
  }
}


void limpiar_L2(void){
  
  //Reseteamos la posicion de la linea para que empieze a escribir desde el principio de esta
  positionL2=0;
  
  //Borramos el contenido del buffer de esta linea
  for(unsigned int i = 256; i < 512; i++){
    buffer[i] = NULL;
  }
}
  