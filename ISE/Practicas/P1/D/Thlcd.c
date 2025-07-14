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

extern ARM_DRIVER_SPI Driver_SPI1;
ARM_DRIVER_SPI *SPIdrv = &Driver_SPI1; //n debe ser el numero del interface seleccionado en el fichero de configuracion RTE_device.h
ARM_SPI_STATUS stat;
TIM_HandleTypeDef tim7;

/*Variables internas*/
unsigned char buffer[512]; //Cada página tiene 128 columnas y hay 4 páginas: 128x4 = 512 columnas en total
uint8_t positionL1;
uint8_t positionL2;
uint8_t centrado1 = 30;
uint8_t centrado2 = 35;
uint8_t line = 0;

uint32_t longitudCadena;

/* Private function prototypes -----------------------------------------------*/
//static void SystemClock_Config(void);
//static void Error_Handler(void);

void SPI_Callback (uint32_t event);
void delay (uint32_t n_microsegundos);
void LCD_wr_data (unsigned char data);
void LCD_wr_cmd (unsigned char cmd);
void symbolToLocalBuffer_L1(uint8_t symbol);
void symbolToLocalBuffer_L2(uint8_t symbol);
void symbolToLocalBuffer(uint8_t line, uint8_t symbol);
void write_lcd(char cadena[], int linea, int longitudCadena);
void cleanLine (void);
void cleanBuffer (uint8_t line);

void Codigo_a_Pintar (void);

///*--------------------------------------------------------------------------
//*                                HILO
//*--------------------------------------------------------------------------*/

extern osThreadId_t TID_Display;

//osThreadId_t tid_ThLCD;                       	  // ID del Hilo

//void ThLCD (void *argument);                 	 // Funcion principal del Hilo

//int Init_ThLCD (void) {

//  tid_ThLCD = osThreadNew(ThLCD, NULL, NULL);
//  if (tid_ThLCD == NULL) {
//    return(-1);
//  }
//	
//	Init_PinesGPIO();
//	LCD_Reset();
//	LCD_Init();
//	Codigo_a_Pintar();
//	LCD_Update();
//	
//  return(0);
//}

//void ThLCD (void *argument){
//	
//  while(1){
//    // Insert thread code here...
//		
//    osThreadYield();                            // suspend thread
//    
//  }

//}

/* Private functions ---------------------------------------------------------*/

void SPI_Callback (uint32_t event)
{
    switch (event)
    {
    case ARM_SPI_EVENT_TRANSFER_COMPLETE:
        /* Success: Wakeup Thread */
        osThreadFlagsSet(TID_Display, S_TRANS_DONE_SPI); 
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
  osThreadFlagsWait(S_TRANS_DONE_SPI, osFlagsWaitAny, osWaitForever);
  
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);   //CS = 1
}

//Función que recibe un comando en el LCD
void LCD_wr_cmd (unsigned char cmd){
  
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET); //CS = 0
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, GPIO_PIN_RESET);   //A0 = 0
  
    //Escribimos un dato enviandolo al SPI con ->Send
  SPIdrv->Send(&cmd, sizeof(cmd));
  
  //Esperamos a que se libere el bus SPI: Diapositiva 10 de CMSIS Driver
  osThreadFlagsWait(S_TRANS_DONE_SPI, osFlagsWaitAny, osWaitForever);
  
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
void symbolToLocalBuffer_L1(uint8_t symbol){ //Escribe en la página 0 y 1, eso es la línea 1
  
  uint8_t i, value1, value2;
  uint16_t offset = 0;
  
  offset = 25*(symbol - ' '); //Cada simbolo tiene 25 bytes. Lo que hace el offset es situarse en la 
                              //posición del array donde empieza el caracter. De esta forma, lo va
                              //recorriendo hasta que termina de escribirlo.
  
  for (i = 0; i < 12; i++){
    
    value1 = Arial12x12[offset+i*2+1];
    value2 = Arial12x12[offset+i*2+2];
    
    //Lo que hacemos con value1 y value2 es ir recorriendo los bytes del caracter 2 a 2. Value1 lee
    //el primer byte y value2 lee el segundo byte. El primer byte se representa en la página 0 y el
    //segundo byte se representa en la página 1.
    
    buffer[i+0+/*centrado1*/+positionL1] = value1; //Página 0, empieza en la columna 0
    buffer[i+/*centrado1*/+128+positionL1] = value2; //Página 1, empieza en la columna 128
    
    
  }
  
  positionL1 = positionL1+Arial12x12[offset];
}

//La variable centrado es la que hace que se centre el texto
void symbolToLocalBuffer_L2(uint8_t symbol){ //Escribe en la página 2 y 3, eso es la línea 2
  
  uint8_t i, value1, value2;
  uint16_t offset = 0;
  
  offset = 25*(symbol - ' ');
  
  for (i = 0; i < 12; i++){
    
    value1 = Arial12x12[offset+i*2+1];
    value2 = Arial12x12[offset+i*2+2];
    
    buffer[i+256+/*centrado2*/+positionL2] = value1; //Página 2, empieza en la columna 256
    buffer[i+384+/*centrado2*/+positionL2] = value2; //Página 3, empieza en la columna 384
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

void write_lcd(char cadena[], int linea, int longitudCadena){
  
  for(int j=0; j<longitudCadena; j++){
    symbolToLocalBuffer(linea,cadena[j]);
  }
  LCD_Update();
}

void cleanLine (void)
{
  positionL1 = 0;
  positionL2 = 0;
}

void cleanBuffer (uint8_t line)
{
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

void cleanLCD(void){
	
	cleanLine();
	cleanBuffer(1);
	cleanBuffer(2);
	
}

void Codigo_a_Pintar (void){
	
	
	
}
