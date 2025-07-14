#include "AT24C256.h"

static void I2C_SignalEvent(uint32_t event);

extern ARM_DRIVER_I2C Driver_I2C1;
static ARM_DRIVER_I2C *I2Cdrv = &Driver_I2C1; 
static volatile uint32_t I2C_Event; 

uint8_t temp_index = 0;
extern uint8_t temp_index;
uint8_t hum_index = 0;
uint8_t air_index = 0;
uint8_t light_index = 0;
uint8_t water_index = 0;
uint8_t cons_index = 0;
uint8_t hora_index = 0;

uint8_t temp_read_index = 0;
extern uint8_t temp_read_index;
uint8_t hum_read_index = 0;
uint8_t air_read_index = 0;
uint8_t light_read_index = 0;
uint8_t water_read_index = 0;
uint8_t cons_read_index = 0;
uint8_t hora_read_index = 0;

uint8_t write_buffer[PAGE_SIZE]; 
uint8_t read_buffer[PAGE_SIZE];      

bool full = false;
extern bool full;

char web_buffer[BUFFER_SIZE][STRING_SIZE];

extern char web_buffer[BUFFER_SIZE][STRING_SIZE];;

static void I2C_SignalEvent (uint32_t event) {
  //Guardamos el evento que se ha producido
  I2C_Event |= event;
}

void I2C_Inicialize(void) {
  uint8_t val; //Variable para almacenar el dato recibido de prueba del bus I2C
	
	//Inicializamos el driver especificando la función que vamos a emplear para gestionar sus eventos
  I2Cdrv->Initialize (I2C_SignalEvent);
  
	//Especificamos las características del bus I2C
  I2Cdrv->PowerControl (ARM_POWER_FULL);
  I2Cdrv->Control      (ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_FAST);
  I2Cdrv->Control      (ARM_I2C_BUS_CLEAR, 0);
  
}

uint8_t writememory( uint8_t* WriteData, uint32_t longwrite){
  
	//Limpiamos los flags de eventos al realizar una nueva transmisión
  I2C_Event = 0U;
	//Transmitimos la dirección del registro donde escribir y sus datos
  I2Cdrv->MasterTransmit (ADDRESS_MEMORIA, WriteData, longwrite, false);
	//Esperemos hasta que se realice la transmisión
  while ((I2C_Event & ARM_I2C_EVENT_TRANSFER_DONE) == 0U);
	//Comprobamos que todos los datos se han transmitido
  if ((I2C_Event & ARM_I2C_EVENT_TRANSFER_INCOMPLETE) != 0U) return -1;
	
	return 0;
}

uint8_t readmemory(uint8_t* Registerdirectory, uint8_t* data, uint32_t longlecture){
  
  //Direccion del registro
  //Limpiamos los flags de eventos al realizar una nueva transmisión
  I2C_Event = 0U;
		//Transmitimos la dirección del registro a leer
  I2Cdrv->MasterTransmit (ADDRESS_MEMORIA, Registerdirectory, 2, true);
		//Esperemos hasta que se realice la transmisión
  while ((I2C_Event & ARM_I2C_EVENT_TRANSFER_DONE) == 0U);
		//Comprobamos que todos los datos se han transmitido
  if ((I2C_Event & ARM_I2C_EVENT_TRANSFER_INCOMPLETE) != 0U) return -1;
	
	//Lectura datos registro
		//Limpiamos los flags de eventos al realizar una nueva transmisión
  I2C_Event = 0U;
		//Solicitamos recibir los datos
  I2Cdrv->MasterReceive (ADDRESS_MEMORIA, data, longlecture , false);
		//Esperemos hasta que se realice la transmisión
  while ((I2C_Event & ARM_I2C_EVENT_TRANSFER_DONE) == 0U);
  	//Comprobamos que todos los datos se han transmitido
  if ((I2C_Event & ARM_I2C_EVENT_TRANSFER_INCOMPLETE) != 0U) return -1;

	return 0;
}


void guardar_medida(uint16_t base_addr, uint8_t* data, uint8_t data_len, uint8_t* index) {
    // Calculamos cuántas entradas caben por página según el tamaño de los datos
    uint8_t max_entries = 0X0A;

    // Si el índice supera el límite, lo reseteamos
    if (*index == max_entries) {
        *index = 0;
        full = true;
    }

    // Dirección en memoria para esta medida
    uint16_t addr = base_addr + (*index * data_len);

    // Preparar el buffer: 2 bytes de dirección + data_len bytes de dato
    write_buffer[0] = (addr >> 8) & 0xFF;  // Dirección MSB
    write_buffer[1] = addr & 0xFF;         // Dirección LSB

    for (uint8_t i = 0; i < data_len; i++) {
        write_buffer[2 + i] = data[i];     // Copiamos el dato al buffer
    }

    writememory(write_buffer, 2 + data_len);  // Dirección + dato
    osDelay(5);
    
    (*index)++;  // Aumentamos el índice
}

void guardar_temperatura(uint16_t medida) {
    uint8_t data[2];
    data[0] = (medida >> 8) & 0xFF;
    data[1] = medida & 0xFF;

    guardar_medida(TEMP_PAGE_ADDR, data, 2, &temp_index);
}

void guardar_humedad(uint16_t medida) {
    uint8_t data[2];
    data[0] = (medida >> 8) & 0xFF;
    data[1] = medida & 0xFF;

    guardar_medida(HUM_PAGE_ADDR, data, 2, &hum_index);
}

void guardar_luminosidad(uint32_t medida) {
    uint8_t data[3];
    data[0] = (medida >> 16) & 0xFF;
    data[1] = (medida >> 8) & 0xFF;
    data[2] = medida & 0xFF;

    guardar_medida(LUMI_PAGE_ADDR, data, 3, &light_index);
}

void guardar_calidad_aire(uint8_t medida) {
    uint8_t data = medida;

    guardar_medida(AIR_PAGE_ADDR, &medida, 1, &air_index);
}

void guardar_nivel_agua(uint16_t medida) {
    uint8_t data[2];
    data[0] = (medida >> 8) & 0xFF;
    data[1] = medida & 0xFF;

    guardar_medida(WATER_PAGE_ADDR, data, 2, &water_index);
}

void guardar_consumo(uint16_t medida) {
    uint8_t data[2];
    data[0] = (medida >> 8) & 0xFF;
    data[1] = medida & 0xFF;

    guardar_medida(CONS_PAGE_ADDR, data, 2, &cons_index);
}

void guardar_hora(uint8_t horas, uint8_t minutos, uint8_t segundos) {
    uint8_t data[3] = {horas, minutos, segundos};
    
    guardar_medida(HORA_PAGE_ADDR, data, 3, &hora_index);
}

void leer_hora(uint8_t* horas, uint8_t* minutos, uint8_t* segundos) {
    uint8_t buffer[3];
 
    leer_medida(HORA_PAGE_ADDR, buffer, 3, &hora_read_index);
    
    *horas = buffer[0];
    *minutos = buffer[1];
    *segundos = buffer[2];
}

void leer_medida(uint16_t base_addr, uint8_t* destino, uint8_t data_len, uint8_t* read_index){
    
    uint8_t max_entries = 0X0A;
    
    if (*read_index == max_entries) {
        *read_index = 0;
    }
    
     uint16_t addr = base_addr + (*read_index * data_len);
    
    uint8_t addr_buf[2];
    addr_buf[0] = (addr >> 8) & 0xFF;
    addr_buf[1] = addr & 0xFF;

    readmemory(addr_buf, destino, data_len);
    
    (*read_index)++;
}

void leer_todas_las_medidas(uint16_t* temperatura, uint16_t* humedad, uint8_t* calidad_aire,
                          uint32_t* luminosidad, uint16_t* consumo, uint16_t* nivel_agua,
                          uint8_t* horas, uint8_t* minutos, uint8_t* segundos) {
    uint8_t buffer[3];
  
    // Leer temperatura (2 bytes)
    leer_medida(TEMP_PAGE_ADDR, buffer, 2, &temp_read_index);
    *temperatura = (buffer[0] << 8) | buffer[1];
  
    // Leer humedad (2 bytes)
    leer_medida(HUM_PAGE_ADDR, buffer, 2, &hum_read_index);
    *humedad = (buffer[0] << 8) | buffer[1];
  
    // Leer calidad aire (1 byte)
    leer_medida(AIR_PAGE_ADDR, buffer, 1, &air_read_index);
    *calidad_aire = buffer[0];
  
    // Leer luminosidad (3 bytes)
    leer_medida(LUMI_PAGE_ADDR, buffer, 3, &light_read_index);
    *luminosidad = ((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | buffer[2];
  
    // Leer consumo (2 bytes)
    leer_medida(CONS_PAGE_ADDR, buffer, 2, &cons_read_index);
    *consumo = (buffer[0] << 8) | buffer[1];
  
    // Leer nivel agua (2 bytes)
    leer_medida(WATER_PAGE_ADDR, buffer, 2, &water_read_index);
    *nivel_agua = (buffer[0] << 8) | buffer[1];
    
    // Leer hora (3 bytes: horas, minutos, segundos)
    leer_medida(HORA_PAGE_ADDR, buffer, 3, &hora_read_index);
    *horas = buffer[0];
    *minutos = buffer[1];
    *segundos = buffer[2];
}

void agregar_buffCirc (const char* texto){
  snprintf(web_buffer[temp_read_index - 1], STRING_SIZE, "%s", texto);  // Se resta 1 al indice de lectura para que la posicion del buffer en la que se guarden las medidas a representar se encuentren en el intervalo de 0 a 9
}

void leer_una_medida_y_mostrar(void) {
  uint16_t temp = 0, hum = 0, agua = 0, consumo = 0;
  uint8_t aire = 0, h = 0, m = 0, s = 0;
  uint32_t luz = 0;
    
  leer_todas_las_medidas(&temp, &hum, &aire, &luz, &consumo, &agua, &h, &m, &s);
  
  char linea[STRING_SIZE];
  snprintf(linea, STRING_SIZE,
          "H: %02d:%02d:%02d | T: %.2fºC | H:%.2f %% | WL:%.2f %% | L:%.2f lx | AQ: %d PM",
          h, m, s, temp/100.0f, hum/100.0f, agua/100.0f, luz/100.0f, aire);
  
  agregar_buffCirc(linea);
}
