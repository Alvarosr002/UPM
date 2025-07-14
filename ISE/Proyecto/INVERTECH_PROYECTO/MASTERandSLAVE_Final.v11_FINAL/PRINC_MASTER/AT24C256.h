#ifndef __AT24C256_H
#define __AT24C256_H

#include "cmsis_os2.h" 	
#include "Driver_I2C.h"	
#include "stdio.h"

#define ADDRESS_MEMORIA 0x50 //Dirección de la memoria EEPROM

#define PAGE_SIZE           64
#define TEMP_PAGE_ADDR      0x0000  // Página 0
#define CONS_PAGE_ADDR      0x0040  // Página 1
#define LUMI_PAGE_ADDR      0x0080  // Página 2
#define AIR_PAGE_ADDR       0x00A0  // Página 3
#define HUM_PAGE_ADDR       0x0100  // Página 4
#define WATER_PAGE_ADDR     0x0140  // Página 5
#define HORA_PAGE_ADDR      0x0180  // Página 6

#define BUFFER_SIZE 10
#define STRING_SIZE 90
  
void I2C_Inicialize(void); 
uint8_t readmemory( uint8_t* Registerdirectory, uint8_t* data, uint32_t longlecture); 
uint8_t writememory( uint8_t* WriteData, uint32_t longwrite); 
void guardar_medida(uint16_t base_addr, uint8_t* data, uint8_t data_len, uint8_t* index);
void guardar_temperatura(uint16_t medida);
void guardar_humedad(uint16_t medida);
void guardar_luminosidad(uint32_t medida);
void guardar_calidad_aire(uint8_t medida);
void guardar_nivel_agua(uint16_t medida);
void guardar_consumo(uint16_t medida);
void guardar_hora(uint8_t horas, uint8_t minutos, uint8_t segundos);

void leer_hora(uint8_t* horas, uint8_t* minutos, uint8_t* segundos);
void leer_medida(uint16_t base_addr, uint8_t* destino, uint8_t data_len, uint8_t* index);
void leer_todas_las_medidas(uint16_t* temperatura, uint16_t* humedad, uint8_t* calidad_aire,uint32_t* luminosidad, uint16_t* consumo, uint16_t* nivel_agua,uint8_t* horas, uint8_t* minutos, uint8_t* segundos);

void agregar_buffCirc (const char* texto);
#endif 


