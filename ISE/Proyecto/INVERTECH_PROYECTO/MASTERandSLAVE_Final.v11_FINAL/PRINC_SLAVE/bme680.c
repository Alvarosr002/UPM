#include "BME680.h"
#include "stm32f4xx_hal.h"
#include <stdlib.h>
#include <string.h> 
#include <stdio.h>

// Configuración I2C 
#include "Driver_I2C.h"

// Tabla de factores de corrección según gas_range
	const float constarray1[16] = {
			1.0f, 1.0f, 1.0f, 1.0f, 0.99f, 1.0f, 1.0f, 0.992f,
			1.0f, 1.0f, 0.998f, 0.996f, 1.0f, 1.0f, 1.0f, 0.986f
	};

	const float constarray2[16] = {
			8000000.0f, 4000000.0f, 2000000.0f, 1000000.0f, 499500.0f, 248000.0f, 125000.0f, 62000.0f,
			31000.0f, 15700.0f, 7800.0f, 3900.0f, 1950.0f, 970.0f, 480.0f, 240.0f
	};

//GPIO
GPIO_InitTypeDef GPIO_InitStruct;

// I2C DRIVER INSTANCE
extern ARM_DRIVER_I2C               Driver_I2C2;
static ARM_DRIVER_I2C *bme680drv = &Driver_I2C2;
	
void I2C_SignalEvent_BME680 (uint32_t event);
void Init_I2C_MASTER(void);

static volatile uint32_t I2C_Event;

// HILOS, COLAS, TIMERS
osThreadId_t tid_bme680;

// Declaración de funciones
float compensate_humidity(uint16_t hum_adc, double temp_comp,
                            uint16_t par_h1, uint16_t par_h2, uint16_t par_h3,
                            uint16_t par_h4, uint16_t par_h5, uint16_t par_h6, uint16_t par_h7);
float compensate_temperature(uint32_t temp_adc,
                               uint16_t par_t1, uint16_t par_t2, uint16_t par_t3);

float calc_gas_resistance(uint16_t gas_adc, uint8_t range_switching_error, uint8_t gas_range);
uint8_t calc_res_heat(void);
void config_GAS(void);

int get_HUM(void);
int get_GAS(void);
int get_IAQ(void);

void save_MEASURE(void);

void Init_I2C_MASTER(void);

float measure_GAS(void);
float measure_HUM (void);
void test_scan_bme680(void);
void write_I2C (uint8_t reg, uint8_t value);
uint16_t hum_adc;
uint32_t temp_adc;
float temp_comp;
uint16_t read_I2C (uint8_t reg);
uint8_t read8 (uint8_t reg);
void Conf_bme680 (void);
void ThBME680 (void *argument);

int16_t prueba = 0;

/////////////////Queue////////////////////////////////////////

osMessageQueueId_t mid_MsgQueueBME680;
MSGQUEUE_BME680_t tx_BME680;

int Init_Thbme680 (void){
	
	// CREACIÓN DEL HILO BME680
	tid_bme680 = osThreadNew(ThBME680, NULL, NULL);
  if (tid_bme680 == NULL) {
    return(-1);
  }
  
  mid_MsgQueueBME680 = osMessageQueueNew(MSGQUEUE_BME680, sizeof(MSGQUEUE_BME680_t), NULL);
	if (mid_MsgQueueBME680 == NULL) {
    return(-1);
  }
	
	return NULL;
}

void ThBME680 (void *argument){
	
	//osDelay(2000);
	Init_I2C_MASTER();
	//osDelay(2000);
	//test_scan_bme680();
	// Conf_bme680();
	tx_BME680.air_quality = 0 ;
  tx_BME680.humidity = 0.0 ;
  
	while(1){
    
    Conf_bme680();
    tx_BME680.air_quality = get_IAQ();
    tx_BME680.humidity = measure_HUM();
    osMessageQueuePut(mid_MsgQueueBME680, &tx_BME680, NULL, 0U);
    osDelay(1000);
		//uint8_t meas_Status = read8(MEAS_STATUS);	
	}
}

void I2C_SignalEvent_BME680 (uint32_t event) {
 
  /* Save received events */
  I2C_Event = event;
 
  if (event & ARM_I2C_EVENT_TRANSFER_DONE) {
    /* Transfer or receive is finished */
    osThreadFlagsSet(tid_bme680, S_TRANSFER_DONE);
  }
	
	if (event & ARM_I2C_EVENT_TRANSFER_INCOMPLETE) {
    /* Less data was transferred than requested */
  }
 
  if (event & ARM_I2C_EVENT_ADDRESS_NACK) {
    /* Slave address was not acknowledged */
  }
 
  if (event & ARM_I2C_EVENT_ARBITRATION_LOST) {
    /* Master lost bus arbitration */
  }
 
  if (event & ARM_I2C_EVENT_BUS_ERROR) {
    /* Invalid start/stop position detected */
  }
 
  if (event & ARM_I2C_EVENT_BUS_CLEAR) {
    /* Bus clear operation completed */
  }
 
  if (event & ARM_I2C_EVENT_GENERAL_CALL) {
    /* Slave was addressed with a general call address */
  }
 
  if (event & ARM_I2C_EVENT_SLAVE_RECEIVE) {
    /* Slave addressed as receiver but SlaveReceive operation is not started */
  }
 
  if (event & ARM_I2C_EVENT_SLAVE_TRANSMIT) {
    /* Slave addressed as transmitter but SlaveTransmit operation is not started */
  }
}
  // INICIALIZACION I2C MASTER -> tid_bme680
void Init_I2C_MASTER(void){
	
	osStatus_t status;
	
		__HAL_RCC_GPIOG_CLK_ENABLE();
	
	GPIO_InitStruct.Pin = GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
	
	// Control de alimentación para el sensor
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_RESET);  // CSB = 0 -> SPI (no usar)
	osDelay(1000);   // Simular ciclo de encendido/apagado si se usa GPIO como control de power
	
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_SET);    // CSB = 1 -> MODO I²C
  osDelay(5);
	
	status = bme680drv->Initialize(I2C_SignalEvent_BME680);
  status = bme680drv->PowerControl(ARM_POWER_FULL);
  bme680drv->Control(ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_FAST);
  bme680drv->Control(ARM_I2C_BUS_CLEAR, NULL);
	bme680drv->Control(ARM_I2C_OWN_ADDRESS, ADDR);
	osDelay(5);

}

void Deinit_I2C_BME680(void){
  osStatus_t status;
  
  status = bme680drv->Uninitialize();
  
}
void write_I2C (uint8_t reg, uint8_t value){
	
	uint8_t cmd[2];
	cmd[0] = reg;
	cmd[1] = value;
	
	bme680drv->MasterTransmit(ADDR, cmd, 2, false);
	osThreadFlagsWait(S_TRANSFER_DONE, osFlagsWaitAny, osWaitForever);
	
}

uint16_t read_I2C (uint8_t reg){
	uint8_t measure[2];
	
	bme680drv->MasterTransmit(ADDR, &reg, 1, true);
	osThreadFlagsWait(S_TRANSFER_DONE, osFlagsWaitAny, osWaitForever);
	bme680drv->MasterReceive(ADDR, measure, 2, false);
	osThreadFlagsWait(S_TRANSFER_DONE, osFlagsWaitAny, osWaitForever);
	
	return (measure[0] << 8) | measure[1];
}

uint8_t read8 (uint8_t reg){
	uint8_t buff;
	
	bme680drv->MasterTransmit(ADDR, &reg, 1, false);
	osThreadFlagsWait(S_TRANSFER_DONE, osFlagsWaitAny, osWaitForever);
	bme680drv->MasterReceive(ADDR, &buff, 1, false);
	osThreadFlagsWait(S_TRANSFER_DONE, osFlagsWaitAny, osWaitForever);
	
	return buff;
}

void Conf_bme680 (void){
	
	// Paso 1: Reset del sensor
	write_I2C(RESET_REG, 0xB6);
	osDelay(5);                      // Tarda 2 ms el reset, poemos un osDelay de 5 ms
	
	// Paso 2: Leer chip ID 
	uint8_t chip_id = read8(ID_REG);	// Verificación correcta de comunicación con el sensor
	
	// Configurar oversampling de humedad
  // OVERx2 (0b001) para humedad
	write_I2C(CTRL_HUM,  OVERx16);    // HUM oversampling x4 011 = OVERX4
	
	// Configurar filtro - Filtro ON: 111
	write_I2C(CONFIG, 0x1C);
	
	// Configurar oversampling de temperatura y presión + modo
  // osrs_t = 100 (x4), osrs_p = 000 (x1), mode = 01 (forced)
	uint8_t ctrl_meas = 0x40;
  write_I2C(CTRL_MEAS, ctrl_meas);
	
  // Configuracion del gas
	config_GAS();
	
	// osrs_t = 100 (x4), osrs_p = 000 (x1), mode = 01 (forced)
	ctrl_meas = 0x20 << 2 | MODE_FORCED;
	write_I2C(CTRL_MEAS, ctrl_meas);

  osDelay(1000);
}
void config_GAS(void){
	uint8_t res_heat;
	
	// CONFIGURACIÓNN DEL TIEMPO DE CALENTAMIENTO DEL SENSOR DE GAS - 100ms
	write_I2C(GAS_WAIT_0, 0x59);
	
	// CALENTAMIENTO Y MEDICION DEL SENSOR DE GAS
	res_heat = calc_res_heat();
	write_I2C(RES_WAIT_0, res_heat);
  
	// HABILITAR LAS MEDIDAS DEL GAS
	write_I2C(CTRL_GAS_1, RUN_GAS);
}

float measure_HUM (void){
	//uint16_t hum_adc;
	//uint32_t temp_adc;
	
	uint16_t par_t[3];
	uint16_t par_h[7];
	
	par_t[0]  = (read8(PAR_T1_MSB) << 8) | read8(PAR_T1_LSB);
	par_t[1]  = (read8(PAR_T2_MSB) << 8) | read8(PAR_T2_LSB);
	par_t[2]  = read8(PAR_T3);
	
	temp_adc = (read8(TEMP_MSB) << 12) | (read8(TEMP_LSB) << 4) | (read8(TEMP_xLSB) >> 4);
	temp_comp = compensate_temperature(temp_adc, par_t[0], par_t[1], par_t[2]);
	
	par_h[0]  = (read8(PAR_H1_MSB) << 4) | read8(PAR_H1_LSB) & 0x0F;
	par_h[1]  = (read8(PAR_H2_MSB) << 4) | read8(PAR_H2_LSB) >> 4;
  par_h[2]  = read8(PAR_H3);
	par_h[3]  = read8(PAR_H4);
	par_h[4]  = read8(PAR_H5);
	par_h[5]  = read8(PAR_H6);
	par_h[6]  = read8(PAR_H7);
	hum_adc = read_I2C(HUM_MSB);
	
	float hum_comp = compensate_humidity(hum_adc, temp_comp, par_h[0], par_h[1], par_h[2], par_h[3], par_h[4], par_h[5], par_h[6]);
	
	return hum_comp;
}

float measure_GAS(void){
	float gas_res;
	uint16_t gas_adc;
	uint16_t gas_range;
	uint8_t range_switching_error;
	
	// CALCULO DE LA RESISTENCIA READOUT
	gas_adc = (read8(GAS_MSB) << 2) | read8(GAS_LSB) >> 6;
	gas_range = read8(GAS_LSB) & 0x0F;                      // 0x2B <3:0>
	range_switching_error = read8(0x04) >> 4;               // register addr 0x04 <7:4>
	
	gas_res = calc_gas_resistance(gas_adc, range_switching_error, gas_range);
	
	return gas_res;
}

int get_IAQ(void){
	char* IAQ;
	int   humidity_score, gas_score;
	float air_quality_score;
	
	humidity_score = get_HUM();
  gas_score      = get_GAS();
	
	//Combinar los resultados de la humedad y el gas
  air_quality_score = humidity_score + gas_score;

	air_quality_score = (100 - air_quality_score) * 5;

	return air_quality_score;
}
int get_GAS(void) {
	int   gas;
	float gas_reference;
	int   gas_lower_limit = 10000;  // Bad air quality limit
	int   gas_upper_limit = 300000; // Good air quality limit

	gas_reference=measure_GAS();
  // Calcular la contribucion del gas en el index IAQ
  gas = (0.75 / (gas_upper_limit - gas_lower_limit) * gas_reference - (gas_lower_limit * (0.75 / (gas_upper_limit - gas_lower_limit)))) * 100.00;
  if (gas > 75) gas = 75; 
  if (gas <  0) gas = 0;  
  
	return gas;
}

// Calcular la contribucion de humedad en el index IAQ
int get_HUM(void) {  
	int   humidity_score;
	double hum_reference = 40;
	
  float current_humidity = measure_HUM();
  if (current_humidity >= 38 && current_humidity <= 42) // Humedad +/-5% optimo
    humidity_score = 0.25 * 100;
  else
  { // Humedad no optima
    if (current_humidity < 38)
      humidity_score = 0.25 / hum_reference * current_humidity * 100;
    else
    {
      humidity_score = ((-0.25 / (100 - hum_reference) * current_humidity) + 0.416666) * 100;
    }
  }
  return humidity_score;
}

float compensate_temperature(uint32_t temp_adc,
                               uint16_t par_t1, uint16_t par_t2, uint16_t par_t3)
{
    double var1, var2, t_fine;

    var1 = (((double)temp_adc / 16384.0f) - ((double)par_t1 / 1024.0f)) * ((double)par_t2);
    var2 = ((((double)temp_adc / 131072.0f) - ((double)par_t1 / 8192.0f)) *
            (((double)temp_adc / 131072.0f) - ((double)par_t1 / 8192.0f))) *
            ((double)par_t3 * 16.0f);
    t_fine = var1 + var2;
    float temp_comp = ((t_fine) / 5120.0f);

    return temp_comp;
}

float compensate_humidity(uint16_t hum_adc, double temp_comp,
                            uint16_t par_h1, uint16_t par_h2, uint16_t par_h3,
                            uint16_t par_h4, uint16_t par_h5, uint16_t par_h6, uint16_t par_h7)
{
    double var1, var2, var3, var4;
    float hum_comp;

    var1 = (double)hum_adc - (((double)par_h1 * 16.0) + (((double)par_h3 / 2.0f) * temp_comp));
    var2 = var1 * (((double)par_h2 / 262144.0f) * (1.0 + (((double)par_h4 / 16384.0f) * temp_comp) +
               (((double)par_h5 / 1048576.0f) * temp_comp * temp_comp)));
    var3 = (double)par_h6 / 16384.0f;
    var4 = (double)par_h7 / 2097152.0f;

    hum_comp = var2 + ((var3 + (var4 * temp_comp)) * var2 * var2);

    if (hum_comp > 100.0f)
        hum_comp = 100.0f;
    else if (hum_comp < 0.0f)
        hum_comp = 0.0f;

    return hum_comp;
}
uint8_t calc_res_heat(void)
{
	uint8_t res_heat;
	float var1,var2,var3, var4, var5;
	int16_t par_g2;
	uint16_t par_g[4];
	uint8_t res_heat_range_reg, res_heat_range,res_heat_val;
	
	//LEER LOS PARAMETROS DE CALIBRACION
  par_g[0]  = read8(PAR_G1);
	par_g[1]  = (read8(PAR_G2_MSB) << 8) | read8(PAR_G2_LSB);
  par_g[2]  = read8(PAR_G3);
	
	res_heat_range_reg = read8(0x02);
	res_heat_val = read8(0x00);
	res_heat_range=(res_heat_range_reg & 0x30) >> 4;

	var1 = (((float)par_g[2] / (16.0f)) + 49.0f);
	var2 = ((((float)par_g2 / (32768.0f)) * (0.0005f)) + 0.00235f);
	var3 = ((float)par_g[3] / (1024.0f));
	var4 = (var1 * (1.0f + (var2 * (float)250)));
	var5 = (var4 + (var3 * (float)temp_comp));
	res_heat =(uint8_t)(3.4f *((var5 * (4 / (4 + (float)res_heat_range)) *
									(1 / (1 + ((float)res_heat_val * 0.002f)))) - 25));

	return res_heat;
}
float calc_gas_resistance(uint16_t gas_adc, uint8_t range_switching_error, uint8_t gas_range) {
	float var1 = (1340.0f + 5.0f * range_switching_error);
	float var2 = var1 * (1.0f + constarray1[gas_range]/100.0f);
	float var3 = 1.0f + constarray1[gas_range]/100.0f;
	float gas_res = 1.0f / (float)(var3 * (0.000000125f) * (float)gas_range * ((((float)gas_adc- 512.0f) / var2) + 1.0f));
	return gas_res; // resistencia del sensor en ohmios
}
