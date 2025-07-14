#ifndef __BME680_H
#define __BME680_H

  #include "cmsis_os2.h"
	//#include "bme680_defs.h"
	//#include "bme68x.h"

  // ADDR REGISTROS W/R
	#define ADDR_BME680_W 0xEC
	#define ADDR_BME680_R 0xED
	#define ADDR          0x76
	#define CTRL_W        0xA0
	#define CTRL_R        0xF6
	
	// REGISTERS BME680
	#define ID_REG        0xD0
	#define RESET_REG     0xE0
	#define CTRL_HUM      0x72
	#define CTRL_MEAS     0x74
	#define CONFIG        0x75
	#define STATUS        0x73
	#define MEAS_STATUS   0x1D
	// REGISTERS GAS - HEATER SET POINT 0
	#define IDAC_HEAT_0 0x50   // heater current
	#define RES_WAIT_0  0x5A   // heater resistence
	#define GAS_WAIT_0  0x64   // factor 4
	#define CTRL_GAS_0  0x70   // heater off
	#define CTRL_GAS_1  0x71   // Run gas
	#define RUN_GAS     0x10   
	
	#define MODE_FORCED 0x01
	
	// REGISTERS MEASURE
	#define HUM_MSB 0x25
	#define HUM_LSB	0x26
	#define GAS_MSB 0x2A
	#define GAS_LSB 0x2B
	#define GAS_R   0x2B
	
	#define TEMP_MSB 0x22
	#define TEMP_LSB 0x23
	#define TEMP_xLSB 0x24
	
	#define MEAS_STATUS 0x1D

	#define S_TRANSFER_DONE  0x01
	#define MEDIDA           0x02
	#define MSGQUEUE_OBJECTS 1
	
	// ESTRUCTURA DE LOS OBJETOS DE LAS COLAS
  typedef struct {                                // object data type
     double humidity;
		 int iaq;
  } MSGQUEUE_OBJ_BME680;
	
	enum OSRSH{
		SKIPPED = 000,
		OVERx1  = 001,
		OVERx2  = 010,
		OVERx4  = 011,
		OVERx8  = 100,
		OVERx16 = 101,
	};
	
	// GAS_WAIT_0 <7:6>
	enum GAS_WAIT {   
		WAIT_1  = 00,
		WAIT_4  = 01,
		WAIT_16 = 10,
		WAIT_64 = 11,
	};
	
	enum PAR_T{
		PAR_T1_LSB = 0xE9,   // uint16_t
		PAR_T1_MSB = 0xEA,   // uint16_t
		PAR_T2_LSB = 0x8A,   // uint16_t
		PAR_T2_MSB = 0x8B,   // uint16_t
		PAR_T3 = 0x8C,   // uint8_t
	};
	
	enum PAR_H{
		PAR_H1_MSB = 0xE3,
		PAR_H1_LSB = 0xE2,
		PAR_H2_LSB = 0xE2,
		PAR_H2_MSB = 0xE1,
		PAR_H3 = 0xE4,
		PAR_H4 = 0xE5,
		PAR_H5 = 0xE6,
		PAR_H6 = 0xE7,
		PAR_H7 = 0xE8,
	};
	
	enum PAR_G{
		PAR_G1 = 0xED,
		PAR_G2_LSB = 0xEB,
		PAR_G2_MSB = 0xEC,
		PAR_G3 = 0xEE,
	};
	
  extern osMessageQueueId_t mid_MsgQueue_bme680;	
	
	// FUNCIONES HILO BME680
	int Init_Thbme680(void);
	void ThBME680 (void *argument);
	//void Conf_bme680 (void);
	
#endif
