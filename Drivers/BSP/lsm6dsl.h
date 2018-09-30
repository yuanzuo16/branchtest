#ifndef __LSM6DSL_H
#define __LSM6DSL_H

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx_hal.h"

#define LSM6DSL_I2C_W_ADDRESS      0xD6 //Slave address+Write
#define LSM6DSL_I2C_R_ADDRESS      0xD7 //Slave address+Read



#define WHO_AM_I_ACC_6DSL 0x0F  //WHO_AM_I register, its value is fixed at 6Ah
	 
typedef struct {
	int16_t ACC_X;
  int16_t ACC_Y;
  int16_t ACC_Z;
	int16_t GYR_X;
  int16_t GYR_Y;
  int16_t GYR_Z;
} LSM6DSLDat_t;  //define a structure for LSM6DSL sensor, including 3 axis accelorator value \
                   and 3 axis gyroscope value, all are int16_t

static void I2C_LSM6DSL_Error (void);//error report
void I2C_LSM6DSL_WriteData(uint8_t Reg, uint8_t Value);//write a uint8_t Value to register address Reg
uint8_t I2C_LSM6DSL_ReadData(uint8_t Reg);//read a uint8_t Value from register address Reg
void LSM6DSL_Init(void);//Init LSM6DSL sensor, setting sample rate, low power mode 
void LSM6DSL_GetDat(LSM6DSLDat_t* buff);//log data from sensor
#endif
