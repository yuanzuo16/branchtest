#ifndef __LSM303AGR_H
#define __LSM303AGR_H

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx_hal.h"


#define LSM303AGR_I2C_A_W_ADDRESS      0x32  //ACC's Slave address+Write
#define LSM303AGR_I2C_A_R_ADDRESS      0x33  //ACC's Slave address+Read
#define LSM303AGR_I2C_M_W_ADDRESS      0x3C  //Mag's Slave address+Write
#define LSM303AGR_I2C_M_R_ADDRESS      0x3D  //Mag's Slave address+Read

#define WHO_AM_I_ACC_303AGR 0x0F  //ACC's WHO_AM_I register, its value is fixed at 33h
#define WHO_AM_I_MAG_303AGR 0x4F  //MAG's WHO_AM_I register, its value is fixed at 44h

typedef struct {
	int16_t ACC_X;
  int16_t ACC_Y;
  int16_t ACC_Z;
	int16_t MAG_X;
  int16_t MAG_Y;
  int16_t MAG_Z;
} LSM303AGRDat_t;//define a structure for LSM303AGR sensor, including 3 axis accelorator value \
                   and 3 axis magnetic value, all are int16_t

static void I2C_LSM303AGR_Error (void); //error report
void I2C_LSM303AGR_WriteData(uint8_t Adrr, uint8_t Reg, uint8_t Value); //write a uint8_t Value to register address Reg
uint8_t I2C_LSM303AGR_ReadData(uint8_t Adrr, uint8_t Reg);//read a uint8_t Value from register address Reg
void LSM303AGR_Init(void);//Init LSM303AGR sensor, setting sample rate, low power mode 
void LSM303AGR_GetDat(LSM303AGRDat_t* buff);//log data from sensor
#endif
