/**
 * Copyright (C), xxx 
 * File name:     lsm6dsl.c
 * Author:  Brian     Version:  1.0      Date: 09.16.2018
 * Description:    
            1. LSM6DSL init function.    
            2. LSM6DSL register writing and reading.
						3. Log sample from LSM6DSL.
 * Others:         
 * Function List:  
            1. static void I2C_LSM6DSL_Error (void);
						2. uint8_t I2C_LSM6DSL_ReadData(uint8_t Reg);
            3. void I2C_LSM6DSL_WriteData(uint8_t Reg, uint8_t Value);
            4. void LSM6DSL_Init(void);
						5. void LSM6DSL_GetDat(LSM6DSLDat_t* buff)
 */
#include "stm32l1xx_hal.h"
#include "main.h"
#include "lsm6dsl.h"
#include "i2c.h"
/* USER CODE BEGIN 0 */
uint32_t I2cxTimeout_6DSL = EVAL_I2Cx_TIMEOUT_MAX; //max time out duration

/**
* @brief This function prints i2c error info.
  Author:  Brian
  Description:
     If i2c error, print info and deinit i2c
  Others:
     deinit i2c1 and its io port
*/
static void I2C_LSM6DSL_Error (void)
{
	printf("\r\nLSM6DSL I2C1 error, reinit I2C1...\r\n");
  HAL_I2C_DeInit(&hi2c1);  //reinit i2c1
  HAL_I2C_MspInit(&hi2c1); 
}

/**
* @brief This function reads LSM6DSL sensor's register data.
  Author:  Brian
  Input: LSM6DSL's inner register address
  Return: 8bits value
  Description:
     When Read register, need to send Slave address+Read.
     Read a byte a time.
  Others:
     print error when failed. 
*/
uint8_t I2C_LSM6DSL_ReadData(uint8_t Reg)
{
  HAL_StatusTypeDef status = HAL_OK;
  uint8_t value = 0;
  status = HAL_I2C_Mem_Read(&hi2c1, LSM6DSL_I2C_R_ADDRESS, Reg, I2C_MEMADD_SIZE_8BIT, &value, 1, I2cxTimeout_6DSL);//read a value from register Reg of LSM6DSL via i2c1

  if(status != HAL_OK)
  {
    /* I2C error */
    I2C_LSM6DSL_Error(); 
  }
  return value;
}

/**
* @brief This function writes command to LSM6DSL sensor's register.
  Author:  Brian
  Input: LSM6DSL's inner register address, value need to be written to
  No return.
  Description:
     When Write register, need to send Slave address+Write.
     Write a byte a time.
  Others:
     print error when failed. 
*/
void I2C_LSM6DSL_WriteData(uint8_t Reg, uint8_t Value)
{
  HAL_StatusTypeDef status = HAL_OK;
  
  status = HAL_I2C_Mem_Write(&hi2c1, LSM6DSL_I2C_W_ADDRESS, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, &Value, 1, I2cxTimeout_6DSL);

  if(status != HAL_OK)
  {
    I2C_LSM6DSL_Error();
  }
}

/**
* @brief This function inits LSM6DSL.
  Author:  Brian
  Description:
     Write command to corresponding register address to set sample rate, power mode, etc.
  Others:
     Register address according to datasheet. 
*/
void LSM6DSL_Init(void)
{
	I2C_LSM6DSL_WriteData(0x12, 0x01);//softreset
	I2C_LSM6DSL_WriteData(0x12, 0x40);//BDU enable
	I2C_LSM6DSL_WriteData(0x15, 0x10);//acc low power mode
	I2C_LSM6DSL_WriteData(0x10, 0x18);//12.5Hz, ¡À4g
	I2C_LSM6DSL_WriteData(0x16, 0x80);//gyro low power mode
	I2C_LSM6DSL_WriteData(0x11, 0x36);//12.5Hz 2000dps
}

/**
* @brief This function logs data from LSM6DSL.
  Author:  Brian
  Input: a structure to store 6 axis data.
  No return.
  Description:
     Read 6 axis int16_t data, each data consists of two bytes, read from two register 
  Others:
     Register address according to datasheet. 
*/
void LSM6DSL_GetDat(LSM6DSLDat_t* buff) 
	{
  uint8_t valueL;//low 8bits of data
  uint8_t valueH;//high 8bits of data
	
	valueL = I2C_LSM6DSL_ReadData(0x28); //read x axis of Acc
  valueH = I2C_LSM6DSL_ReadData(0x29);		
  buff->ACC_X = (int16_t)( (valueH << 8) | valueL );
  
	valueL = I2C_LSM6DSL_ReadData(0x2A);  //read y axis of Acc
  valueH = I2C_LSM6DSL_ReadData(0x2B); 
  buff->ACC_Y = (int16_t)( (valueH << 8) | valueL );
  
	valueL = I2C_LSM6DSL_ReadData(0x2C);  //read z axis of Acc
  valueH = I2C_LSM6DSL_ReadData(0x2D);  
  buff->ACC_Z = (int16_t)( (valueH << 8) | valueL );
		
	valueL = I2C_LSM6DSL_ReadData(0x22);  //read x axis of Gyro
  valueH = I2C_LSM6DSL_ReadData(0x23);		
  buff->GYR_X = (int16_t)( (valueH << 8) | valueL );
  
	valueL = I2C_LSM6DSL_ReadData(0x24);  //read y axis of Gyro
  valueH = I2C_LSM6DSL_ReadData(0x25); 
  buff->GYR_Y = (int16_t)( (valueH << 8) | valueL );
  
	valueL = I2C_LSM6DSL_ReadData(0x26);  //read z axis of Gyro
  valueH = I2C_LSM6DSL_ReadData(0x27);  
  buff->GYR_Z = (int16_t)( (valueH << 8) | valueL );
	
}

/* USER CODE END 0 */
