/**
 * Copyright (C), xxx 
 * File name:     lsm303agr.c
 * Author:  Brian     Version:  1.0      Date: 09.16.2018
 * Description:    
            1. LSM303AGR init.    
            2. LSM303AGR register writing and reading.
						3. Log sample from LSM303AGR.
 * Others:         
 * Function List:  
            1. static void I2C_LSM303AGR_Error (void);
            2. void I2C_LSM303AGR_WriteData(uint8_t Adrr, uint8_t Reg, uint8_t Value);
            3. uint8_t I2C_LSM303AGR_ReadData(uint8_t Adrr, uint8_t Reg);
            4. void LSM303AGR_Init(void);
            5. void LSM303AGR_GetDat(LSM303AGRDat_t* buff);
 */
#include "stm32l1xx_hal.h"
#include "main.h"
#include "lsm303agr.h"
#include "i2c.h"
/* USER CODE BEGIN 0 */

uint32_t I2cxTimeout_303AGR = EVAL_I2Cx_TIMEOUT_MAX;//max time out duration

/**
* @brief This function prints i2c error info.
  Author:  Brian
  Description:
     If i2c error, print info and deinit i2c
  Others:
     deinit i2c1 and its io port
*/
static void I2C_LSM303AGR_Error (void)
{
	printf("\r\nLSM303AGR I2C1 error, reinit I2C1...\r\n");
  HAL_I2C_DeInit(&hi2c1);//reinit i2c1
  HAL_I2C_MspInit(&hi2c1);  
}

/**
* @brief This function reads LSM303AGR sensor's register data.
  Author:  Brian
  Input: LSM303AGR's device i2c address and inner register address
  Return: 8bits value
  Description:
     When Read register, need to send Slave address+Read, LSM303AGR has two devices, ACC and MAG, different devices have different device i2c address
     Read a byte a time.
  Others:
     print error when failed. 
*/
uint8_t I2C_LSM303AGR_ReadData(uint8_t Adrr, uint8_t Reg)
{
  HAL_StatusTypeDef status = HAL_OK;
  uint8_t value = 0;
  status = HAL_I2C_Mem_Read(&hi2c1, Adrr, Reg, I2C_MEMADD_SIZE_8BIT, &value, 1, I2cxTimeout_303AGR);//read a value from register Reg of LSM303AGRGR via i2c1

  if(status != HAL_OK)
  {
    /* I2C error */
   I2C_LSM303AGR_Error();
  
  }
  return value;
}

/**
* @brief This function writes command to LSM303AGR sensor's register.
  Author:  Brian
  Input: LSM303AGRGR's device i2c address, inner register address, value need to be written to
  No return.
  Description:
     When Write register, need to send Slave address+Write, two devices, two different device i2c address
     Write a byte a time.
  Others:
     print error when failed. 
*/
void I2C_LSM303AGR_WriteData(uint8_t Adrr, uint8_t Reg, uint8_t Value)
{
  HAL_StatusTypeDef status = HAL_OK;
  
  status = HAL_I2C_Mem_Write(&hi2c1, Adrr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, &Value, 1, I2cxTimeout_303AGR);

  if(status != HAL_OK)
  {
    I2C_LSM303AGR_Error();
  }
}


/**
* @brief This function inits LSM303AGR.
  Author:  Brian
  Description:
     Write command to corresponding register address to set sample rate, power mode, etc.
  Others:
     Register address according to datasheet, two devices, two different device i2c address 
*/
void LSM303AGR_Init(void)
{
	I2C_LSM303AGR_WriteData(LSM303AGR_I2C_A_W_ADDRESS, 0x20, 0x27);//normal mode,Z,Y,X enable
	I2C_LSM303AGR_WriteData(LSM303AGR_I2C_A_W_ADDRESS, 0x23, 0x90);//BUD enable, ¡À4g 
	I2C_LSM303AGR_WriteData(LSM303AGR_I2C_M_W_ADDRESS, 0x60, 0x20); //software reset
	I2C_LSM303AGR_WriteData(LSM303AGR_I2C_M_W_ADDRESS, 0x60, 0x00); //normal moede, 10Hz, Continuous mode 

}

/**
* @brief This function logs data from LSM303AGR.
  Author:  Brian
  Input: a structure to store 6 axis data.
  No return.
  Description:
     Read 6 axis int16_t data, each data consists of two bytes, read from two register 
  Others:
     Register address according to datasheet, two devices, two different device i2c address 
*/
void LSM303AGR_GetDat(LSM303AGRDat_t* buff) 
	{
  uint8_t valueL;//low 8bits of data
  uint8_t valueH;//high 8bits of data
	
	valueL = I2C_LSM303AGR_ReadData(LSM303AGR_I2C_A_R_ADDRESS,0x28);  //read x axis of Acc
  valueH = I2C_LSM303AGR_ReadData(LSM303AGR_I2C_A_R_ADDRESS,0x29);		
  buff->ACC_X = (int16_t)( (valueH << 8) | valueL );
  
	valueL = I2C_LSM303AGR_ReadData(LSM303AGR_I2C_A_R_ADDRESS,0x2A); //read y axis of Acc 
  valueH = I2C_LSM303AGR_ReadData(LSM303AGR_I2C_A_R_ADDRESS,0x2B); 
  buff->ACC_Y = (int16_t)( (valueH << 8) | valueL );
  
	valueL = I2C_LSM303AGR_ReadData(LSM303AGR_I2C_A_R_ADDRESS,0x2C);  //read z axis of Acc
  valueH = I2C_LSM303AGR_ReadData(LSM303AGR_I2C_A_R_ADDRESS,0x2D);  
  buff->ACC_Z = (int16_t)( (valueH << 8) | valueL );
		
	valueL = I2C_LSM303AGR_ReadData(LSM303AGR_I2C_M_R_ADDRESS,0x68);//read x axis of Mag
  valueH = I2C_LSM303AGR_ReadData(LSM303AGR_I2C_M_R_ADDRESS,0x69);		
  buff->MAG_X = (int16_t)( (valueH << 8) | valueL );
  
	valueL = I2C_LSM303AGR_ReadData(LSM303AGR_I2C_M_R_ADDRESS,0x6A); //read y axis of Mag 
  valueH = I2C_LSM303AGR_ReadData(LSM303AGR_I2C_M_R_ADDRESS,0x6B); 
  buff->MAG_Y = (int16_t)( (valueH << 8) | valueL );
  
	valueL = I2C_LSM303AGR_ReadData(LSM303AGR_I2C_M_R_ADDRESS,0x6C); //read z axis of Mag 
  valueH = I2C_LSM303AGR_ReadData(LSM303AGR_I2C_M_R_ADDRESS,0x6D);  
  buff->MAG_Z = (int16_t)( (valueH << 8) | valueL );
	
}

/* USER CODE END 0 */
