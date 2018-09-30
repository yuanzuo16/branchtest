#ifndef __SENSORDATASTORE_H
#define __SENSORDATASTORE_H

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx_hal.h"

#define PEKEY1        0x89ABCDEF //write PEKEY1 and PEKEY2 to FLASH_PEKEYR to unlock FLASH_PECR register           
#define PEKEY2        0x02030405     
#define PRGKEY1       0x8C9DAEBF //write PRGKEY1 and PRGKEY2 to FLASH_PEKEYR to unlock program memory  
#define PRGKEY2       0x13141516

#define DATSIZE 6  //Head, cnt, 9 axis data, parity, 12* halfword(int16_t) = 6 word

#define FLASH_SYS_SIZE					(512*1024)  //all size, 512KB
#define FLASH_ADDR_START				0x08007000  //data writing address																	
#define FLASH_ADDR_END					(FLASH_ADDR_START+FLASH_SYS_SIZE) //end of valid data address

#define FLASH_MY_SIZE				    (400*1024)	//set a size for user, 400KB																	
#define FLASH_MY_ADDR_START	    (FLASH_ADDR_END - FLASH_MY_SIZE) //user's start address

union u32i16  //define a union to align uint32 and int16 for sample save to and read from flash
{
	 uint32_t dat32[DATSIZE]; 
	 int16_t dat16[DATSIZE*2];
};
 
void SensorDataStore_Write(uint32_t flash_w_addr_offset,uint32_t write_data[],uint32_t size); //write data to flash
void SensorDataStore_Read(uint32_t flash_r_addr_offset, uint32_t size);//read data from flash

#endif
