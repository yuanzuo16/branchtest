/**
 * Copyright (C), xxx 
 * File name:     sensordatastore.c
 * Author:  Brian     Version:  1.0      Date: 09.16.2018
 * Description:    
            1. Writing sample to flash.    
            2. Reading sample from flash.
 * Others:         
 * Function List:  
            1. void SensorDataStore_Write(uint32_t flash_w_addr_offset,uint32_t write_data[],uint32_t size); 
            2. void SensorDataStore_Read(uint32_t flash_r_addr_offset, uint32_t size);
 */
#include "stm32l1xx_hal.h"
#include "main.h"
#include "sensordatastore.h"
#include "stm32l1xx_hal_flash.h"
#include "stm32l1xx_hal_flash_ex.h"
/* USER CODE BEGIN 0 */

/**
* @brief This function write data to flash.
  Author:  Brian
	Input: write address offset, write data array, size of data
	No return
  Description:
     step1. Unlock flash.
	   step2. Caculate page position, page offset and size remain
	   step3. When writing to first address, erase the page. Otherwise, if there is not enough size for data writing, erase the next page in advance
	   step4. Write data to address, and increase the write address by 4 each time
	   step5. Lock flash.
  Others:
     If write address exceed, print error
*/

void SensorDataStore_Write(uint32_t flash_w_addr_offset,uint32_t write_data[],uint32_t size)
{
	
	HAL_FLASH_Unlock();// unlock flash	

	uint32_t tempwadd = FLASH_MY_ADDR_START + flash_w_addr_offset;
	
	uint32_t pagepos = tempwadd/256;//page number
	uint16_t pageoff = (tempwadd%256)/sizeof(uint32_t);//the position in a page 
	uint16_t sizeremain = 256/sizeof(uint32_t)-pageoff;//remain size for writing
	
	uint32_t *pdat=write_data;
	
		if(flash_w_addr_offset==0)//write data from the first address, erase the page first
		{
				FLASH_EraseInitTypeDef f; 
				f.TypeErase = FLASH_TYPEERASE_PAGES;
				f.PageAddress = tempwadd;
				f.NbPages = 1; //1 page	
				uint32_t PageError = 0;
				HAL_FLASHEx_Erase(&f, &PageError);//erase page			
		}
		else if(sizeremain<sizeof(uint32_t)*size) //if there is not enough size for data writing, erase the next page in advance
		{
				FLASH_EraseInitTypeDef f; 
				f.TypeErase = FLASH_TYPEERASE_PAGES;
				f.PageAddress = (pagepos+1)*256; //next page's address
				f.NbPages = 1;
				uint32_t PageError = 0;
				HAL_FLASHEx_Erase(&f, &PageError);//erase page			
		}


	for(uint32_t t=0;t<size;t++) //write data to flash
	{
	  if(tempwadd <= FLASH_ADDR_END-sizeof(uint32_t)) //address is valid
	  {
			*pdat = write_data[t];
			HAL_FLASH_Program(FLASH_TYPEERASEDATA_WORD,tempwadd,*pdat);	//write a word
			pdat++;
			tempwadd += sizeof(uint32_t); //address added by sizeof(uint32_t)
	  }
	  else
	  {
	    printf("\r\nFlash Write Addr error!\r\n"); //address is invalid, print error
	    break;
	  }
	  
	}	
	HAL_FLASH_Lock();
}

/**
* @brief This function read data from flash.
  Author:  Brian
	Input: read address offset, size of data
	No return
  Description:
     Read a word each time, and align all the data to int16_t to get 9 axis data and other info. 
     Print data to PC.
     If reading address is bigger than valid sample data address, print info.
  Others:

*/
void SensorDataStore_Read(uint32_t flash_r_addr_offset, uint32_t size)
{
	union u32i16 read_data_buf;//6 words = 12 int16_t, Head+cnt+9 axis data+parity,
	uint32_t tempradd = FLASH_MY_ADDR_START + flash_r_addr_offset;
	uint32_t read_data_cnt=0;
		
	 for(uint8_t i=0;i<size;i++)
	 {
		 for(uint8_t j=0;j<DATSIZE;j++)//each set of data include 6 bytes
		 {	
         if(tempradd <= FLASH_ADDR_END) //read address valid
				 {
						read_data_buf.dat32[j] = *(__IO uint32_t*)(tempradd); //read a word
						tempradd += sizeof(uint32_t);	//address added by sizeof(uint32_t)					 
				 }
				 else
				 {
					  printf("\r\nRequesting data from flash error!\r\n");//error
					  break;
				 }											 
		 }							  
		 
		     if(read_data_buf.dat32[0]==0x00000000) //read out blank flash, means all samples have been read out
					{
						printf("\r\nAll the samples read, including %.1f seconds' data!\r\n",(float)read_data_cnt/2);
						break;
					}
         else	
				 {
				   read_data_cnt++;
				 }					 

			int16_t read_parity=0;
			for(uint8_t i=0;i<DATSIZE*2-2;i++)
			{
			  read_parity ^= read_data_buf.dat16[i];
			}
			
			if(read_data_buf.dat16[DATSIZE*2-1]!=read_parity)
			{
			  printf("\r\nData Parity Error!\r\n");
			}
			else
			{				
				printf("9 axis data reading: ");
				for(uint8_t i=0;i<DATSIZE*2;i++)
				{
					 printf("%7d, ",read_data_buf.dat16[i]);
				}
				printf("\r\n"); 
		  }
		
			
	 }

}
/* USER CODE END 0 */

