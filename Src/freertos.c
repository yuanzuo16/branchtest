/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/**
 * Copyright (C), xxx 
 * File name:     freertos.c
 * Author:  Brian     Version:  1.0      Date: 09.16.2018
 * Description:    
            1. freertos init, including setting up tasks, semaphore and queue.    
            2. Data logging and save.
						3. Processing user command and reading data from flash.
						4. Led flashing.
 * Others:         
 * Function List:  
            1. void MX_FREERTOS_Init(void);
						2. void Func_Sensor_Data_Acquiring_Saving(void const * argument);
            3. void Func_User_Cmd_Processing(void const * argument);
            4. void Func_Led_Flashing(void const * argument);
 */
/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */     
#include "tim.h"
#include "lsm6dsl.h"
#include "lsm303agr.h"
#include "sensordatastore.h"
#include "string.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId Task_DataacquHandle;
osThreadId Task_UsercmdHandle;
osThreadId Task_LedHandle;
osMessageQId CmdQueueHandle;
osSemaphoreId Timer3SemHandle;

/* USER CODE BEGIN Variables */
extern uint8_t g_user_key_value;//global var, defined in main.c
/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void Func_Sensor_Data_Acquiring_Saving(void const * argument);
void Func_User_Cmd_Processing(void const * argument);
void Func_Led_Flashing(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* Hook prototypes */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of TimerSem */
  osSemaphoreDef(TimerSem);
  Timer3SemHandle = osSemaphoreCreate(osSemaphore(TimerSem), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of Task_Dataacqu */
  osThreadDef(Task_Dataacqu, Func_Sensor_Data_Acquiring_Saving, osPriorityNormal, 0, 128); //Task Priority highest
  Task_DataacquHandle = osThreadCreate(osThread(Task_Dataacqu), NULL);

  /* definition and creation of Task_Usercmd */
  osThreadDef(Task_Usercmd, Func_User_Cmd_Processing, osPriorityBelowNormal, 0, 128); //Task Priority medium
  Task_UsercmdHandle = osThreadCreate(osThread(Task_Usercmd), NULL);

  /* definition and creation of Task_Led */
  osThreadDef(Task_Led, Func_Led_Flashing, osPriorityLow, 0, 128);  //Task Priority lowest
  Task_LedHandle = osThreadCreate(osThread(Task_Led), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */
  /* definition and creation of CmdQueue */
  osMessageQDef(CmdQueue, 1, uint32_t);
  CmdQueueHandle = osMessageCreate(osMessageQ(CmdQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
	 if(osSemaphoreWait(Timer3SemHandle,100) != osOK) //get the timer semaphore first, because the semaphore created by HAL library \
		                            is defalut valid, need to get it so that Func_Sensor_Data_Acquiring_Saving function will get the right semaphore from timer3
	 {
      printf("\r\nTimerSem failed to get!\r\n");
   }
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* Func_Sensor_Data_Acquiring_Saving function 
* @brief This function write data to flash.
  Author:  Brian
  Description:
     step1. Wait for timer3 semaphore, log data every 0.5 second.
	   step2. printf data to PC.
	   step3. Package data.
	   step4. Write data to flash.
  Others:
     Using a head for data spliting, a cnt to indicate the time of sample, a parity to verify the data package.
*/		 
void Func_Sensor_Data_Acquiring_Saving(void const * argument)
{
  /* USER CODE BEGIN Func_Sensor_Data_Acquiring_Saving */

  /* Infinite loop */
		LSM6DSLDat_t lsm6data;
    LSM303AGRDat_t lsm3data;
		union u32i16 write_data_buf;//6 words = 12 int16_t, Head+cnt+9 axis data+parity
	  static uint32_t s_write_addr_offset=0;//flash write address offset
    static uint16_t s_write_cnt=0;//the number of data saved in flash, increased by 1 when logging a new sample
  /* Infinite loop */
  for(;;)
  {
	  if(osSemaphoreWait(Timer3SemHandle,500) == osOK)//wait for timer's semaphore
	  {
			LSM6DSL_GetDat(&lsm6data); //log sample from LSM6DSL
			LSM303AGR_GetDat(&lsm3data); //log sample from LSM303AGR
			
			//package the data, format as head(0x55),cnt,3 axis ACC,3 axis GYR,3 axis MAG,parity\
			parity generated by a formula, and print data to PC	
			s_write_cnt++;
			write_data_buf.dat16[0] = 0x55AA;               
			write_data_buf.dat16[1] = s_write_cnt; 
			
			int16_t *p_data;
			p_data = &lsm6data.ACC_X;
			printf("9 axis data: ");
			for(uint8_t i=0;i<sizeof(LSM6DSLDat_t)/2;i++)
			{
				 write_data_buf.dat16[i+2] = *p_data;
				 printf("%7d, ",*p_data);
				 p_data++;
			}
			
			p_data = &lsm3data.MAG_X;
			for(uint8_t i=0;i<sizeof(LSM303AGRDat_t)/2-3;i++)
			{
				 write_data_buf.dat16[sizeof(LSM6DSLDat_t)/2+2+i] = *p_data;
				 printf("%7d, ",*p_data);
				 p_data++;
			}
			printf("\r\n");

			int16_t write_parity=0;
			for(uint8_t i=0;i<DATSIZE*2-2;i++)
			{
			  write_parity ^= write_data_buf.dat16[i];
			}
			write_data_buf.dat16[DATSIZE*2-1] = write_parity;
			
			SensorDataStore_Write(s_write_addr_offset,&write_data_buf.dat32[0],DATSIZE);	//write data to flash	 
			s_write_addr_offset += sizeof(uint32_t) *DATSIZE; //write address added by sizeof(uint32_t)*DATSIZE bytes
		 
	  }
    osDelay(5);
	}
  /* USER CODE END Func_Sensor_Data_Acquiring_Saving */
}

/* Func_User_Cmd_Processing function 
  Author:  Brian
  Description:
     To receive command from UART2 via osMessage and further process.
     step1. Receive readsample command to stop data sampling and wait for further command.
     step2. Receive the number of seconds' data that user wants to read from the internal flash.
     step3. Read the data and send to PC via UART2
     step4. Wait for restart command or pressing USER KEY to exit and continue data sampling.
     At each step, when received restart command or pressing USER KEY can exit and restart data sampling.
     When received invalid command, print warning.
     During data sampling, only readsample and restart command are valid, the former can stop data sampling 
     and process user's data requesting, while the later can restart data sampling.
  Others:
     One second's data includes 2*9 axis data.
*/
void Func_User_Cmd_Processing(void const * argument)
{
  /* USER CODE BEGIN Func_User_Cmd_Processing */
  /* Infinite loop */
	osEvent val;
	static uint8_t s_user_req_cnt=0; //count when get a queue from UART
  uint32_t read_addr_offset=0;//flash read address offset

  for(;;)
  {
		val = osMessageGet(CmdQueueHandle,osWaitForever);	//get queue by using osMessageGet
		if(val.status == osEventMessage)
		{
		    if(g_user_key_value==1) s_user_req_cnt = 0;  //when g_user_key_value is 1- during data sampling, always detect whether command is readsample
				
				s_user_req_cnt++;	
				
				if(s_user_req_cnt==1)  //step1. detect readsample command
			  {	
					if(strcmp(val.value.p,"readsample")==0)
					{
						printf("\r\nUser requesting samples...\r\n");		 
						HAL_TIM_Base_Stop_IT(&htim3);  //stop timer3 to log sample from sensor
						printf("\r\nHow many seconds' data to read from flash?\r\n");
						printf("\rOr send restart command or Press USER KEY to exit!\r\n");	
						g_user_key_value = 0; //set g_user_key_value to 0, so that data sampling can begin when press USER KEY\
						                    Since received readsample command, to go to step2
					}
					else if((strcmp(val.value.p,"restart")==0)) //restart is also valid command
					{
					  printf("\r\nRestart data sampling...\r\n");		 
	          HAL_TIM_Base_Start_IT(&htim3);//enable timer3
						g_user_key_value = 1; //set g_user_key_value to 1, so that data sampling can stop when press USER KEY
						s_user_req_cnt= 0;  //stay at step1
					}
					else	
					{
						printf("\r\nCommand invalid!\r\n"); //printf warning
						printf("\rSend restart command or Press USER KEY to exit!\r\n");	
						s_user_req_cnt = 0;
					}
				}
				 
				if(s_user_req_cnt==2) //step2. detect valid number of 
			  {
          if(strspn(val.value.p,"0123456789")==strlen(val.value.p))
					{
						uint32_t read_size=0;
						read_size = 2*atoi(val.value.p); //change command to uint32_t number, one second has two sets of data
						printf("\r\nTrying to read %d seconds' samples, as bellow:\r\n",read_size/2);						
						SensorDataStore_Read(read_addr_offset,read_size); //read data from flash and print
						printf("\r\nSample reading finished!\r\n");
						printf("\rSend restart command or Press USER KEY to exit!\r\n");// when received valid number, go to step3
					}
					else if((strcmp(val.value.p,"restart")==0))  //restart, and go to step1
					{
					  printf("\r\nRestart data sampling...\r\n");		 
	          HAL_TIM_Base_Start_IT(&htim3);
						g_user_key_value = 1;
						s_user_req_cnt= 0;
					}
					else  //print error, and stay at step2
					{
						printf("\r\nInput a valid size, should only include number!\r\n");
						printf("\rOr send restart command or Press USER KEY to exit!\r\n");
						s_user_req_cnt= 1;
					}
				}
				
				if(s_user_req_cnt==3) //step3. waiti for restart command or USER KEY pressing
				{
				  if((strcmp(val.value.p,"restart")==0)) //restart
					{
					  printf("\r\nRestart data Sampling...\r\n");		 
	          HAL_TIM_Base_Start_IT(&htim3);
						g_user_key_value = 1;
						s_user_req_cnt= 0;
					}
					else
					{
					  printf("\r\nSend restart command or Press USER KEY to exit!\r\n");
					  s_user_req_cnt = 2;  //stay at step3
					}
				}
				
		}				
    osDelay(5);
	}

  /* USER CODE END Func_User_Cmd_Processing */
}

/* Func_Led_Flashing function */
void Func_Led_Flashing(void const * argument)
{
  /* USER CODE BEGIN Func_Led_Flashing */
  /* Infinite loop */
  for(;;)
  {
		HAL_GPIO_TogglePin(led_green_GPIO_Port,led_green_Pin);
    osDelay(1000);
  }
  /* USER CODE END Func_Led_Flashing */
}

/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
