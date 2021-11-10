 /******************************************************************************
  * @file    oil_float.c
  * @author  MCD Application Team
  * @version V1.1.2
  * @date    01-June-2017
  * @brief   manages the sensors on the application
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
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
  
  /* Includes ------------------------------------------------------------------*/
#include "hw.h"
#include "flash_eraseprogram.h"
#include "systime.h"

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint32_t PAGEError = 0;

__IO float data32_comp_year_mon_day_hour_min = 0;

__IO uint32_t data_on_add = 0, data_on_add04=0,data_on_add12=0,data_on_add16=0;

char printf_buff[100]={"\0"};

/*Variable used for Erase procedure*/
static FLASH_EraseInitTypeDef EraseInitStruct;	

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions ---------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
/* Erase the user Flash area
    (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/
void  FLASH_erase(uint32_t page_address)
{
  /* Fill EraseInit structure*/
  EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.PageAddress = page_address;
  EraseInitStruct.NbPages     = 1;

	BACKUP_PRIMASK();
	
	DISABLE_IRQ( );
	
	HAL_FLASH_Unlock();
	
  if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK)
	{
		RESTORE_PRIMASK();
//    while (1)
//    {
      /* indicate error in Erase operation */
      PRINTF("error in Flash Erase operation\n\r");
//    }
	}
/* Lock the Flash to disable the flash control register access (recommended
     to protect the FLASH memory against possible unwanted operation) *********/
  HAL_FLASH_Lock();
	RESTORE_PRIMASK();
}

void EEPROM_erase_one_address(uint32_t address)
{
	BACKUP_PRIMASK();
	
	DISABLE_IRQ( );
	
	HAL_FLASHEx_DATAEEPROM_Unlock();
	
	if(HAL_FLASHEx_DATAEEPROM_Erase(address)!=HAL_OK)
	{
		RESTORE_PRIMASK();
//    while (1)
//    {
      /* indicate error in Erase operation */
      PRINTF("error in EEPROM Erase operation\n\r");
//    }
	}
	
	HAL_FLASHEx_DATAEEPROM_Lock();
	
	RESTORE_PRIMASK();
}

void EEPROM_erase_lora_config(void)
{
	uint32_t address;
	
	address=EEPROM_USER_START_ADDR_CONFIG;
	
	BACKUP_PRIMASK();
	
	DISABLE_IRQ( );
	
	HAL_FLASHEx_DATAEEPROM_Unlock();
	while(address<EEPROM_USER_END_ADDR_CONFIG)
	{
		if(HAL_FLASHEx_DATAEEPROM_Erase(address)!=HAL_OK)
		{
			RESTORE_PRIMASK();
//			while (1)
//			{
				/* indicate error in Erase operation */
				PRINTF("error in EEPROM Erase operation\n\r");
//			}
		}
		address = address + 4;
  }
	
	HAL_FLASHEx_DATAEEPROM_Lock();
	
	RESTORE_PRIMASK();
}

void  FLASH_erase_all_sensor_data_storage(uint32_t page_address)
{
  /* Fill EraseInit structure*/
  EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.PageAddress = page_address;
  EraseInitStruct.NbPages     = 400;

	BACKUP_PRIMASK();
	
	DISABLE_IRQ( );
	
	HAL_FLASH_Unlock();
	
  if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK)
	{
		RESTORE_PRIMASK();
//    while (1)
//    {
      /* indicate error in Erase operation */
      PRINTF("error in Flash Erase operation\n\r");
//    }
	}
/* Lock the Flash to disable the flash control register access (recommended
     to protect the FLASH memory against possible unwanted operation) *********/
  HAL_FLASH_Lock();
	RESTORE_PRIMASK();
}

void  FLASH_program(uint32_t add, uint32_t *data, uint8_t count)
{
	uint32_t Address=0;
	int i=0;
	
	BACKUP_PRIMASK();
	
	DISABLE_IRQ( );
	
	HAL_FLASH_Unlock();

  Address = add;

  while (i<count)
  {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, data[i]) == HAL_OK)
    {
      Address = Address + 4;
			i++;
		}
    else
    {
			RESTORE_PRIMASK();
      /* Error occurred while writing data in Flash memory.*/
//      while (1)
//      {
        PRINTF("error in Flash Write operation\n\r");
//      }
    }
  }

  /* Lock the Flash to disable the flash control register access (recommended
     to protect the FLASH memory against possible unwanted operation) *********/
  HAL_FLASH_Lock();
	RESTORE_PRIMASK();
}

void EEPROM_program(uint32_t add, uint32_t *data, uint8_t count)
{
	uint32_t Address=0;
	int i=0;
	Address = add;
	
	BACKUP_PRIMASK();
	
	DISABLE_IRQ( );
	
	HAL_FLASHEx_DATAEEPROM_Unlock();
	while (i<count)
  {
		if(HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_WORD,Address,data[i])== HAL_OK)
		{
			Address = Address + 4;
				i++;
		}
		else
		{
			RESTORE_PRIMASK();
			/* Error occurred while writing data in EEPROM memory.*/
//      while (1)
//      {
        PRINTF("error in EEPROM Write error\r");
//      }
		}
  }
	HAL_FLASHEx_DATAEEPROM_Lock();
	RESTORE_PRIMASK();
}

uint32_t find_addr_first_after_systemreset(uint32_t addr)
{
	uint32_t current_address,address_record,data_comp_min;
	int32_t data_comp;
	current_address=addr;
	
	data_on_add=*(__IO uint32_t *)current_address;
	
	while(data_on_add!=0x00)
	{
		current_address = current_address+16;
		if(current_address>=FLASH_SENSOR_DATA_END_ADDR)
		{
			current_address=FLASH_SENSOR_DATA_START_ADDR;
			data_on_add=*(__IO uint32_t *)current_address;
			SysTime_t sysTimeCurrent = { 0 };
	
	    sysTimeCurrent=SysTimeGet();
			data_comp=sysTimeCurrent.Seconds-data_on_add;
			if(data_comp<0)
			{
				data_comp=~data_comp+1;
			}
			data_comp_min=data_comp;
			while(current_address<=FLASH_SENSOR_DATA_END_ADDR)
			{
				data_on_add=*(__IO uint32_t *)current_address;
				data_comp=sysTimeCurrent.Seconds-data_on_add;
				if(data_comp<0)
				{
					data_comp=~data_comp+1;
				}
				
				if(data_comp<=data_comp_min)
				{
					address_record=current_address;
					data_comp_min=data_comp;					
				}
				current_address = current_address+16;
			}
			current_address=((address_record-FLASH_BASE)/128);
		  current_address=current_address*128+FLASH_BASE;//将地址归为当前page的首地址
			current_address+=128;//next page
			break;
		}
		data_on_add=*(__IO uint32_t *)current_address;
	}
	
	if(current_address>=FLASH_SENSOR_DATA_END_ADDR)
	{
//		PPRINTF("address more than user end ADDR\r");
		FLASH_erase(FLASH_SENSOR_DATA_START_ADDR);
		current_address=FLASH_SENSOR_DATA_START_ADDR;
	}
	
//	PPRINTF("find address= %0X\r",current_address);
	return current_address;
}

uint32_t find_addr(uint32_t addr)
{
	uint32_t current_address;
	current_address=addr;
	
	data_on_add=*(__IO uint32_t *)current_address;
	
	if(data_on_add!=0x00)
	{
		FLASH_erase(current_address);//擦除当前地址所在的整个page
			
		current_address=((current_address-FLASH_BASE)/128);
		current_address=current_address*128+FLASH_BASE;//将地址归为当前page的首地址
	}
	
	if(current_address>=FLASH_SENSOR_DATA_END_ADDR)
	{
//		PPRINTF("address more than user end ADDR\r");
		FLASH_erase(FLASH_SENSOR_DATA_START_ADDR);
		current_address=FLASH_SENSOR_DATA_START_ADDR;
	}
//	PPRINTF("find address= %0X\r",current_address);
	return current_address;
}

uint32_t store_sensor_data(uint32_t add,uint32_t *sensor_data)
{
	int i=0;
	
	BACKUP_PRIMASK();
	
	DISABLE_IRQ( );
	
	HAL_FLASH_Unlock();
	
	while (add < FLASH_SENSOR_DATA_END_ADDR)
  {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, add, sensor_data[i]) == HAL_OK)
    {
      add = add + 4;
			i++;
			
			if(i==4)
			{
				i=0;
				break;
			}
		}
    else
    {
      /* Error occurred while writing data in Flash memory.
         User can add here some code to deal with this error */
			RESTORE_PRIMASK();
//      while (1)
//      {
        PRINTF("error in Flash Write operation\n\r");
//      }
    }
  }
	
	HAL_FLASH_Lock();
	RESTORE_PRIMASK();

	return add;
}

uint8_t read_data_on_flash_buff(uint16_t address_start,uint16_t address_end)
{
	uint32_t buff[4];
  uint32_t timestamp;
	uint8_t de;
	
	uint16_t illu,ba,cou;
	uint8_t le;	
	float T_sht20,H_sht20,T_ds18b20;
	float adc;
	short t1,h1,t2;
	
  uint16_t startpage=address_start;
  uint16_t endpage=address_end;
	
	__IO uint32_t add1=FLASH_SENSOR_DATA_START_ADDR+(FLASH_PAGE_SIZE*(startpage-1));
	__IO uint32_t end_address=FLASH_SENSOR_DATA_START_ADDR+(FLASH_PAGE_SIZE*(endpage));
	
//	PRINTF("%0X %0X\r",add1,end_address);
//	int i=0;
	
	for (;add1 < end_address;)
//	while (add1 < end_address)
  {
		PPRINTF("%0X ",add1);

		for(int i=0;i<4;i++)
		{
      buff[i] = *(__IO uint32_t *)add1;
      add1 = add1 + 4;
		}	

		timestamp=buff[0];

		struct tm localtime;
		SysTimeLocalTime( timestamp, &localtime );
		
		de=buff[1]>>16 & 0x0f;
		
		ba=(buff[1]>>8&0xff)<<8 | (buff[1]&0xff);
		
		t1=(((buff[2]>>24&0xff)<<8) | (buff[2]>>16&0xff));
		if(t1<0)
		{
			T_sht20=(~t1+1)/-100.0;
		}
		else
		  T_sht20=t1/100.0;
		
		h1=(((buff[2]>>8&0xff)<<8) | (buff[2]&0xff));
		if(h1<0)
		{
			H_sht20=(~h1+1)/-10.0;
		}
		else
			H_sht20=h1/10.0;
		
		/*
		 dev=0x01,ds18b20
		 */
		if(de==0x00)
		{
			if(ba!=0)
			{
			  sprintf(printf_buff,"%d/%d/%d %02d:%02d:%02d %d %d sht_temp=%0.2f sht_hum=%0.1f\r",localtime.tm_year-100,localtime.tm_mon,localtime.tm_mday,localtime.tm_hour,localtime.tm_min,localtime.tm_sec,de,ba,T_sht20,H_sht20);
			
			  PPRINTF("%s",printf_buff);
			}
		}
		else if(de==0x01 || de==0x09)
		{
			t2=((buff[3]>>24&0xff)<<8) | (buff[3]>>16&0xff);
			if(t2<0)
			{
				T_ds18b20=(~t2+1)/-100.0;
			}
			else
				T_ds18b20=t2/100.0;
			
			sprintf(printf_buff,"%d/%d/%d %02d:%02d:%02d %d %d sht_temp=%0.2f sht_hum=%0.1f ds_temp=%0.2f\r",localtime.tm_year-100,localtime.tm_mon,localtime.tm_mday,localtime.tm_hour,localtime.tm_min,localtime.tm_sec,de,ba,T_sht20,H_sht20,T_ds18b20);
			
			PPRINTF("%s",printf_buff);
	  }
		else if(de==0x04)
		{
			le=buff[3]>>24&0xff;
//			ex=buff[3]>>16&0xff;
			
			if(le==0x00)
			{
				sprintf(printf_buff,"%d/%d/%d %02d:%02d:%02d %d %d sht_temp=%0.2f sht_hum=%0.1f level:low\r",localtime.tm_year-100,localtime.tm_mon,localtime.tm_mday,localtime.tm_hour,localtime.tm_min,localtime.tm_sec,de,ba,T_sht20,H_sht20);
			}	
      else if(le==0x01)
			{
				sprintf(printf_buff,"%d/%d/%d %02d:%02d:%02d %d %d sht_temp=%0.2f sht_hum=%0.1f level:high\r",localtime.tm_year-100,localtime.tm_mon,localtime.tm_mday,localtime.tm_hour,localtime.tm_min,localtime.tm_sec,de,ba,T_sht20,H_sht20);
			}		
			
			PPRINTF("%s",printf_buff);			
		}		
		else if(de==0x05)
		{
			illu=buff[3]>>16&0xffff;
			
			sprintf(printf_buff,"%d/%d/%d %02d:%02d:%02d %d %d sht_temp=%0.2f sht_hum=%0.1f illu_lux=%d\r",localtime.tm_year-100,localtime.tm_mon,localtime.tm_mday,localtime.tm_hour,localtime.tm_min,localtime.tm_sec,de,ba,T_sht20,H_sht20,illu);
			
			PPRINTF("%s",printf_buff);				
		}		
		else if(de==0x06)
		{
			adc=(buff[3]>>16&0xffff)/1000.0;
			
			sprintf(printf_buff,"%d/%d/%d %02d:%02d:%02d %d %d sht_temp=%0.2f sht_hum=%0.1f adc_v=%0.3f\r",localtime.tm_year-100,localtime.tm_mon,localtime.tm_mday,localtime.tm_hour,localtime.tm_min,localtime.tm_sec,de,ba,T_sht20,H_sht20,adc);
			
			PPRINTF("%s",printf_buff);			
		}	
		else if(de==0x07 || de==0x08)
		{
			if(de==0x07)
			{
			  cou=(buff[3]>>16&0xffff);
			}
			else
				cou=buff[3];
			
			sprintf(printf_buff,"%d/%d/%d %02d:%02d:%02d %d %d sht_temp=%0.2f sht_hum=%0.1f count_times=%d\r",localtime.tm_year-100,localtime.tm_mon,localtime.tm_mday,localtime.tm_hour,localtime.tm_min,localtime.tm_sec,de,ba,T_sht20,H_sht20,cou);
			
			PPRINTF("%s",printf_buff);					
		}
		else
		{
			sprintf(printf_buff,"%d/%d/%d %02d:%02d:%02d %d %d sht_temp=%0.2f sht_hum=%0.1f\r",localtime.tm_year-100,localtime.tm_mon,localtime.tm_mday,localtime.tm_hour,localtime.tm_min,localtime.tm_sec,de,ba,T_sht20,H_sht20);
			
			PPRINTF("%s",printf_buff);
		}
	
			PPRINTF("\n\r");
		
		buff[0]=0;
		buff[1]=0;
		buff[2]=0;
		buff[3]=0;
  }
	
	return 1;
}

uint8_t read_data_on_flash_buff_last_sets_data(uint32_t address_start,uint16_t last_sets_data)
{
	uint32_t buff[4];
	uint32_t timestamp;
	
	uint8_t de;
	uint16_t illu,ba,cou;
	uint8_t le;	
	float T_sht20,H_sht20,T_ds18b20;
	float adc;
	short t1,h1,t2;
	
	__IO uint32_t add1=address_start;

	for (uint16_t j=1;j <= last_sets_data;j++)
//	while (add1 < end_address)
  {
		PPRINTF("%d ",j);

		for(int i=0;i<4;i++)
		{
      buff[i] = *(__IO uint32_t *)add1;
      add1 = add1 + 4;
			if(add1>=FLASH_SENSOR_DATA_END_ADDR)
			{
				add1=FLASH_SENSOR_DATA_START_ADDR;
			}
		}
		
    timestamp=buff[0];
		
		struct tm localtime;
		SysTimeLocalTime( timestamp, &localtime );
		
		de=buff[1]>>16 & 0xff;
		
		ba=(buff[1]>>8&0xff)<<8 | (buff[1]&0xff);
		
		t1=(((buff[2]>>24&0xff)<<8) | (buff[2]>>16&0xff));
		if(t1<0)
		{
			T_sht20=(~t1+1)/-100.0;
		}
		else
		  T_sht20=t1/100.0;
		
		h1=(((buff[2]>>8&0xff)<<8) | (buff[2]&0xff));
		if(h1<0)
		{
			H_sht20=(~h1+1)/-10.0;
		}
		else
			H_sht20=h1/10.0;
		
		/*
		 dev=0x01,ds18b20
		 */
		if(de==0x00)
		{
			if(ba!=0)
			{
			  sprintf(printf_buff,"%d/%d/%d %02d:%02d:%02d %d %d sht_temp=%0.2f sht_hum=%0.1f\r",localtime.tm_year-100,localtime.tm_mon,localtime.tm_mday,localtime.tm_hour,localtime.tm_min,localtime.tm_sec,de,ba,T_sht20,H_sht20);
			
			  PPRINTF("%s",printf_buff);
			}
		}
		else if(de==0x01 || de==0x09)
		{
			t2=((buff[3]>>24&0xff)<<8) | (buff[3]>>16&0xff);
			if(t2<0)
			{
				T_ds18b20=(~t2+1)/-100.0;
			}
			else
				T_ds18b20=t2/100.0;
			
			sprintf(printf_buff,"%d/%d/%d %02d:%02d:%02d %d %d sht_temp=%0.2f sht_hum=%0.1f ds_temp=%0.2f\r",localtime.tm_year-100,localtime.tm_mon,localtime.tm_mday,localtime.tm_hour,localtime.tm_min,localtime.tm_sec,de,ba,T_sht20,H_sht20,T_ds18b20);
			
			PPRINTF("%s",printf_buff);
	  }
		else if(de==0x04)
		{
			le=buff[3]>>24&0xff;
//			ex=buff[3]>>16&0xff;
			
			if(le==0x00)
			{
				sprintf(printf_buff,"%d/%d/%d %02d:%02d:%02d %d %d sht_temp=%0.2f sht_hum=%0.1f level:low\r",localtime.tm_year-100,localtime.tm_mon,localtime.tm_mday,localtime.tm_hour,localtime.tm_min,localtime.tm_sec,de,ba,T_sht20,H_sht20);
			}	
      else if(le==0x01)
			{
				sprintf(printf_buff,"%d/%d/%d %02d:%02d:%02d %d %d sht_temp=%0.2f sht_hum=%0.1f level:high\r",localtime.tm_year-100,localtime.tm_mon,localtime.tm_mday,localtime.tm_hour,localtime.tm_min,localtime.tm_sec,de,ba,T_sht20,H_sht20);
			}		
			
			PPRINTF("%s",printf_buff);			
		}		
		else if(de==0x05)
		{
			illu=buff[3]>>16&0xffff;
			
			sprintf(printf_buff,"%d/%d/%d %02d:%02d:%02d %d %d sht_temp=%0.2f sht_hum=%0.1f illu_lux=%d\r",localtime.tm_year-100,localtime.tm_mon,localtime.tm_mday,localtime.tm_hour,localtime.tm_min,localtime.tm_sec,de,ba,T_sht20,H_sht20,illu);
			
			PPRINTF("%s",printf_buff);				
		}		
		else if(de==0x06)
		{
			adc=(buff[3]>>16&0xffff)/1000.0;
			
			sprintf(printf_buff,"%d/%d/%d %02d:%02d:%02d %d %d sht_temp=%0.2f sht_hum=%0.1f adc_v=%0.3f\r",localtime.tm_year-100,localtime.tm_mon,localtime.tm_mday,localtime.tm_hour,localtime.tm_min,localtime.tm_sec,de,ba,T_sht20,H_sht20,adc);
			
			PPRINTF("%s",printf_buff);			
		}	
		else if(de==0x07 || de==0x08)
		{
			if(de==0x07)
			{
			  cou=(buff[3]>>16&0xffff);
			}
			else
				cou=buff[3];
			
			sprintf(printf_buff,"%d/%d/%d %02d:%02d:%02d %d %d sht_temp=%0.2f sht_hum=%0.1f count_times=%d\r",localtime.tm_year-100,localtime.tm_mon,localtime.tm_mday,localtime.tm_hour,localtime.tm_min,localtime.tm_sec,de,ba,T_sht20,H_sht20,cou);
			
			PPRINTF("%s",printf_buff);					
		}		
		else
		{
			sprintf(printf_buff,"%d/%d/%d %02d:%02d:%02d %d %d sht_temp=%0.2f sht_hum=%0.1f\r",localtime.tm_year-100,localtime.tm_mon,localtime.tm_mday,localtime.tm_hour,localtime.tm_min,localtime.tm_sec,de,ba,T_sht20,H_sht20);
			
			PPRINTF("%s",printf_buff);
		}
	
			PPRINTF("\n\r");
		
		buff[0]=0;
		buff[1]=0;
		buff[2]=0;
		buff[3]=0;
  }
	
	return 1;
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
