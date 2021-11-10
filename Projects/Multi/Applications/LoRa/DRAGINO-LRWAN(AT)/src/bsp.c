 /******************************************************************************
  * @file    bsp.c
  * @author  MCD Application Team
  * @version V1.1.4
  * @date    08-January-2018
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
#include <string.h>
#include <stdlib.h>
#include "hw.h"
#include "timeServer.h"
#include "bsp.h"
#include "lora.h"
#include "RX8010.h"
#include "gpio_exti.h"
#include "ds18b20.h"
#include "bh1750.h"
#include "delay.h"
#include "iwdg.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions ---------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

//static GPIO_InitTypeDef  GPIO_InitStruct;
bool first_read=0;
bool connect_flags=0;
bool connect_flags_temp=0;
extern uint16_t batteryLevel_mV;
extern uint8_t Ext;
extern bool ds18b20_connect_status;
extern uint16_t power_time;
extern uint8_t exitintmode;
extern uint8_t exitmode;
extern bool connect_flags;
extern bool level_status;
static __IO uint16_t AD_code1=0;
static void ADC_Dxpd(uint16_t adc_nums[]);
static uint16_t ADC_Average(void);

void BSP_sensor_Read( sensor_t *sensor_data )
{
	IWDG_Refresh();		
	if(Ext==0x01 || Ext==0x09)
	{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_SET);//open
		DelayMs(10);//Required
	  sensor_data->temp_ds=DS18B20_GetTemp_SkipRom();
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_RESET);//close
	}
	else if((Ext==0x04)||(Ext==0x07)||(Ext==0x08))
	{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_SET);//open
		check_connect();
		level_status=HAL_GPIO_ReadPin(GPIO_Exit_PORT,GPIO_Exit_PIN);		
	}
	else if(Ext==0x05)
	{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_SET);//open
		I2C_IoInit();
		sensor_data->illuminance=bh1750_read();
		if(first_read==0)
		{
			sensor_data->illuminance=bh1750_read();
			first_read=1;			
		}			
		I2C_DoInit();		
	}
	else if(Ext==0x06)
	{
		check_connect();
		if(connect_flags==1)
		{
			sensor_data->adc_PA4=65535.0;
		}
		else
		{
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_SET);//open
			DelayMs(200+power_time);
			HW_GetBatteryLevel( );	
			AD_code1=ADC_Average();  //PA4		
			sensor_data->adc_PA4=AD_code1*batteryLevel_mV/4095;
			if(sensor_data->adc_PA4<=10.0)
			{
				sensor_data->adc_PA4=0.0;
			}
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_RESET);//close	
		}		
	}
	else
	{
    ds18b20_connect_status=0;
	}

	connect_flags_temp=connect_flags;
	if((Ext>=0x04)&&(Ext<=0x08))
	{
		if(connect_flags_temp==1)
		{
			ds18b20_connect_status=0;		
		}
		else if(connect_flags_temp==0)
		{
			ds18b20_connect_status=1;				
		}
	}
	connect_flags=0;	
	
	sensor_data->temp_sht=SHT20_RT();
	sensor_data->hum_sht=SHT20_RH();
}

void  BSP_sensor_Init( void  )
{
	EEPROM_Read_Config();
	
  BSP_RX8010_Init();
	
	GPIO_EXTI_IoInit();
	Led_IoInit();
	
	GPIO_RX8010_timer_interrupt_IoInit();
	
	if(Ext==0x04)
	{
		switch(exitintmode)
		{
			case 1:	
			GPIO_Exit_RISING_FALLINGInit();
			break;
			case 2:	
			GPIO_Exit_FALLINGInit();	
			break;	
			case 3:	
			GPIO_Exit_RISINGInit();		
			break;	
			default:
			break;							
		}
	}
	else if(Ext==0x07||Ext==0x08)
	{
		switch(exitmode)
		{
				case 0:	
				GPIO_Exit_FALLINGInit();		
				break;	
				case 1:	
				GPIO_Exit_RISINGInit();		
				break;	
				default:
				break;					
		}				
	}
}

static void ADC_Dxpd(uint16_t adc_nums[])
{
	int i, j, temp, isSorted;  
	for(i=0; i<6-1; i++)
	{
		isSorted = 1;  
		for(j=0; j<6-1-i; j++)
		{
			if(adc_nums[j] > adc_nums[j+1])
			{
				temp = adc_nums[j];
				adc_nums[j] = adc_nums[j+1];
				adc_nums[j+1] = temp;
				isSorted = 0; 
			}
		}
		if(isSorted) break;
	}
}

static uint16_t ADC_Average(void)
{
	uint16_t sum = 0;
	uint16_t adc_number[6];
	
	for(uint8_t j=0; j<6; j++)
	{
     adc_number[j]=HW_AdcReadChannel( ADC_Channel_IN4 );
	}
	ADC_Dxpd(adc_number);
	
	for(uint8_t i=1; i<5; i++)
	{
		sum = sum + adc_number[i];
	}
	
	return sum/4;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
