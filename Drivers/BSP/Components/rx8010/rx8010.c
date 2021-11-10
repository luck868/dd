 /******************************************************************************
  * @file    RX8010.c
  * @author  MCD Application Team
  * @version V1.1.4
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
#include "RX8010.h"
#include "timeServer.h"
#include "delay.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions ---------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
/* I2C handler declaration */
I2C_HandleTypeDef I2cHandle;

/* I2C TIMING Register define when I2C clock source is SYSCLK */
/* I2C TIMING is calculated in case of the I2C Clock source is the SYSCLK = 32 MHz */
#define I2C_TIMING    0x10A13E56 /* 100 kHz with analog Filter ON, Rise Time 400ns, Fall Time 100ns */ 
//#define I2C_TIMING      0x00B1112E /* 400 kHz with analog Filter ON, Rise Time 250ns, Fall Time 100ns */
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#define RX8010_WriteAddress 0x64
#define RX8010_ReadAddress  0x65

#define BCD_TO_HEX(bcd) ((((bcd)>>4)*10)+((bcd)&0x0F))
#define HEX_TO_BCD(hex) ((((hex)/10)<<4)+((hex)%10))

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions ---------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
void  BSP_RX8010_Init( void )
{
  /*##-1- Configure the I2C peripheral ######################################*/
  I2cHandle.Instance              = I2Cx;
  I2cHandle.Init.Timing           = I2C_TIMING;
  I2cHandle.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
  I2cHandle.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
  I2cHandle.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  I2cHandle.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
  I2cHandle.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;  
  I2cHandle.Init.OwnAddress1      = 0xF0;
  I2cHandle.Init.OwnAddress2      = 0xFE;
  
  if(HAL_I2C_Init(&I2cHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /* Enable the Analog I2C Filter */
  HAL_I2CEx_ConfigAnalogFilter(&I2cHandle,I2C_ANALOGFILTER_ENABLE);
	
  /* Enable the Digital I2C Filter */	
	HAL_I2CEx_ConfigDigitalFilter(&I2cHandle, 0);
}
/**
  * @brief I2C MSP Initialization 
  *        This function configures the hardware resources used in this example: 
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration  
  *           - DMA configuration for transmission request by peripheral 
  *           - NVIC configuration for DMA interrupt request enable
  * @param hi2c: I2C handle pointer
  * @retval None
  */
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
  GPIO_InitTypeDef  GPIO_InitStruct;
  RCC_PeriphCLKInitTypeDef  RCC_PeriphCLKInitStruct;
  
  /*##-1- Configure the I2C clock source. The clock is derived from the SYSCLK #*/
  RCC_PeriphCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2Cx;
  RCC_PeriphCLKInitStruct.I2c1ClockSelection = RCC_I2CxCLKSOURCE_SYSCLK;
  HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct);

  /*##-2- Enable peripherals and GPIO Clocks #################################*/
  /* Enable GPIO TX/RX clock */
  I2Cx_SCL_GPIO_CLK_ENABLE();
  I2Cx_SDA_GPIO_CLK_ENABLE();
  /* Enable I2Cx clock */
  I2Cx_CLK_ENABLE();
  
  /*##-3- Configure peripheral GPIO ##########################################*/  
  /* I2C TX GPIO pin configuration  */
  GPIO_InitStruct.Pin       = I2Cx_SCL_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = I2Cx_SCL_SDA_AF;
  HAL_GPIO_Init(I2Cx_SCL_GPIO_PORT, &GPIO_InitStruct);
    
  /* I2C RX GPIO pin configuration  */
  GPIO_InitStruct.Pin       = I2Cx_SDA_PIN;
  GPIO_InitStruct.Alternate = I2Cx_SCL_SDA_AF;
  HAL_GPIO_Init(I2Cx_SDA_GPIO_PORT, &GPIO_InitStruct);
}
/**
  * @brief I2C MSP De-Initialization 
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  *          - Revert GPIO, DMA and NVIC configuration to their default state
  * @param hi2c: I2C handle pointer
  * @retval None
  */
void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c)
{
  /*##-1- Reset peripherals ##################################################*/
  I2Cx_FORCE_RESET();
  I2Cx_RELEASE_RESET();

	I2Cx_CLK_DISABLE();
  /*##-2- Disable peripherals and GPIO Clocks #################################*/
  /* Configure I2C Tx as alternate function  */
  HAL_GPIO_DeInit(I2Cx_SCL_GPIO_PORT, I2Cx_SCL_PIN);
  /* Configure I2C Rx as alternate function  */
  HAL_GPIO_DeInit(I2Cx_SDA_GPIO_PORT, I2Cx_SDA_PIN);
}

float SHT20_RH(void)
{
	uint8_t check_number=0,check_crc=0;	
	uint8_t txdata[1]={0xf5};//Humidity measurement
  uint8_t rxdata[3];
	uint16_t AD_code;
	float hum;
  bool read_status=1;
		
	do
	{
		read_status=1;
		uint32_t currentTime = TimerGetCurrentTime();
		while(HAL_I2C_Master_Transmit(&I2cHandle,0x80,txdata,1,1000) != HAL_OK)
		{
			if(TimerGetElapsedTime(currentTime) >= 1000)
			{
				HAL_I2CEx_ConfigDigitalFilter(&I2cHandle, 0);
				read_status=0;
				break;
			}
		}

		if(read_status==1)
		{
			DelayMs(30);
			currentTime = TimerGetCurrentTime();
			while(HAL_I2C_Master_Receive(&I2cHandle,0x81,rxdata,3,1000) != HAL_OK)
			{
				if(TimerGetElapsedTime(currentTime) >= 1000)
				{
					HAL_I2CEx_ConfigDigitalFilter(&I2cHandle, 0);
					read_status=0;
					break;
				}
			}
			
      if((read_status==1)&&((SHT20_CheckSum_CRC8(rxdata))==1))	
			{
				check_crc=4;
			}			
      else 
      {
				check_crc++;
			}		
		}
		check_number++;
	}while((read_status==0)&&(check_number<=3)&&(check_crc<=3));

	if(read_status==1)
	{
		AD_code=(rxdata[0]<<8)+rxdata[1];
		AD_code &=~0x0003;
		hum=(AD_code*125.0/65536.0)-6.0;
			
		if(hum>100)
		{
			hum=100;
		}
		else if(hum<0)
		{
			hum=0;
		}
	}
	else
	{
		hum=3276.7;
	}
	
	return hum;
}

float SHT20_RT(void)
{
	uint8_t check_number=0,check_crc=0;	
	uint8_t txdata[1]={0xf3};//Temperature measurement
	uint8_t rxdata[3];
	uint16_t AD_code;
	float tem;
	bool read_status=1;
		
	do
	{	
		read_status=1;
	  uint32_t currentTime = TimerGetCurrentTime();		
		while(HAL_I2C_Master_Transmit(&I2cHandle,0x80,txdata,1,1000) != HAL_OK)
    {
			  if(TimerGetElapsedTime(currentTime) >= 1000)
				{
					HAL_I2CEx_ConfigDigitalFilter(&I2cHandle, 0);
					read_status=0;
					break;
				}
    }

		if(read_status==1)
		{
			DelayMs(90);			
			currentTime = TimerGetCurrentTime();		
			while(HAL_I2C_Master_Receive(&I2cHandle,0x81,rxdata,3,1000) != HAL_OK)
			{
			  if(TimerGetElapsedTime(currentTime) >= 1000)
				{	
					HAL_I2CEx_ConfigDigitalFilter(&I2cHandle, 0);
					read_status=0;					
					break;
				}
      }
			
      if((read_status==1)&&((SHT20_CheckSum_CRC8(rxdata))==1))	
			{
				check_crc=4;
			}			
      else 
      {
				check_crc++;
			}				
		}
		check_number++;
	}while((read_status==0)&&(check_number<=3)&&(check_crc<=3));
	
	if(read_status==1)
	{
		AD_code=(rxdata[0]<<8)+rxdata[1];
		AD_code &=~0x0003;
		tem=(AD_code*175.72/65536.0)-46.85;
		
		if(tem>125)
		{
			tem=125;
		}
		else if(tem<-40)
		{
			tem=-40;
		}
	}
	else 
	{
		tem=327.67;
	}
		
  return tem;
}

uint8_t SHT20_CheckSum_CRC8(uint8_t* Result) 
{
	uint8_t data[2];
	data[0] = Result[0];
	data[1] = Result[1];

	uint32_t POLYNOMIAL = 0x131;
	uint8_t crc = 0;
	uint8_t bit = 0;
	uint8_t byteCtr = 0;

	//calculates 8-Bit checksum with given polynomial
	for (byteCtr = 0; byteCtr < 2; ++byteCtr)
	{
		crc ^= (data[byteCtr]);
		for (bit = 8; bit > 0; --bit)
		{
			if (crc & 0x80) crc = (crc << 1) ^ POLYNOMIAL;
			else crc = (crc << 1);
		}
	}
	if (crc == Result[2]) {
		return 1;
	}
	else {
		return 0;
	}
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

