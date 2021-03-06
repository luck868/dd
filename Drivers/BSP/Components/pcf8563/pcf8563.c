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
//#define I2C_TIMING    0x10A13E56 /* 100 kHz with analog Filter ON, Rise Time 400ns, Fall Time 100ns */ 
#define I2C_TIMING      0x00B1112E /* 400 kHz with analog Filter ON, Rise Time 250ns, Fall Time 100ns */
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#define I2C_WriteAddress 0x64
#define I2C_ReadAddress  0x65

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
  /* Infinite loop */
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
	
  /* NVIC for I2Cx */
  HAL_NVIC_SetPriority(I2Cx_IRQn, 0, 1);
  HAL_NVIC_EnableIRQ(I2Cx_IRQn);
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

  /*##-2- Disable peripherals and GPIO Clocks #################################*/
  /* Configure I2C Tx as alternate function  */
  HAL_GPIO_DeInit(I2Cx_SCL_GPIO_PORT, I2Cx_SCL_PIN);
  /* Configure I2C Rx as alternate function  */
  HAL_GPIO_DeInit(I2Cx_SDA_GPIO_PORT, I2Cx_SDA_PIN);
}

void RX8010_set_date(uint8_t year,uint8_t month,uint8_t day,uint8_t hour,uint8_t min,uint8_t sec)
{
	  uint8_t txdata[14]={0x10,0x00,0x11,0x00,0x12,0x00,0x14,0x00,0x15,0x00,0x16,0x00};

	  txdata[1]=HEX_TO_BCD(sec);
		txdata[3]=HEX_TO_BCD(min);
		txdata[5]=HEX_TO_BCD(hour);
//		txdata[7]=HEX_TO_BCD(week);
		txdata[7]=HEX_TO_BCD(day);
		txdata[9]=HEX_TO_BCD(month);
		txdata[11]=HEX_TO_BCD(year);
		
		for(int i=0;i<12;i+=2)
	  {
		  while(HAL_I2C_Master_Transmit(&I2cHandle,I2C_WriteAddress,&txdata[i],2,5000) != HAL_OK)
      {
        if(HAL_I2C_GetError(&I2cHandle) != HAL_I2C_ERROR_AF)
        {}
      }
//		  HAL_Delay(1);
	  }
}

uint8_t RX8010_read_S(void)
{
	  uint8_t txdata[1]={0x10};
		uint8_t rxdata[1];
		uint8_t value;
	
		while(HAL_I2C_Master_Transmit(&I2cHandle,I2C_WriteAddress,txdata,1,1000) != HAL_OK)
    {
        if(HAL_I2C_GetError(&I2cHandle) != HAL_I2C_ERROR_AF)
        {}
    }
		
		while(HAL_I2C_Master_Receive(&I2cHandle,I2C_ReadAddress,rxdata,1,1000) != HAL_OK)
    {
        if(HAL_I2C_GetError(&I2cHandle) != HAL_I2C_ERROR_AF)
        {}
    }
	value=BCD_TO_HEX(rxdata[0]&0x7F);
		
	return value;
}

uint8_t RX8010_read_M(void)
{
	  uint8_t txdata[1]={0x11};
		uint8_t rxdata[1];
		uint8_t value;
	
		while(HAL_I2C_Master_Transmit(&I2cHandle,I2C_WriteAddress,txdata,1,1000) != HAL_OK)
    {
        if(HAL_I2C_GetError(&I2cHandle) != HAL_I2C_ERROR_AF)
        {}
    }
		
		while(HAL_I2C_Master_Receive(&I2cHandle,I2C_ReadAddress,rxdata,1,1000) != HAL_OK)
    {
        if(HAL_I2C_GetError(&I2cHandle) != HAL_I2C_ERROR_AF)
        {}
    }
	value=BCD_TO_HEX(rxdata[0]&0x7F);

	return value;
}

uint8_t RX8010_read_H(void)
{
	  uint8_t txdata[1]={0x12};
		uint8_t rxdata[1];
		uint8_t value;
	
		while(HAL_I2C_Master_Transmit(&I2cHandle,I2C_WriteAddress,txdata,1,1000) != HAL_OK)
    {
        if(HAL_I2C_GetError(&I2cHandle) != HAL_I2C_ERROR_AF)
        {}
    }
		
		while(HAL_I2C_Master_Receive(&I2cHandle,I2C_ReadAddress,rxdata,1,1000) != HAL_OK)
    {
        if(HAL_I2C_GetError(&I2cHandle) != HAL_I2C_ERROR_AF)
        {}
    }
	value=BCD_TO_HEX(rxdata[0]&0x3F);

	return value;
}

uint8_t RX8010_read_Day(void)
{
	  uint8_t txdata[1]={0x14};
		uint8_t rxdata[1];
		uint8_t value;
	
		while(HAL_I2C_Master_Transmit(&I2cHandle,I2C_WriteAddress,txdata,1,1000) != HAL_OK)
    {
        if(HAL_I2C_GetError(&I2cHandle) != HAL_I2C_ERROR_AF)
        {}
    }
		
		while(HAL_I2C_Master_Receive(&I2cHandle,I2C_ReadAddress,rxdata,1,1000) != HAL_OK)
    {
        if(HAL_I2C_GetError(&I2cHandle) != HAL_I2C_ERROR_AF)
        {}
    }
	value=BCD_TO_HEX(rxdata[0]&0x3f);

	return value;
}

uint8_t RX8010_read_Week(void)
{
	  uint8_t txdata[1]={0x13};
		uint8_t rxdata[1];
		uint8_t value;
	
		while(HAL_I2C_Master_Transmit(&I2cHandle,I2C_WriteAddress,txdata,1,1000) != HAL_OK)
    {
        if(HAL_I2C_GetError(&I2cHandle) != HAL_I2C_ERROR_AF)
        {}
    }
		
		while(HAL_I2C_Master_Receive(&I2cHandle,I2C_ReadAddress,rxdata,1,1000) != HAL_OK)
    {
        if(HAL_I2C_GetError(&I2cHandle) != HAL_I2C_ERROR_AF)
        {}
    }
	value=rxdata[0]&0x7f;

	return value;
}

uint8_t RX8010_read_Month(void)
{
	  uint8_t txdata[1]={0x15};
		uint8_t rxdata[1];
		uint8_t value;
	
		while(HAL_I2C_Master_Transmit(&I2cHandle,I2C_WriteAddress,txdata,1,1000) != HAL_OK)
    {
        if(HAL_I2C_GetError(&I2cHandle) != HAL_I2C_ERROR_AF)
        {}
    }
		
		while(HAL_I2C_Master_Receive(&I2cHandle,I2C_ReadAddress,rxdata,1,1000) != HAL_OK)
    {
        if(HAL_I2C_GetError(&I2cHandle) != HAL_I2C_ERROR_AF)
        {}
    }
	value=BCD_TO_HEX(rxdata[0]&0x1f);

	return value;
}

uint8_t RX8010_read_Year(void)
{
	  uint8_t txdata[1]={0x16};
		uint8_t rxdata[1];
		uint8_t value;
	
		while(HAL_I2C_Master_Transmit(&I2cHandle,I2C_WriteAddress,txdata,1,1000) != HAL_OK)
    {
        if(HAL_I2C_GetError(&I2cHandle) != HAL_I2C_ERROR_AF)
        {}
    }
		
		while(HAL_I2C_Master_Receive(&I2cHandle,I2C_ReadAddress,rxdata,1,1000) != HAL_OK)
    {
        if(HAL_I2C_GetError(&I2cHandle) != HAL_I2C_ERROR_AF)
        {}
    }
	value=BCD_TO_HEX(rxdata[0]&0xff);

	return value;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

