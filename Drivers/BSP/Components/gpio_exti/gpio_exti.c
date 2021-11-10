 /******************************************************************************
  * @file    ogpio_exti.c
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
#include "gpio_exti.h"

extern bool connect_flags;
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions ---------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
void  GPIO_EXTI_IoInit( void  )
{
	GPIO_InitTypeDef GPIO_InitStruct={0};
	GPIO_USERKEY_CLK_ENABLE();
	
	GPIO_InitStruct.Mode =GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Pin = GPIO_USERKEY_PIN;

  HW_GPIO_Init( GPIO_USERKEY_PORT, GPIO_USERKEY_PIN, &GPIO_InitStruct );
	
	/* Enable and set EXTI lines 4 to 15 Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
}

void  GPIO_RX8010_timer_interrupt_IoInit( void  )
{
	GPIO_InitTypeDef GPIO_InitStruct={0};
	GPIO_RX8010_interrupt_CLK_ENABLE();
	
	GPIO_InitStruct.Mode =GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Pin = GPIO_RX8010_interrupt_USERKEY_PIN;

  HW_GPIO_Init( GPIO_RX8010_interrupt_USERKEY_PORT, GPIO_RX8010_interrupt_USERKEY_PIN, &GPIO_InitStruct );
}

void  GPIO_EXTI_IoDeInit( void  )
{
	GPIO_InitTypeDef GPIO_InitStruct={0};
	GPIO_InitStruct.Mode =GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;

  HW_GPIO_Init( GPIO_USERKEY_PORT, GPIO_USERKEY_PIN, &GPIO_InitStruct );
}

void Led_IoInit(void)
{
	GPIO_InitTypeDef GPIO_InitStruct={0};
	LED_CLK_ENABLE();
	
	GPIO_InitStruct.Mode =GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Pin = LED_BLUE_PIN|LED_GREEN_PIN|LED_RED_PIN;

  HW_GPIO_Init( LED_RGB_PORT, LED_BLUE_PIN|LED_GREEN_PIN|LED_RED_PIN, &GPIO_InitStruct );
	
	HAL_GPIO_WritePin(LED_RGB_PORT,LED_BLUE_PIN,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_RGB_PORT,LED_RED_PIN,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_RGB_PORT,LED_GREEN_PIN,GPIO_PIN_RESET);
	
	__HAL_RCC_GPIOB_CLK_ENABLE();
	GPIO_InitStruct.Mode =GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Pin = GPIO_PIN_5;

  HW_GPIO_Init( GPIOB, GPIO_PIN_5, &GPIO_InitStruct );
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_RESET);
}

void  GPIO_Exit_RISING_FALLINGInit( void  )
{
	GPIO_InitTypeDef GPIO_InitStruct={0};
	GPIO_Exit_CLK_ENABLE();
  	
	GPIO_InitStruct.Mode =GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Pin = GPIO_Exit_PIN ;

  HW_GPIO_Init( GPIO_Exit_PORT, GPIO_Exit_PIN , &GPIO_InitStruct );
}

void  GPIO_Exit_FALLINGInit( void  )
{
	GPIO_InitTypeDef GPIO_InitStruct={0};
	GPIO_Exit_CLK_ENABLE();
  	
	GPIO_InitStruct.Mode =GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Pin = GPIO_Exit_PIN ;

  HW_GPIO_Init( GPIO_Exit_PORT, GPIO_Exit_PIN , &GPIO_InitStruct );
}

void  GPIO_Exit_RISINGInit( void  )
{
	GPIO_InitTypeDef GPIO_InitStruct={0};
	GPIO_Exit_CLK_ENABLE();
  	
	GPIO_InitStruct.Mode =GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Pin = GPIO_Exit_PIN ;

  HW_GPIO_Init( GPIO_Exit_PORT, GPIO_Exit_PIN , &GPIO_InitStruct );
}

void  GPIO_EXTI4_IoDeInit( void  )
{
	GPIO_InitTypeDef GPIO_InitStruct={0};
	
  HW_GPIO_Init( GPIO_Exit_PORT, GPIO_Exit_PIN , &GPIO_InitStruct  );
}

void check_connect(void)
{
	__GPIOA_CLK_ENABLE();

	GPIO_InitTypeDef GPIO_InitStruct1={0};	
	GPIO_InitStruct1.Pin = GPIO_PIN_9;	
  GPIO_InitStruct1.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct1.Pull  = GPIO_PULLDOWN;
  GPIO_InitStruct1.Speed = GPIO_SPEED_HIGH;
  HW_GPIO_Init( GPIOA, GPIO_PIN_9, &GPIO_InitStruct1 );
	
	GPIO_InitTypeDef GPIO_InitStruct2={0};	
	GPIO_InitStruct2.Pin = GPIO_PIN_10;
	GPIO_InitStruct2.Mode =GPIO_MODE_INPUT;
  GPIO_InitStruct2.Pull = GPIO_NOPULL;
  HW_GPIO_Init( GPIOA, GPIO_PIN_10, &GPIO_InitStruct2 );
	
	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10)==SET)
	{
		connect_flags=1;
	}
	else
	{
		connect_flags=0;	
	}
	
	GPIO_InitTypeDef GPIO_InitStruct3={0};	
  HW_GPIO_Init( GPIOA, GPIO_PIN_9, &GPIO_InitStruct3 );
	
	GPIO_InitTypeDef GPIO_InitStruct4={0};	
  HW_GPIO_Init( GPIOA, GPIO_PIN_10, &GPIO_InitStruct4 );
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
