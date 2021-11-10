/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: contains all hardware driver

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
 /******************************************************************************
  * @file    flash_eraseprogram.h
  * @author  MCD Application Team
  * @version V1.1.2
  * @date    01-June-2017
  * @brief   contains all hardware driver
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FLASH_ERASEPROGRAM_H__
#define __FLASH_ERASEPROGRAM_H__

#ifdef __cplusplus
 extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */ 
/**
 * @brief  initialises the 
 *
 * @note
 * @retval None
 */
#define FLASH_USER_START_ADDR_CONFIG   (FLASH_BASE + FLASH_PAGE_SIZE * 800)        /* Start @ of user Flash area store config */
#define FLASH_USER_END_ADDR            (FLASH_USER_START_ADDR + FLASH_PAGE_SIZE)   /* End @ of user Flash area store key*/

#define FLASH_USER_START_ADDR_KEY      (FLASH_BASE + FLASH_PAGE_SIZE * 802)

#define EEPROM_USER_START_ADDR_KEY     (DATA_EEPROM_BASE+0x04)
#define EEPROM_USER_START_ADDR_CONFIG  (DATA_EEPROM_BASE+0x04*24)
#define EEPROM_USER_END_ADDR_CONFIG    (EEPROM_USER_START_ADDR_CONFIG+0x04*25)

/**
 * store sensor data in flash page810~page1210,total 400pages
 * 0x8019500 ~ 0x8025d00
 */	 
#define FLASH_SENSOR_DATA_START_ADDR   (FLASH_BASE + FLASH_PAGE_SIZE * 810)             
#define FLASH_SENSOR_DATA_END_ADDR     (FLASH_SENSOR_DATA_START_ADDR + FLASH_PAGE_SIZE*400)

void  FLASH_erase(uint32_t page_address);
void  FLASH_program(uint32_t add, uint32_t *data, uint8_t count);

uint8_t read_data_on_flash_buff(uint16_t address_start,uint16_t address_end);
uint8_t read_data_on_flash_buff_last_sets_data(uint32_t address_start,uint16_t last_sets_data);

void  FLASH_erase_all_sensor_data_storage(uint32_t page_address);
uint32_t find_addr(uint32_t addr);
uint32_t store_sensor_data(uint32_t add,uint32_t *sensor_data);

void EEPROM_erase_one_address(uint32_t address);
void EEPROM_erase_lora_config(void);
void EEPROM_program(uint32_t add, uint32_t *data, uint8_t count);

uint32_t find_addr_first_after_systemreset(uint32_t addr);

#ifdef __cplusplus
}
#endif

#endif /* __FLASH_ERASEPROGRAM_H__ */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/