/*******************************************************************************
 * @file    at.c
 * @author  MCD Application Team
 * @version V1.1.4
 * @date    08-January-2018
 * @brief   at command API
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
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "at.h"
#include "utilities.h"
#include "lora.h"
#include "LoRaMacTest.h"
#include "radio.h"
#include "vcom.h"
#include "tiny_sscanf.h"
#include "version.h"
#include "hw_msp.h"
#include "flash_eraseprogram.h"
#include "timeServer.h"
#include "systime.h"
#include "hw_rtc.h"
#include "rx8010.h"
#include "command.h"
#include "gpio_exti.h"
#include "delay.h"

/* External variables --------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/**
 * @brief Max size of the data that can be received
 */
#define MAX_RECEIVED_DATA 255
extern uint32_t APP_TX_DUTYCYCLE;
extern uint16_t REJOIN_TX_DUTYCYCLE;
extern uint8_t response_level;

bool debug_flags=0;
uint8_t symbtime1_value=0;  //RX1windowtimeout 
uint8_t flag1=0;

uint8_t symbtime2_value=0;  //RX2windowtimeout 
uint8_t flag2=0;
uint8_t dwelltime;
uint8_t parse_flag=0;
uint16_t power_time=0;
uint8_t exitintmode;
uint8_t exitmode;
extern uint32_t password_get;
extern uint8_t Ext;
extern bool sleep_status;

extern __IO uint32_t write_address;
extern bool FDR_status;
extern uint16_t batteryLevel_mV;
extern uint32_t count;

extern TimerEvent_t MacStateCheckTimer;
extern TimerEvent_t TxDelayedTimer;
extern TimerEvent_t AckTimeoutTimer;
extern TimerEvent_t RxWindowTimer1;
extern TimerEvent_t RxWindowTimer2;
extern TimerEvent_t TxTimer;
extern TimerEvent_t ReJoinTimer;
extern TimerEvent_t CalibrationUTCTimer;
extern TimerEvent_t sht20tempcompTimer;
extern TimerEvent_t DS18B20IDTimer;

static TimerEvent_t ATcommandsTimer;
static void OnTimerATcommandsEvent( void );

extern uint32_t LoRaMacState;

extern uint8_t is_PDTA_command;
extern uint8_t is_PLDTA_command;

extern uint8_t work_mode;
extern uint16_t collection_interval;
extern short comp_temp1_sht20,comp_temp2_sht20;

extern uint8_t currentLeapSecond;
extern uint8_t time_synchronization_method;
extern uint8_t time_synchronization_interval;
extern uint8_t pid_flag;
extern uint8_t RX2DR_setting_status;

/* Private macro -------------------------------------------------------------*/
/**
 * @brief Macro to return when an error occurs
 */
#define CHECK_STATUS(status) do {                    \
    ATEerror_t at_status = translate_status(status); \
    if (at_status != AT_OK) { return at_status; }    \
  } while (0)

/* Private variables ---------------------------------------------------------*/
/*!
 * User application data buffer size
 */
#define LORAWAN_APP_DATA_BUFF_SIZE                           64

/*!
 * User application data
 */
static uint8_t AppDataBuff[LORAWAN_APP_DATA_BUFF_SIZE];

/*!
 * User application data structure
 */
static lora_AppData_t AppData={ AppDataBuff,  0 ,0 };
/**
 * @brief Buffer that contains the last received data
 */
static char ReceivedData[MAX_RECEIVED_DATA];

/**
 * @brief Size if the buffer that contains the last received data
 */
static unsigned ReceivedDataSize = 0;

/**
 * @brief Application port the last received data were on
 */
static uint8_t ReceivedDataPort;

/* Private function prototypes -----------------------------------------------*/
/**
 * @brief  Translate a LoRaMacStatus_t into an ATEerror_t
 * @param  the LoRaMacStatus_t status
 * @retval the corresponding ATEerror_t code
 */
static ATEerror_t translate_status(LoRaMacStatus_t status);

/**
 * @brief  Get 16 bytes values in hexa
 * @param  The string containing the 16 bytes, something like ab:cd:01:...
 * @param  The buffer that will contain the bytes read
 * @retval The number of bytes read
 */
static int sscanf_16_hhx(const char *from, uint8_t *pt);

/**
 * @brief  Print 16 bytes as %02x
 * @param  the pointer containing the 16 bytes to print
 * @retval None
 */
static void print_16_02x(uint8_t *pt);

/**
 * @brief  Get 4 bytes values in hexa
 * @param  The string containing the 16 bytes, something like ab:cd:01:21
 * @param  The buffer that will contain the bytes read
 * @retval The number of bytes read
 */
static int sscanf_uint32_as_hhx(const char *from, uint32_t *value);

/**
 * @brief  Print 4 bytes as %02x
 * @param  the value containing the 4 bytes to print
 * @retval None
 */
static void print_uint32_as_02x(uint32_t value);

/**
 * @brief  Print 8 bytes as %02x
 * @param  the pointer containing the 8 bytes to print
 * @retval None
 */
static void print_8_02x(uint8_t *pt);

/**
 * @brief  Print an int
 * @param  the value to print
 * @retval None
 */
static void print_d(int value);

/**
 * @brief  Print an unsigned int
 * @param  the value to print
 * @retval None
 */
static void print_u(unsigned int value);

/* Exported functions ------------------------------------------------------- */

void set_at_receive(uint8_t AppPort, uint8_t* Buff, uint8_t BuffSize)
{
  if (MAX_RECEIVED_DATA <= BuffSize)
    BuffSize = MAX_RECEIVED_DATA;
  memcpy1((uint8_t *)ReceivedData, Buff, BuffSize);
  ReceivedDataSize = BuffSize;
  ReceivedDataPort = AppPort;
}

ATEerror_t at_return_ok(const char *param)
{
  return AT_OK;
}

ATEerror_t at_return_error(const char *param)
{
  return AT_ERROR;
}

ATEerror_t at_DEBUG_run(const char *param)
{
  debug_flags=1;
	PPRINTF("Enter Debug mode\r\n");			
	debug_flags=1;
	
  return AT_OK;	
}
ATEerror_t at_reset(const char *param)
{
  NVIC_SystemReset();
  return AT_OK;
}

ATEerror_t at_sleep(const char *param)
{
  TimerStop(&MacStateCheckTimer);
	TimerStop(&TxDelayedTimer);
	TimerStop(&AckTimeoutTimer);

  TimerStop(&RxWindowTimer1);
  TimerStop(&RxWindowTimer2);
	TimerStop(&TxTimer);
	TimerStop(&ReJoinTimer);
	TimerStop( &CalibrationUTCTimer);
	TimerStop( &sht20tempcompTimer);
	TimerStop( &DS18B20IDTimer );
	
	sleep_status=1;
	
	AT_PRINTF("SLEEP\n\r");
  return AT_OK;
}

ATEerror_t at_sleep_status_get(const char *param)
{
  print_d(sleep_status);
  return AT_OK;
}

ATEerror_t at_FDR(const char *param)
{
	EEPROM_erase_one_address(DATA_EEPROM_BASE);
	EEPROM_erase_lora_config();
	AT_PRINTF("OK\n\r");
	NVIC_SystemReset();
  return AT_OK;
}

ATEerror_t at_DevEUI_get(const char *param)
{
  print_8_02x(lora_config_deveui_get());
  return AT_OK;
}

ATEerror_t at_DevEUI_set(const char *param)
{
  uint8_t DevEUI[8];
  if (tiny_sscanf(param, "%hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx",
                  &DevEUI[0], &DevEUI[1], &DevEUI[2], &DevEUI[3],
                  &DevEUI[4], &DevEUI[5], &DevEUI[6], &DevEUI[7]) != 8)
  {
    return AT_PARAM_ERROR;
  }
  
  lora_config_deveui_set(DevEUI);
  return AT_OK;
}

ATEerror_t at_AppEUI_get(const char *param)
{
  print_8_02x(lora_config_appeui_get());
  return AT_OK;
}

ATEerror_t at_AppEUI_set(const char *param)
{
  uint8_t AppEui[8];
  if (tiny_sscanf(param, "%hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx",
                  &AppEui[0], &AppEui[1], &AppEui[2], &AppEui[3],
                  &AppEui[4], &AppEui[5], &AppEui[6], &AppEui[7]) != 8)
  {
    return AT_PARAM_ERROR;
  }
  
  lora_config_appeui_set(AppEui);
  return AT_OK;
}

ATEerror_t at_DevAddr_set(const char *param)
{
  uint32_t DevAddr;
  if (sscanf_uint32_as_hhx(param, &DevAddr) != 4)
  {
    return AT_PARAM_ERROR;
  }
  lora_config_devaddr_set(DevAddr);
  return AT_OK;
}

ATEerror_t at_DevAddr_get(const char *param)
{
  lora_config_devaddr_get();
  return AT_OK;
}

ATEerror_t at_AppKey_get(const char *param)
{
  print_16_02x(lora_config_appkey_get());
  return AT_OK;
}

ATEerror_t at_AppKey_set(const char *param)
{
  uint8_t AppKey[16];
  if (sscanf_16_hhx(param, AppKey) != 16)
  {
    return AT_PARAM_ERROR;
  }
  
  lora_config_appkey_set(AppKey);
  return AT_OK;
}

ATEerror_t at_NwkSKey_get(const char *param)
{
	print_16_02x(lora_config_nwkskey_get());
  return AT_OK;
}

ATEerror_t at_NwkSKey_set(const char *param)
{
	uint8_t NwkSKey[16];
  if (sscanf_16_hhx(param, NwkSKey) != 16)
  {
    return AT_PARAM_ERROR;
  }
  
  lora_config_nwkskey_set(NwkSKey);
  return AT_OK;
}

ATEerror_t at_AppSKey_get(const char *param)
{
	print_16_02x(lora_config_appskey_get());
  return AT_OK;
}

ATEerror_t at_AppSKey_set(const char *param)
{
	uint8_t AppSKey[16];
  if (sscanf_16_hhx(param, AppSKey) != 16)
  {
    return AT_PARAM_ERROR;
  }
  
  lora_config_appskey_set(AppSKey);
  return AT_OK;
}

ATEerror_t at_ADR_get(const char *param)
{
  MibRequestConfirm_t mib;
  LoRaMacStatus_t status;

  mib.Type = MIB_ADR;
  status = LoRaMacMibGetRequestConfirm(&mib);
  CHECK_STATUS(status);
  print_d(mib.Param.AdrEnable);

  return AT_OK;
}

ATEerror_t at_ADR_set(const char *param)
{
  MibRequestConfirm_t mib;
  LoRaMacStatus_t status;

  mib.Type = MIB_ADR;

  switch (param[0])
  {
    case '0':
    case '1':
      mib.Param.AdrEnable = param[0] - '0';
      status = LoRaMacMibSetRequestConfirm(&mib);
      CHECK_STATUS(status);
      break;
    default:
      return AT_PARAM_ERROR;
  }

  return AT_OK;
}

ATEerror_t at_TransmitPower_get(const char *param)
{
  MibRequestConfirm_t mib;
  LoRaMacStatus_t status;

  mib.Type = MIB_CHANNELS_TX_POWER;
  status = LoRaMacMibGetRequestConfirm(&mib);
  CHECK_STATUS(status);
  print_d(mib.Param.ChannelsTxPower);

  return AT_OK;
}

ATEerror_t at_TransmitPower_set(const char *param)
{
  MibRequestConfirm_t mib;
  LoRaMacStatus_t status;

  mib.Type = MIB_CHANNELS_TX_POWER;
  if (tiny_sscanf(param, "%hhu", &mib.Param.ChannelsTxPower) != 1)
  {
    return AT_PARAM_ERROR;
  }
  status = LoRaMacMibSetRequestConfirm(&mib);
  CHECK_STATUS(status);

  PPRINTF("Attention:Take effect after AT+ADR=0\r\n");
  return AT_OK;
}

ATEerror_t at_DataRate_get(const char *param)
{

  MibRequestConfirm_t mib;
  LoRaMacStatus_t status;

  mib.Type = MIB_CHANNELS_DATARATE;

  status = LoRaMacMibGetRequestConfirm(&mib);
  CHECK_STATUS(status);
  print_d(mib.Param.ChannelsDatarate);

  return AT_OK;
}

ATEerror_t at_DataRate_set(const char *param)
{
  int8_t datarate;
	
  if (tiny_sscanf(param, "%hhu", &datarate) != 1)
  {
    return AT_PARAM_ERROR;
  }

#if defined( REGION_AS923 )
	  if(datarate>=8)
		{
    return AT_PARAM_ERROR;			
		}
#elif defined( REGION_AU915 )
	  if((datarate==7)||(datarate>=14))
		{
    return AT_PARAM_ERROR;			
		}
#elif defined( REGION_CN470 )
	  if(datarate>=6)	
		{
    return AT_PARAM_ERROR;			
		}
#elif defined( REGION_CN779 )
	  if(datarate>=8)
		{
    return AT_PARAM_ERROR;			
		}
#elif defined( REGION_EU433 )
	  if(datarate>=8)
		{
    return AT_PARAM_ERROR;			
		}
#elif defined( REGION_IN865 )
	  if(datarate>=8)
		{
    return AT_PARAM_ERROR;			
		}
#elif defined( REGION_EU868 )
	  if(datarate>=8)
		{
    return AT_PARAM_ERROR;			
		}
#elif defined( REGION_KR920 )
	  if(datarate>=6)
		{
    return AT_PARAM_ERROR;			
		}
#elif defined( REGION_US915 )
	  if(((datarate>=5)&&(datarate<=7))||(datarate>=14))
		{
    return AT_PARAM_ERROR;			
		}
#elif defined( REGION_RU864 )
	  if(datarate>=8)
		{
    return AT_PARAM_ERROR;			
		}	
#elif defined( REGION_KZ865 )
	  if(datarate>=8)
		{
    return AT_PARAM_ERROR;			
		}			
#endif
	
  lora_config_tx_datarate_set(datarate) ;

  PPRINTF("Attention:Take effect after AT+ADR=0\r\n");
  return AT_OK;
}

ATEerror_t at_DutyCycle_set(const char *param)
{
  switch (param[0])
  {
    case '0':
      lora_config_duty_cycle_set(LORA_DISABLE);
      break;
    case '1':
      lora_config_duty_cycle_set(LORA_ENABLE);
      break;
    default:
      return AT_PARAM_ERROR;
  }

  return AT_OK;
}

ATEerror_t at_DutyCycle_get(const char *param)
{
  if (lora_config_duty_cycle_get() == LORA_ENABLE)
    AT_PRINTF("1\r\n");
  else
    AT_PRINTF("0\r\n");

  return AT_OK;
}


ATEerror_t at_PublicNetwork_get(const char *param)
{
  MibRequestConfirm_t mib;
  LoRaMacStatus_t status;

  mib.Type = MIB_PUBLIC_NETWORK;
  status = LoRaMacMibGetRequestConfirm(&mib);
  CHECK_STATUS(status);
  print_d(mib.Param.EnablePublicNetwork);

  return AT_OK;
}

ATEerror_t at_PublicNetwork_set(const char *param)
{
  MibRequestConfirm_t mib;
  LoRaMacStatus_t status;

  mib.Type = MIB_PUBLIC_NETWORK;
  switch (param[0])
  {
    case '0':
    case '1':
      mib.Param.EnablePublicNetwork = param[0] - '0';
      status = LoRaMacMibSetRequestConfirm(&mib);
      CHECK_STATUS(status);
      break;
    default:
      return AT_PARAM_ERROR;
  }

  return AT_OK;
}

ATEerror_t at_Rx2Frequency_get(const char *param)
{
  MibRequestConfirm_t mib;
  LoRaMacStatus_t status;

  mib.Type = MIB_RX2_CHANNEL;
  status = LoRaMacMibGetRequestConfirm(&mib);
  CHECK_STATUS(status);
  print_d(mib.Param.Rx2Channel.Frequency);

  return AT_OK;
}

ATEerror_t at_Rx2Frequency_set(const char *param)
{
  MibRequestConfirm_t mib;
  LoRaMacStatus_t status;

  mib.Type = MIB_RX2_CHANNEL;
  status = LoRaMacMibGetRequestConfirm(&mib);
  CHECK_STATUS(status);

  if (tiny_sscanf(param, "%lu", &mib.Param.Rx2Channel.Frequency) != 1)
  {
    return AT_PARAM_ERROR;
  }

  status = LoRaMacMibSetRequestConfirm(&mib);
  CHECK_STATUS(status);

  return AT_OK;
}

ATEerror_t at_Rx2DataRate_get(const char *param)
{
  MibRequestConfirm_t mib;
  LoRaMacStatus_t status;

  mib.Type = MIB_RX2_CHANNEL;
  status = LoRaMacMibGetRequestConfirm(&mib);
  CHECK_STATUS(status);
  print_d(mib.Param.Rx2Channel.Datarate);

  return AT_OK;
}

ATEerror_t at_Rx2DataRate_set(const char *param)
{
  MibRequestConfirm_t mib;
  LoRaMacStatus_t status;

  mib.Type = MIB_RX2_CHANNEL;

  status = LoRaMacMibGetRequestConfirm(&mib);
  CHECK_STATUS(status);

  if (tiny_sscanf(param, "%hhu", &mib.Param.Rx2Channel.Datarate) != 1)
  {
    return AT_PARAM_ERROR;
  }

	RX2DR_setting_status=1;
  status = LoRaMacMibSetRequestConfirm(&mib);
  CHECK_STATUS(status);

  return AT_OK;
}

ATEerror_t at_Rx1Delay_get(const char *param)
{
  MibRequestConfirm_t mib;
  LoRaMacStatus_t status;

  mib.Type = MIB_RECEIVE_DELAY_1;
  status = LoRaMacMibGetRequestConfirm(&mib);
  CHECK_STATUS(status);
  print_u(mib.Param.ReceiveDelay1);

  return AT_OK;
}

ATEerror_t at_Rx1Delay_set(const char *param)
{
  MibRequestConfirm_t mib;
  LoRaMacStatus_t status;

  mib.Type = MIB_RECEIVE_DELAY_1;
  if (tiny_sscanf(param, "%lu", &mib.Param.ReceiveDelay1) != 1)
  {
    return AT_PARAM_ERROR;
  }
  status = LoRaMacMibSetRequestConfirm(&mib);
  CHECK_STATUS(status);

  return AT_OK;
}

ATEerror_t at_Rx2Delay_get(const char *param)
{
  MibRequestConfirm_t mib;
  LoRaMacStatus_t status;

  mib.Type = MIB_RECEIVE_DELAY_2;
  status = LoRaMacMibGetRequestConfirm(&mib);
  CHECK_STATUS(status);
  print_u(mib.Param.ReceiveDelay2);

  return AT_OK;
}

ATEerror_t at_Rx2Delay_set(const char *param)
{
  MibRequestConfirm_t mib;
  LoRaMacStatus_t status;

  mib.Type = MIB_RECEIVE_DELAY_2;
  if (tiny_sscanf(param, "%lu", &mib.Param.ReceiveDelay2) != 1)
  {
    return AT_PARAM_ERROR;
  }
  status = LoRaMacMibSetRequestConfirm(&mib);
  CHECK_STATUS(status);

  return AT_OK;
}

ATEerror_t at_JoinAcceptDelay1_get(const char *param)
{
  MibRequestConfirm_t mib;
  LoRaMacStatus_t status;

  mib.Type = MIB_JOIN_ACCEPT_DELAY_1;
  status = LoRaMacMibGetRequestConfirm(&mib);
  CHECK_STATUS(status);
  print_u(mib.Param.JoinAcceptDelay1);

  return AT_OK;
}

ATEerror_t at_JoinAcceptDelay1_set(const char *param)
{
  MibRequestConfirm_t mib;
  LoRaMacStatus_t status;

  mib.Type = MIB_JOIN_ACCEPT_DELAY_1;
  if (tiny_sscanf(param, "%lu", &mib.Param.JoinAcceptDelay1) != 1)
  {
    return AT_PARAM_ERROR;
  }
  status = LoRaMacMibSetRequestConfirm(&mib);
  CHECK_STATUS(status);

  PPRINTF("Attention:Take effect after ATZ\r\n");
  return AT_OK;
}

ATEerror_t at_JoinAcceptDelay2_get(const char *param)
{
  MibRequestConfirm_t mib;
  LoRaMacStatus_t status;

  mib.Type = MIB_JOIN_ACCEPT_DELAY_2;
  status = LoRaMacMibGetRequestConfirm(&mib);
  CHECK_STATUS(status);
  print_u(mib.Param.JoinAcceptDelay2);

  return AT_OK;
}

ATEerror_t at_JoinAcceptDelay2_set(const char *param)
{
  MibRequestConfirm_t mib;
  LoRaMacStatus_t status;

  mib.Type = MIB_JOIN_ACCEPT_DELAY_2;
  if (tiny_sscanf(param, "%lu", &mib.Param.JoinAcceptDelay2) != 1)
  {
    return AT_PARAM_ERROR;
  }
  status = LoRaMacMibSetRequestConfirm(&mib);
  CHECK_STATUS(status);

  PPRINTF("Attention:Take effect after ATZ\r\n");
  return AT_OK;
}

ATEerror_t at_NetworkJoinMode_get(const char *param)
{
  print_d((lora_config_otaa_get() == LORA_ENABLE ? 1 : 0));
  return AT_OK;
}

ATEerror_t at_NetworkJoinMode_set(const char *param)
{
  LoraState_t status;

  switch (param[0])
  {
    case '0':
      status = LORA_DISABLE;
      break;
    case '1':
      status = LORA_ENABLE;
      break;
    default:
      return AT_PARAM_ERROR;
  }

  lora_config_otaa_set(status);
	
  PPRINTF("Attention:Take effect after ATZ\r\n");
  return AT_OK;
}

ATEerror_t at_NetworkID_get(const char *param)
{
  MibRequestConfirm_t mib;
  LoRaMacStatus_t status;

  mib.Type = MIB_NET_ID;
  status = LoRaMacMibGetRequestConfirm(&mib);
  CHECK_STATUS(status);
  print_uint32_as_02x(mib.Param.NetID);

  return AT_OK;
}

ATEerror_t at_NetworkID_set(const char *param)
{
  MibRequestConfirm_t mib;
  LoRaMacStatus_t status;

  mib.Type = MIB_NET_ID;
  if (sscanf_uint32_as_hhx(param, &mib.Param.NetID) != 4)
  {
    return AT_PARAM_ERROR;
  }
  status = LoRaMacMibSetRequestConfirm(&mib);
  CHECK_STATUS(status);

  return AT_OK;
}

ATEerror_t at_UplinkCounter_get(const char *param)
{
  MibRequestConfirm_t mib;
  LoRaMacStatus_t status;

  mib.Type = MIB_UPLINK_COUNTER;
  status = LoRaMacMibGetRequestConfirm(&mib);
  CHECK_STATUS(status);
  print_u(mib.Param.UpLinkCounter);

  return AT_OK;
}

ATEerror_t at_UplinkCounter_set(const char *param)
{
  MibRequestConfirm_t mib;
  LoRaMacStatus_t status;

  mib.Type = MIB_UPLINK_COUNTER;
  if (tiny_sscanf(param, "%lu", &mib.Param.UpLinkCounter) != 1)
  {
    return AT_PARAM_ERROR;
  }
  status = LoRaMacMibSetRequestConfirm(&mib);
  CHECK_STATUS(status);

  return AT_OK;
}

ATEerror_t at_DownlinkCounter_get(const char *param)
{
  MibRequestConfirm_t mib;
  LoRaMacStatus_t status;

  mib.Type = MIB_DOWNLINK_COUNTER;
  status = LoRaMacMibGetRequestConfirm(&mib);
  CHECK_STATUS(status);
  print_u(mib.Param.DownLinkCounter);

  return AT_OK;
}

ATEerror_t at_DownlinkCounter_set(const char *param)
{
  MibRequestConfirm_t mib;
  LoRaMacStatus_t status;

  mib.Type = MIB_DOWNLINK_COUNTER;
  if (tiny_sscanf(param, "%lu", &mib.Param.DownLinkCounter) != 1)
  {
    return AT_PARAM_ERROR;
  }
  status = LoRaMacMibSetRequestConfirm(&mib);
  CHECK_STATUS(status);

  return AT_OK;
}

ATEerror_t at_DeviceClass_get(const char *param)
{
  MibRequestConfirm_t mib;
  LoRaMacStatus_t status;

  mib.Type = MIB_DEVICE_CLASS;
  status = LoRaMacMibGetRequestConfirm(&mib);
  CHECK_STATUS(status);
  AT_PRINTF("%c\r\n", 'A' + mib.Param.Class);

  return AT_OK;
}

ATEerror_t at_DeviceClass_set(const char *param)
{
  MibRequestConfirm_t mib;
  LoRaMacStatus_t status;

  mib.Type = MIB_DEVICE_CLASS;
  switch (param[0])
  {
    case 'A':
    case 'B':
    case 'C':
      /* assume CLASS_A == 0, CLASS_B == 1, etc, which is the case for now */
      mib.Param.Class = (DeviceClass_t)(param[0] - 'A');
      status = LoRaMacMibSetRequestConfirm(&mib);
      CHECK_STATUS(status);
      break;
    default:
      return AT_PARAM_ERROR;
  }

  PPRINTF("Attention:Take effect after ATZ\r\n");
  return AT_OK;
}

ATEerror_t at_Join(const char *param)
{
	FDR_status=0;
	
  LORA_Join();

  return AT_OK;
}

ATEerror_t at_NetworkJoinStatus(const char *param)
{
  MibRequestConfirm_t mibReq;
  LoRaMacStatus_t status;

  mibReq.Type = MIB_NETWORK_JOINED;
  status = LoRaMacMibGetRequestConfirm(&mibReq);

  if (status == LORAMAC_STATUS_OK)
  {
    print_d(mibReq.Param.IsNetworkJoined ? 1 : 0);
    return AT_OK;
  }
  return AT_ERROR;
}

ATEerror_t at_SendBinary(const char *param)
{
  LoraErrorStatus status;
  const char *buf= param;
  unsigned char bufSize= strlen(param);
  uint32_t appPort;
  unsigned size=0;
  char hex[3];
  
    /* read and set the application port */
  if (1 != tiny_sscanf(buf, "%u:", &appPort))
  {
    PPRINTF("AT+SEND without the application port");
    return AT_PARAM_ERROR;
  }
  
  /* skip the application port */
  while (('0' <= buf[0]) && (buf[0] <= '9'))
  {
    buf ++;
    bufSize --;
  };
  
  if (buf[0] != ':')
  {
    PPRINTF("AT+SEND without the application port");
    return AT_PARAM_ERROR;
  }
  else
  {
    /*ok skip the char ':' */
    buf ++;
    bufSize --;
  }

  hex[2] = 0;
  while ((size < LORAWAN_APP_DATA_BUFF_SIZE) && (bufSize > 1))
  {
    hex[0] = buf[size*2];
    hex[1] = buf[size*2+1];
    if (tiny_sscanf(hex, "%hhx", &AppData.Buff[size]) != 1)
    {
      return AT_PARAM_ERROR;
    }
    size++;
    bufSize -= 2;
  }
  if (bufSize != 0)
  {
    return AT_PARAM_ERROR;
  }
  
  AppData.BuffSize = size;
  AppData.Port= appPort;

  status = LORA_send( &AppData, lora_config_reqack_get() );
  
  if (status == LORA_SUCCESS)
  {
    return AT_OK;
  }
  else
  {
    return AT_ERROR;
  }
}

ATEerror_t at_Send(const char *param)
{
  LoraErrorStatus status;
  const char *buf= param;
  unsigned char bufSize= strlen(param);
  uint32_t appPort;
  
    /* read and set the application port */
  if (1 != tiny_sscanf(buf, "%u:", &appPort))
  {
    PPRINTF("AT+SEND without the application port");
    return AT_PARAM_ERROR;
  }
  
  /* skip the application port */
  while (('0' <= buf[0]) && (buf[0] <= '9'))
  {
    buf ++;
    bufSize --;
  };
  
  if (buf[0] != ':')
  {
    PPRINTF("AT+SEND without the application port");
    return AT_PARAM_ERROR;
  }
  else
  {
    /*ok skip the char ':' */
    buf ++;
    bufSize --;
  }
  
  if (bufSize > LORAWAN_APP_DATA_BUFF_SIZE)
  {
    bufSize = LORAWAN_APP_DATA_BUFF_SIZE;
  }
  memcpy1(AppData.Buff, (uint8_t *)buf, bufSize);
  AppData.BuffSize = bufSize;
  AppData.Port= appPort;
  
  status = LORA_send( &AppData, lora_config_reqack_get() );
  
  if (status == LORA_SUCCESS)
  {
    return AT_OK;
  }
  else
  {
    return AT_ERROR;
  }
}

ATEerror_t at_ReceiveBinary(const char *param)
{
  unsigned i;
  
  AT_PRINTF("%d:", ReceivedDataPort);
  for (i = 0; i < ReceivedDataSize; i++)
  {
    AT_PRINTF("%02x", ReceivedData[i]);
  }
  AT_PRINTF("\r\n");
  ReceivedDataSize = 0;

  return AT_OK;
}

ATEerror_t at_Receive(const char *param)
{
  AT_PRINTF("%d:", ReceivedDataPort);
  if (ReceivedDataSize)
  {
    AT_PRINTF("%s", ReceivedData);
    ReceivedDataSize = 0;
  }
  AT_PRINTF("\r\n");

  return AT_OK;
}

ATEerror_t at_version_get(const char *param)
{
  AT_PRINTF(AT_VERSION_STRING" ");
	region_printf();
	
  return AT_OK;
}

ATEerror_t at_ack_set(const char *param)
{
	if( ( LoRaMacState & 0x00000001 ) == 0x00000001 )
	{
		return AT_BUSY_ERROR;
	}
	
  switch (param[0])
  {
    case '0':
      lora_config_reqack_set(LORAWAN_UNCONFIRMED_MSG);
      break;
    case '1':
      lora_config_reqack_set(LORAWAN_CONFIRMED_MSG);
      break;
    default:
      return AT_PARAM_ERROR;
  }

  return AT_OK;
}

ATEerror_t at_ack_get(const char *param)
{
  print_d (((lora_config_reqack_get() == LORAWAN_CONFIRMED_MSG) ? 1 : 0));
  return AT_OK;
}

ATEerror_t at_isack_get(const char *param)
{
  print_d(((lora_config_isack_get() == LORA_ENABLE) ? 1 : 0));
  return AT_OK;
}

ATEerror_t at_snr_get(const char *param)
{
  print_u(lora_config_snr_get());
  return AT_OK;
}

ATEerror_t at_rssi_get(const char *param)
{
  print_d(lora_config_rssi_get());
  return AT_OK;
}

ATEerror_t at_TDC_set(const char *param)
{ 
	uint32_t time=0;
	if (tiny_sscanf(param, "%lu", &time) != 1)
  {
    return AT_PARAM_ERROR;
  }
	
	if(time<5000)
	{
		PRINTF("TDC setting needs to be high than 4s\n\r");
		return AT_PARAM_ERROR;
	}
	else
		APP_TX_DUTYCYCLE=time;
	
	return AT_OK;
}
ATEerror_t at_TDC_get(const char *param)
{ 
	print_d(APP_TX_DUTYCYCLE);
	return AT_OK;
}

ATEerror_t at_BAT_get(const char *param)
{ 
	HW_GetBatteryLevel( );
	print_d(batteryLevel_mV);
	return AT_OK;
}

ATEerror_t at_ATDISABLE(const char *param)
{ 
	parse_flag=0;
	
	return AT_OK;
}

ATEerror_t at_PASSWORD_set(const char *param)
{
	if (tiny_sscanf(param, "%lu", &password_get) != 1)
  {
		return AT_PARAM_ERROR;
	}
	
	return AT_OK;
}

ATEerror_t at_PASSWORD_get(const char *param)
{
	print_d(password_get);
	
	return AT_OK;
}

void at_PASSWORD_comp(const char *param)
{
	uint32_t password;
	
	if (tiny_sscanf(param, "%lu", &password) != 1)
  {
	}
	
	if(password==password_get)
	{
		parse_flag=1;
		
	  TimerInit( &ATcommandsTimer, OnTimerATcommandsEvent );
    TimerSetValue(  &ATcommandsTimer, 300000);//timeout=5 min
    TimerStart( &ATcommandsTimer );
		
		PRINTF("Correct Password\n\r");
		
//		key_printf();
	}
	else
		PRINTF("Incorrect Password\n\r");
}

void OnTimerATcommandsEvent(void)
{
	TimerStop( &ATcommandsTimer );
	parse_flag=0;
}

ATEerror_t at_application_port_set(const char *param)
{
	 uint8_t application_port;

  if (tiny_sscanf(param, "%hhu", &application_port) != 1)
  {
    return AT_PARAM_ERROR;
  }
  
  lora_config_application_port_set(application_port) ;

  return AT_OK;
}

ATEerror_t at_application_port_get(const char *param)
{
	print_d(lora_config_application_port_get());
	return AT_OK;
}

ATEerror_t at_CHE_set(const char *param)
{
	uint8_t fre;
	if (tiny_sscanf(param, "%d", &fre) != 1)
  {
    return AT_PARAM_ERROR;
  }
	
	#if defined ( REGION_CN470 )
	if((fre>12)||((fre>=1)&&(fre<=10)))
	{
		fre=11;
	  PPRINTF("Error Subband, must be 0 or 11,12\r\n");	
	}
	else
	{
  	PPRINTF("Attention:Take effect after ATZ\r\n");	
	}
	#elif defined ( REGION_US915 )
	if(fre>8)
	{
		fre=1;
	  PPRINTF("Error Subband, must be 0 ~ 8\r\n");
	}
	else
	{
  	PPRINTF("Attention:Take effect after ATZ\r\n");	
	}	
	#elif defined ( REGION_AU915 )
	if(fre>8)
	{
		fre=1;
	  PPRINTF("Error Subband, must be 0 ~ 8\r\n");		
	}
	else
	{
  	PPRINTF("Attention:Take effect after ATZ\r\n");	
	}
	#else
	fre=0;
	#endif
	
	customize_set8channel_set(fre);
	
	return AT_OK;
}

ATEerror_t at_CHE_get(const char *param)
{ 
  print_d(customize_set8channel_get());
	uint8_t i;
	double fre1;
	double j,k,l;
	
	i=customize_set8channel_get();
	
	#if defined ( REGION_CN470 )
	  j=470.3;k=1.6;l=0.2;
	#elif defined ( REGION_US915 )
	if(i==9)
	{
    j=902.2;k=0.1;l=1.6;
	}
	else
	{j=902.3;k=1.6;l=0.2;}
	#else
	if(i==10)
	{
    j=915.0;k=0.1;l=1.6;
	}
	else
	{j=915.2;k=1.6;l=0.2;}
	#endif
	
	if(i)
	{
	  fre1=j+(i-1)*k;
	  for(int i=0;i<8;i++)
	  {		
		  PPRINTF("%.1f ",fre1);
		  fre1=fre1+l;
	  }
	  PPRINTF("\n\r");
  }
	else PPRINTF("Use default channel\r\n");
	
	return AT_OK;
}

ATEerror_t at_CHS_set(const char *param)
{
	uint32_t fre;
	if (tiny_sscanf(param, "%lu", &fre) != 1)
  {
    return AT_PARAM_ERROR;
  }
	
	if((100000000<fre&&fre<999999999)||fre==0)
	{
	  customize_freq1_set(fre);
	}
	else 
	{
		return AT_PARAM_ERROR;
	}
	
	PPRINTF("Attention:Take effect after ATZ\r\n");
	
	return AT_OK;
}

ATEerror_t at_CHS_get(const char *param)
{ 
	print_d(customize_freq1_get());
	return AT_OK;
}

ATEerror_t at_PDTA_set(const char *param)
{
	MibRequestConfirm_t mibReq;
  LoRaMacStatus_t mac_status;
	
	uint16_t start_page=0,end_page=0;
	
	uint8_t status=0;
	
	is_PDTA_command=1;
	
  mibReq.Type = MIB_NETWORK_JOINED;
  mac_status = LoRaMacMibGetRequestConfirm(&mibReq);
		
	if (tiny_sscanf(param, "%d,%d",
                  &start_page, &end_page) != 2)
  {
    return AT_PARAM_ERROR;
  }
		
	if(start_page==0||end_page==0||start_page>400||end_page>400)
	{
		return AT_PARAM_ERROR;
	}
	
	if(start_page<=end_page)
	{
		if(sleep_status==1)
		{
			status=read_data_on_flash_buff(start_page,end_page);
		}
		else
		{
			if(mibReq.Param.IsNetworkJoined == 1)
			{
				if( ( LoRaMacState & 0x00000001 ) == 0x00000001 )
				{
					return AT_BUSY_ERROR;
				}
				else
				{
					if(work_mode==0)
					  PPRINTF("Stop Tx events when read sensor data\r");
					else
						PPRINTF("Stop Tx and CITEMP events when read sensor data\r");
				}
		  }
			else
			{
				PPRINTF("Stop Tx events when read sensor data\r");
			}
			
			TimerStop(&MacStateCheckTimer);
	    TimerStop(&TxDelayedTimer);
	    TimerStop(&AckTimeoutTimer);

      TimerStop(&RxWindowTimer1);
      TimerStop(&RxWindowTimer2);
	    TimerStop(&TxTimer);
			TimerStop(&ReJoinTimer);
			TimerStop( &CalibrationUTCTimer);
			TimerStop( &sht20tempcompTimer);
			status=read_data_on_flash_buff(start_page,end_page);
	  }
	}
	else
		return AT_PARAM_ERROR;
	
	if(status==1)
	{
		status=0;
		
		if(sleep_status==0)
		{
			if (mac_status == LORAMAC_STATUS_OK)
			{
				if(mibReq.Param.IsNetworkJoined == 1)
				{				
				  if(work_mode==0)
					{
						PPRINTF("Start Tx events\r");				    
					}
					else
					{
						PPRINTF("Start Tx and CITEMP events\r");
            TimerStart( &sht20tempcompTimer);
					}
					
				  TimerStart(&TxTimer);
				}
				else
				{
					if(FDR_status==0)
					{
						PPRINTF("Start Tx events\r");
					  TimerStart(&TxDelayedTimer);
					}
				}
			}
	  }
	}
	
	is_PDTA_command=0;
	return AT_OK;
}

ATEerror_t at_PLDTA_set(const char *param)
{
	MibRequestConfirm_t mibReq;
  LoRaMacStatus_t mac_status;
	
	uint32_t address=0;
	uint16_t num=0;
	
	uint8_t status=0;
	
	is_PLDTA_command=1;
	
  mibReq.Type = MIB_NETWORK_JOINED;
  mac_status = LoRaMacMibGetRequestConfirm(&mibReq);
		
	if (tiny_sscanf(param, "%lu",
                  &num) != 1)
  {
    return AT_PARAM_ERROR;
  }
		
	if(num==0||num>3200)
	{
		return AT_PARAM_ERROR;
	}
	
	address=write_address;
	
	for(uint16_t i=0;i<num;i++)
	{
		address-=16;
		if(address<=FLASH_SENSOR_DATA_START_ADDR)
		{
			address=FLASH_SENSOR_DATA_END_ADDR;
		}
	}
	
		if(sleep_status==1)
		{
			status=read_data_on_flash_buff_last_sets_data(address,num);
		}
		else
		{
			if(mibReq.Param.IsNetworkJoined == 1)
			{
				if( ( LoRaMacState & 0x00000001 ) == 0x00000001 )
				{
					return AT_BUSY_ERROR;
				}
				else
				{
					if(work_mode==0)
					  PPRINTF("Stop Tx events when read sensor data\r");
					else
						PPRINTF("Stop Tx and CITEMP events when read sensor data\r");
				}
		  }
			else
			{
				PPRINTF("Stop Tx events when read sensor data\r");
			}
			
			TimerStop(&MacStateCheckTimer);
	    TimerStop(&TxDelayedTimer);
	    TimerStop(&AckTimeoutTimer);

      TimerStop(&RxWindowTimer1);
      TimerStop(&RxWindowTimer2);
	    TimerStop(&TxTimer);
			TimerStop(&ReJoinTimer);
			TimerStop( &CalibrationUTCTimer);
			TimerStop( &sht20tempcompTimer);
			status=read_data_on_flash_buff_last_sets_data(address,num);;
	  }
		
	DelayMs(1000);
	if(status==1)
	{
		status=0;
		
		if(sleep_status==0)
		{
			if (mac_status == LORAMAC_STATUS_OK)
			{
				if(mibReq.Param.IsNetworkJoined == 1)
				{
				  if(work_mode==0)
					{
						PPRINTF("Start Tx events\r");				    
					}
					else
					{
						PPRINTF("Start Tx and CITEMP events\r");
            TimerStart( &sht20tempcompTimer);
					}
					
				  TimerStart(&TxTimer);
				}
				else
				{
					if(FDR_status==0)
					{
						PPRINTF("Start Tx events\r");
					  TimerStart(&TxDelayedTimer);
					}
				}
			}
	  }
	}
	
	is_PLDTA_command=0;
	return AT_OK;
}

ATEerror_t at_CLRDTA_run(const char *param)
{
	MibRequestConfirm_t mibReq;
  LoRaMacStatus_t mac_status;
	
	mibReq.Type = MIB_NETWORK_JOINED;
  mac_status = LoRaMacMibGetRequestConfirm(&mibReq);
	
	if(sleep_status==0)
	{
		if(mibReq.Param.IsNetworkJoined == 1)
		{
			if( ( LoRaMacState & 0x00000001 ) == 0x00000001 )
			{
				return AT_BUSY_ERROR;
			}
			else
			{
				if(work_mode==0)
				  PPRINTF("Stop Tx events,Please wait for the erase to complete\r");
				else
					PPRINTF("Stop Tx and CITEMP events,Please wait for the erase to complete\r");
			}
		}
		else
		{
			PPRINTF("Stop Tx events,Please wait for the erase to complete\r");
		}
  }
	
	PRINTF("Clear all stored sensor data...\r");
	
	TimerStop(&MacStateCheckTimer);
	TimerStop(&TxDelayedTimer);
	TimerStop(&AckTimeoutTimer);

  TimerStop(&RxWindowTimer1);
  TimerStop(&RxWindowTimer2);
	TimerStop(&TxTimer);
	TimerStop(&ReJoinTimer);
	TimerStop( &CalibrationUTCTimer);
	TimerStop( &sht20tempcompTimer);
	
	FLASH_erase_all_sensor_data_storage(FLASH_SENSOR_DATA_START_ADDR);
	write_address=FLASH_SENSOR_DATA_START_ADDR;
	DelayMs(1000);
	
		if(sleep_status==0)
		{
			if (mac_status == LORAMAC_STATUS_OK)
			{
				if(mibReq.Param.IsNetworkJoined == 1)
				{
				  if(work_mode==0)
					{
						PPRINTF("Start Tx events\r");
					}
					else
					{
						PPRINTF("Start Tx and CITEMP events\r");
            TimerStart( &sht20tempcompTimer);
					}
					
				  TimerStart(&TxTimer);
				}
				else
				{
					if(FDR_status==0)
					{
						PPRINTF("Start Tx events\r");
					  TimerStart(&TxDelayedTimer);
					}
				}
			}
	  }
	
	return AT_OK;
}

ATEerror_t at_CFG_run(const char *param)
{
	MibRequestConfirm_t mibReq;
  LoRaMacStatus_t mac_status;
	
	mibReq.Type = MIB_NETWORK_JOINED;
  mac_status = LoRaMacMibGetRequestConfirm(&mibReq);
	
	if(sleep_status==0)
	{
		if(mibReq.Param.IsNetworkJoined == 1)
		{
			if( ( LoRaMacState & 0x00000001 ) == 0x00000001 )
			{
				return AT_BUSY_ERROR;
			}
			else
			{
				if(work_mode==0)
				  PPRINTF("Stop Tx events,Please wait for all configurations to print\r");
				else
					PPRINTF("Stop Tx and CITEMP events,Please wait for all configurations to print\r");
			}
		}
		else
		{
			PPRINTF("Stop Tx events,Please wait for all configurations to print\r");
		}
  }
	
	PRINTF("Printf all config...\r");
	
	TimerStop(&MacStateCheckTimer);
	TimerStop(&TxDelayedTimer);
	TimerStop(&AckTimeoutTimer);

  TimerStop(&RxWindowTimer1);
  TimerStop(&RxWindowTimer2);
	TimerStop(&TxTimer);
	TimerStop( &CalibrationUTCTimer);
  TimerStop( &sht20tempcompTimer);
	
	printf_all_config();
	DelayMs(1000);
	
		if(sleep_status==0)
		{
			if (mac_status == LORAMAC_STATUS_OK)
			{
				if(mibReq.Param.IsNetworkJoined == 1)
				{
				  if(work_mode==0)
					{
						PPRINTF("Start Tx events\r");
					}
					else
					{
						PPRINTF("Start Tx and CITEMP events\r");
            TimerStart( &sht20tempcompTimer);
					}
					
				  TimerStart(&TxTimer);
				}
				else
				{
					if(FDR_status==0)
					{
						PPRINTF("Start Tx events\r");
					  TimerStart(&TxDelayedTimer);
					}
				}
			}
	  }
	
	return AT_OK;
}

ATEerror_t at_EXT_set(const char *param)
{
	uint16_t mode,param_ext;
	if(tiny_sscanf(param, "%d,%d", &mode,&param_ext) != 2)
	{
		if(tiny_sscanf(param, "%d", &mode) != 1)
		{
			return AT_PARAM_ERROR;
		}
	}
	
	if((mode==4)&&(param_ext!=0))
	{
		exitintmode=param_ext;
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
	else if(mode==6)
	{
		power_time=param_ext;
	}
	else if(mode==7||mode==8)
	{
		exitmode=param_ext;
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
	else if (mode>15)
  {
    return AT_PARAM_ERROR;
  }
	
	Ext=mode;
	if((Ext!=4)&&(Ext!=7)&&(Ext!=8))
	{
		GPIO_EXTI4_IoDeInit();
	}
	
	return AT_OK;
}

ATEerror_t at_EXT_get(const char *param)
{
	if(Ext==4)
	{
		PPRINTF("%d,%d\r\n",Ext,exitintmode);
	}
	else if(Ext==6)
	{
		PPRINTF("%d,%d\r\n",Ext,power_time);		
	}
	else if(Ext==7||Ext==8)
	{
		PPRINTF("%d,%d\r\n",Ext,exitmode);		
	}
	else
	{
		print_d(Ext);		
	}
	return AT_OK;
}

ATEerror_t at_WMOD_set(const char *param)
{
	uint8_t mode;
	if (tiny_sscanf(param, "%d", &mode) != 1)
  {
    return AT_PARAM_ERROR;
  }
	
	if(mode>1)
	{
		return AT_PARAM_ERROR;
	}
	else
	  work_mode=mode;
	
	PPRINTF("Attention:Take effect after ATZ\r\n");
	
	return AT_OK;
}

ATEerror_t at_WMOD_get(const char *param)
{
	print_d(work_mode);
	return AT_OK;
}

ATEerror_t at_ARTEMP_set(const char *param)
{
	short temp1=0,temp2=0;
	if (tiny_sscanf(param, "%d,%d",
                  &temp1, &temp2) != 2)
  {
    return AT_PARAM_ERROR;
  }
	
	if(temp1>temp2||temp1<-40||temp2>125)
		return AT_PARAM_ERROR;
	else
	{
		comp_temp1_sht20=temp1;
		comp_temp2_sht20=temp2;
	}
	
	PPRINTF("%d,%d\r\n",temp1,temp2);
	return AT_OK;
}

ATEerror_t at_ARTEMP_get(const char *param)
{
	PPRINTF("%d,%d\r\n",comp_temp1_sht20,comp_temp2_sht20);
	return AT_OK;
}

ATEerror_t at_CITEMP_set(const char *param)
{ 
	uint32_t time=0;
	if (tiny_sscanf(param, "%lu", &time) != 1)
  {
    return AT_PARAM_ERROR;
  }
	
	if(work_mode==1)
	{
		if(time>65535)
		{
			return AT_PARAM_ERROR;
		}
		else
			collection_interval=time;
		
		if(collection_interval!=0)
		{
			if(sleep_status==0 && FDR_status==0)
			{
//				RX8010_timer_interrupt(collection_interval);
			}
		}
//		else
//			RX8010_timer_interrupt_disable();
  }
	else 
		AT_PRINTF("Work Mode error\r\n");
	
	return AT_OK;
}

ATEerror_t at_CITEMP_get(const char *param)
{ 
	print_d(collection_interval);
	return AT_OK;
}

ATEerror_t at_SETCNT_set(const char *param)
{
	if(Ext!=7&&Ext!=8)
	{
  return AT_PARAM_ERROR;		
	}
	
	uint32_t count_temp;
	if (tiny_sscanf(param,"%d",&count_temp) != 1)
	{
		return AT_PARAM_ERROR;
	}
	
  if(Ext==0x07&&count_temp>65535)
	{
		return AT_PARAM_ERROR;
	}		
	count=count_temp;
	PRINTF("Set count:%d\r\n",count);
		
	return AT_OK;
	
}

ATEerror_t at_DwellTime_set(const char *param)
{
#if defined( REGION_AS923 )	|| defined( REGION_AU915 )
	uint8_t dwelltime_temp;
	if (tiny_sscanf(param, "%d", &dwelltime_temp) != 1)	
	{
    return AT_PARAM_ERROR;		
	}
	
	if((dwelltime_temp==1)||(dwelltime_temp==0))
	{
		dwelltime=dwelltime_temp;
	}
	else
	{
    return AT_PARAM_ERROR;			
	}

	PPRINTF("Attention:Take effect after ATZ\r\n");
	
	return AT_OK;
#else
    PPRINTF("This Channel Plan is not support\r\n");
		return AT_OK;	
#endif	
}

ATEerror_t at_DwellTime_get(const char *param)
{
  print_d(dwelltime);	
  return AT_OK;	
}

ATEerror_t at_RJTDC_set(const char *param)
{ 
	uint16_t time=0;
	if (tiny_sscanf(param, "%lu", &time) != 1)
  {
    return AT_PARAM_ERROR;
  }
	
	if(time>0 && time<=65535)//min
	{
		REJOIN_TX_DUTYCYCLE=time;
	}
	else
	{
		PRINTF("The RJTDC range is 1 to 65535\n\r");
		return AT_PARAM_ERROR;	
	}
	
	return AT_OK;
}
ATEerror_t at_RJTDC_get(const char *param)
{ 
	print_d(REJOIN_TX_DUTYCYCLE);
	return AT_OK;
}

ATEerror_t at_RPL_set(const char *param)
{ 
	uint8_t time=0;
	if (tiny_sscanf(param, "%d", &time) != 1)
  {
    return AT_PARAM_ERROR;
  }
	
	/*
	 0 : null
	 1 : unconfirm downlink data
	 2 : confirm downlink data
	 3 : MAC command
	 4 : MAC command or confirm downlink data
	 */
	if(time<=4)
	{
		response_level=time;
	}
	else
	{
		PRINTF("The response level range is 0 to 4\n\r");
		return AT_PARAM_ERROR;	
	}
	
	return AT_OK;
}
ATEerror_t at_RPL_get(const char *param)
{ 
	print_d(response_level);
	return AT_OK;
}

ATEerror_t at_timestamp_set(const char *param)
{ 
	uint32_t timestamp=0;
	SysTime_t sysTime = { 0 };
	if (tiny_sscanf(param, "%lu", &timestamp) != 1)
  {
    return AT_PARAM_ERROR;
  }
	
	if(timestamp>1611878400)//20210129 00:00:00
	{
		sysTime.Seconds=timestamp;
		SysTimeSet( sysTime );	
	}
	else
	{
		PRINTF("value needs to be greater than 1611878400\n\r");
		return AT_PARAM_ERROR;
	}
	return AT_OK;
}

ATEerror_t at_timestamp_get(const char *param)
{ 
	struct tm localtime;
	SysTime_t sysTimeCurrent = { 0 };
	
	sysTimeCurrent=SysTimeGet();	
	SysTimeLocalTime( sysTimeCurrent.Seconds, &localtime );

	PPRINTF("systime= %u %d %d %d %d %d %d\n\r",sysTimeCurrent.Seconds,localtime.tm_year+1900,localtime.tm_mon,localtime.tm_mday,localtime.tm_hour,localtime.tm_min,localtime.tm_sec);
	return AT_OK;
}

ATEerror_t at_LeapSecond_set(const char *param)
{ 
	uint8_t second=0;
	if (tiny_sscanf(param, "%d", &second) != 1)
  {
    return AT_PARAM_ERROR;
  }
	
	if(second<=255)
	{
		currentLeapSecond=second;
	}
	else
	{
		PRINTF("The LeapSecond range is 0 to 255\n\r");
		return AT_PARAM_ERROR;	
	}
	
	return AT_OK;
}
ATEerror_t at_LeapSecond_get(const char *param)
{ 
	print_d(currentLeapSecond);
	return AT_OK;
}

ATEerror_t at_time_synchronization_method_set(const char *param)
{ 
	uint8_t mode=0;
	if (tiny_sscanf(param, "%d", &mode) != 1)
  {
    return AT_PARAM_ERROR;
  }
	
	if(mode<2)
	{
		time_synchronization_method=mode;
		if(time_synchronization_method)
		{
			PRINTF("The network server must support LoRaWAN v1.0.3\r");
		}
	}
	else
	{
		PRINTF("The time synchronization method range is 0 or 1\n\r");
		return AT_PARAM_ERROR;	
	}
	
	return AT_OK;
}
ATEerror_t at_time_synchronization_method_get(const char *param)
{ 
	print_d(time_synchronization_method);
	return AT_OK;
}

ATEerror_t at_time_synchronization_interval_set(const char *param)
{ 
	uint8_t interval=0;
	if (tiny_sscanf(param, "%d", &interval) != 1)
  {
    return AT_PARAM_ERROR;
  }
	
	if(interval>0 && interval<=255)
	{
		time_synchronization_interval=interval;
	}
	else
	{
		PRINTF("The time synchronization interval range is 1 to 255\n\r");
		return AT_PARAM_ERROR;	
	}
	
	return AT_OK;
}
ATEerror_t at_time_synchronization_interval_get(const char *param)
{ 
	print_d(time_synchronization_interval);
	return AT_OK;
}

ATEerror_t at_pid_set(const char *param)
{
	uint8_t pid_temp;
	if (tiny_sscanf(param, "%d", &pid_temp) != 1)
  {
    return AT_PARAM_ERROR;
  }
	
	if(pid_temp>1)
	{
		PPRINTF("PID must be 0 or 1\r\n");
		return AT_PARAM_ERROR;
	}
	else
	  pid_flag=pid_temp;
	
	if(Ext==0x01||Ext==0x09)
	{
		if(pid_flag==0)
		{
			TimerStop( &DS18B20IDTimer );
		}
		else
			PPRINTF("Attention:Take effect after ATZ\r\n");
	}
						
	return AT_OK;
}

ATEerror_t at_pid_get(const char *param)
{
	print_d(pid_flag);
	return AT_OK;
}

/* Private functions ---------------------------------------------------------*/

static ATEerror_t translate_status(LoRaMacStatus_t status)
{
  if (status == LORAMAC_STATUS_BUSY)
  {
    return AT_BUSY_ERROR;
  }
  if (status == LORAMAC_STATUS_PARAMETER_INVALID)
  {
    return AT_PARAM_ERROR;
  }
  if (status == LORAMAC_STATUS_NO_NETWORK_JOINED)
  {
    return AT_NO_NET_JOINED;
  }
  if (status != LORAMAC_STATUS_OK)
  {
    return AT_ERROR;
  }
  return AT_OK;
}

static int sscanf_16_hhx(const char *from, uint8_t *pt)
{
  return tiny_sscanf(from, "%hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx",
                     &pt[0], &pt[1], &pt[2], &pt[3], &pt[4], &pt[5], &pt[6],
                     &pt[7], &pt[8], &pt[9], &pt[10], &pt[11], &pt[12], &pt[13],
                     &pt[14], &pt[15]);
}

static void print_16_02x(uint8_t *pt)
{
  AT_PRINTF("%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\r\n",
            pt[0], pt[1], pt[2], pt[3],
            pt[4], pt[5], pt[6], pt[7],
            pt[8], pt[9], pt[10], pt[11],
            pt[12], pt[13], pt[14], pt[15]);
}

static int sscanf_uint32_as_hhx(const char *from, uint32_t *value)
{
  return tiny_sscanf(from, "%hhx %hhx %hhx %hhx",
                     &((unsigned char *)(value))[3],
                     &((unsigned char *)(value))[2],
                     &((unsigned char *)(value))[1],
                     &((unsigned char *)(value))[0]);
}

static void print_uint32_as_02x(uint32_t value)
{
  AT_PRINTF("%02x %02x %02x %02x\r\n",
            (unsigned)((unsigned char *)(&value))[3],
            (unsigned)((unsigned char *)(&value))[2],
            (unsigned)((unsigned char *)(&value))[1],
            (unsigned)((unsigned char *)(&value))[0]);
}

static void print_8_02x(uint8_t *pt)
{
  AT_PRINTF("%02x %02x %02x %02x %02x %02x %02x %02x\r\n",
            pt[0], pt[1], pt[2], pt[3], pt[4], pt[5], pt[6], pt[7]);
}

static void print_d(int value)
{
  AT_PRINTF("%d\r\n", value);
}

static void print_u(unsigned int value)
{
  AT_PRINTF("%u\r\n", value);
}
