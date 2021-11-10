/******************************************************************************
  * @file    main.c
  * @author  MCD Application Team
  * @version V1.1.4
  * @date    08-January-2018
  * @brief   this is the main!
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
#include "low_power_manager.h"
#include "lora.h"
#include "bsp.h"
#include "timeServer.h"
#include "vcom.h"
#include "version.h"
#include "command.h"
#include "at.h"
#include "flash_eraseprogram.h"
#include "RX8010.h"
#include "ds18b20.h"
#include "systime.h"
#include "delay.h"
#include "iwdg.h"
#include "gpio_exti.h"

extern SysTime_t LastTxdoneTime;
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/*!
 * Defines the application data transmission duty cycle. 60s, value in [ms].
 */
uint32_t APP_TX_DUTYCYCLE=300000;//ms
static uint8_t DS18B20_ID_temp[1][8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
extern uint8_t DS18B20_ID[1][8];

/*
 *Poll Message Flag: 1: This message is a poll message from server, 0: means this is a normal uplink.
 *Data Record Flag: 1: this message is stored in Flash,0: This message is not stored in flash
 *UTC Set time OK: 1: Set time ok,0: N/A
 *UTC Set Time Request:1: Request server downlink UTC time, 0 : N/A
 */
bool Poll_Message_Flag=0,Data_Record_Flag=0,UTC_Accept=0,UTC_Request=1;
static uint32_t ServerSetTDC;
static uint32_t timestamp1,timestamp2;
uint8_t TDC_flag=0;
void read_sensor_data(uint32_t timestamp,uint8_t dev,uint16_t bat,float temp20,float hum20,float extsen,uint16_t rev);
static uint32_t bsp_sensor_data[4]={0x00000000,0x00000000,0x00000000,0x00000000};
uint8_t Ext=0x01;//sensor type,0x01=ds18b20
uint8_t work_mode=0;
uint16_t collection_interval=1;//1min

__IO uint32_t write_address;
bool sleep_status=0;//AT+SLEEP
bool is_lora_joined=0;
bool is_time_to_send=0;
bool is_time_to_IWDG_Refresh=0;
bool exti_flag=0;
bool exti_flag2;
bool level_status=0;
bool exit_fr=0;
extern bool debug_flags;

uint8_t is_PDTA_command=0;
uint8_t is_PLDTA_command=0;

extern uint8_t dwelltime;
uint8_t atz_flags=0;
uint16_t REJOIN_TX_DUTYCYCLE=20;//min
bool MAC_COMMAND_ANS_status=0;
uint8_t response_level=0;
bool is_there_data=0;
bool rejoin_status=0;
bool rejoin_keep_status=0;
bool is_time_to_linkcheck=0;
bool is_time_to_rejoin=0;
bool JoinReq_NbTrails_over=0;
bool unconfirmed_downlink_data_ans_status=0,confirmed_downlink_data_ans_status=0;
float collection_temp_sht20=0;
bool is_time_to_temp_comp=0;
bool beyond_status=0;
bool is_time_to_send_beyond_data;
static void sht20_temp_comp(void);
short comp_temp1_sht20=-40,comp_temp2_sht20=125;
bool is_time_to_send_ds18b20_id=0;//pid=1

uint8_t currentLeapSecond;
uint8_t time_synchronization_method;
uint8_t time_synchronization_interval;

bool payload_oversize;

//uint8_t Ext4_record_status;
//bool Ext4_is_time_to_comp_record_status;	
uint8_t pid_flag;

#define BCD_TO_HEX2(bcd) ((((bcd)>>4)*10)+((bcd)&0x0F))

/*!
 * LoRaWAN Adaptive Data Rate
 * @note Please note that when ADR is enabled the end-device should be static
 */
#define LORAWAN_ADR_STATE LORAWAN_ADR_ON
/*!
 * LoRaWAN Default data Rate Data Rate
 * @note Please note that LORAWAN_DEFAULT_DATA_RATE is used only when ADR is disabled 
 */
#define LORAWAN_DEFAULT_DATA_RATE DR_0
/*!
 * LoRaWAN application port
 * @note do not use 224. It is reserved for certification
 */
#define LORAWAN_APP_PORT                            2
/*!
 * Number of trials for the join request.
 */
#define JOINREQ_NBTRIALS                            200
/*!
 * LoRaWAN default endNode class port
 */
#define LORAWAN_DEFAULT_CLASS                       CLASS_A
/*!
 * LoRaWAN default confirm state
 */
#define LORAWAN_DEFAULT_CONFIRM_MSG_STATE           LORAWAN_UNCONFIRMED_MSG
/*!
 * User application data buffer size
 */
#define LORAWAN_APP_DATA_BUFF_SIZE                           256
/*!
 * User application data
 */
static uint8_t AppDataBuff[LORAWAN_APP_DATA_BUFF_SIZE];

int user_key_exti_flag=0;

uint16_t batteryLevel_mV;
uint8_t user_key_duration=0;
uint8_t payloadlens=0;
void user_key_event(void);

extern TimerEvent_t MacStateCheckTimer;
extern TimerEvent_t TxDelayedTimer;
extern TimerEvent_t AckTimeoutTimer;
extern TimerEvent_t RxWindowTimer1;
extern TimerEvent_t RxWindowTimer2;
extern bool rx2_flags;
extern uint32_t LoRaMacState;
extern uint8_t exitintmode;
extern uint8_t exitmode;
extern uint16_t power_time;
extern uint32_t count;
extern bool connect_flags_temp;
/*!
 * User application data structure
 */
static lora_AppData_t AppData={ AppDataBuff,  0 ,0 };
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* call back when LoRa endNode has received a frame*/
static void LORA_RxData( lora_AppData_t *AppData);

/* call back when LoRa endNode has just joined*/
static void LORA_HasJoined( void );

/* call back when LoRa endNode has just switch the class*/
static void LORA_ConfirmClass ( DeviceClass_t Class );

/* LoRa endNode send request*/
static void Send( void );

#if defined(LoRa_Sensor_Node)
/* start the tx process*/
static void LoraStartTx(TxEventType_t EventType);

static void StartIWDGRefresh(TxEventType_t EventType);
static void LoraStartRejoin(TxEventType_t EventType);
static void StartRetrieval(TxEventType_t EventType);
static void StartCalibrationUTCtime(TxEventType_t EventType);
static void Startsht20tempcomptime(TxEventType_t EventType);
static void StartDS18B20ID(TxEventType_t EventType);

TimerEvent_t TxTimer;
TimerEvent_t downlinkLedTimer;
TimerEvent_t downlinkLedTimer;
TimerEvent_t NetworkJoinedLedTimer;
TimerEvent_t PressButtonTimesLedTimer;
TimerEvent_t PressButtonTimeoutTimer;
TimerEvent_t IWDGRefreshTimer;
TimerEvent_t ReJoinTimer;
TimerEvent_t RetrievalTimer;
TimerEvent_t CalibrationUTCTimer;
TimerEvent_t sht20tempcompTimer;
TimerEvent_t DS18B20IDTimer;

bool is_time_to_find_timestamp1_on_flash=0;
bool is_time_to_reply_retrieval_request=0;
bool is_time_to_send_retrieval_data=0;
__IO uint32_t find_timestamp_addr;
uint32_t find_timestamp_addr_record;

uint8_t retrieval_uplink_time=0;

bool joined_led=0;
uint8_t press_button_times=0;//Press the button times in a row fast
uint8_t OnPressButtonTimeout_status=0;
extern TimerEvent_t TxDelayedTimer;

extern void printf_joinmessage(void);

/* tx timer callback function*/
static void OnTxTimerEvent( void );
#endif

void OndownlinkLedEvent(void);
void OnNetworkJoinedLedEvent(void);
void OnPressButtonTimesLedEvent(void);
void OnPressButtonTimeoutEvent(void);
void OnIWDGRefreshTimeoutEvent(void);
static void OnReJoinTimerEvent( void );
static void OnRetrievalTimeoutEvent( void );
static void find_timestamp1_on_flash(void);//find the address where timestamp is stored
static void Reply_to_retrieval_request(void);
static void OnCalibrationUTCTimeoutEvent( void );
static void onsht20tempcompTimeoutEvent( void );
void OnDS18B20IDTimeoutEvent(void);
bool comp_ds18b20_id(void);

void send_exti(void);
/* Private variables ---------------------------------------------------------*/
/* load Main call backs structure*/
static LoRaMainCallback_t LoRaMainCallbacks ={ HW_GetBatteryLevel,
                                               HW_GetTemperatureLevel,
                                               HW_GetUniqueId,
                                               HW_GetRandomSeed,
                                               LORA_RxData,
                                               LORA_HasJoined,
                                               LORA_ConfirmClass};

/* !
 *Initialises the Lora Parameters
 */
static  LoRaParam_t LoRaParamInit= {LORAWAN_ADR_STATE,
                                    LORAWAN_DEFAULT_DATA_RATE,  
                                    LORAWAN_PUBLIC_NETWORK,
                                    JOINREQ_NBTRIALS};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main( void )
{
  /* STM32 HAL library initialization*/
  HAL_Init( );
  
  /* Configure the system clock*/
  SystemClock_Config( );
  
  /* Configure the debug mode*/
  DBG_Init( );
  
  /* Configure the hardware*/
  HW_Init( );
 
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */
   CMD_Init();
	
  /*Disbale Stand-by mode*/
  LPM_SetOffMode(LPM_APPLI_Id , LPM_Disable );
	
	clr_all_sensor_data();
	
	write_address=FLASH_SENSOR_DATA_START_ADDR;
//	write_address=find_addr(write_address);
	
	write_address=find_addr_first_after_systemreset(write_address);
	
	iwdg_init();
	
	StartIWDGRefresh(TX_ON_EVENT);
	
	if(UID_COMP()==0)
	{
    /* Configure the Lora Stack*/
    LORA_Init( &LoRaMainCallbacks, &LoRaParamInit);
	}
	else
		PPRINTF("Device error\n\r");
	
  while( 1 )
  {
		/* Handle UART commands */
    CMD_Process();

		send_exti();
		
		if(joined_led==1)
		{
			joined_led=0;
			DelayMs(2000);
			HAL_GPIO_WritePin(LED_RGB_PORT,LED_RED_PIN|LED_GREEN_PIN|LED_BLUE_PIN,GPIO_PIN_RESET);
			TimerInit( &NetworkJoinedLedTimer, OnNetworkJoinedLedEvent );
			TimerSetValue( &NetworkJoinedLedTimer, 5000);
			HAL_GPIO_WritePin(LED_RGB_PORT,LED_GREEN_PIN,GPIO_PIN_SET); 
			TimerStart( &NetworkJoinedLedTimer );
		}
		
		if(is_lora_joined==1)
		{
			AT_PRINTF("JOINED\n\r");
      rejoin_keep_status=0;
	
			if((lora_config_otaa_get() == LORA_ENABLE ? 1 : 0))
			{
				printf_joinmessage();
			}			
			
      TimerStop(&ReJoinTimer);
			is_lora_joined=0;
			joined_led=1;
			
			if(work_mode==1)
			{
				Startsht20tempcomptime(TX_ON_TIMER);
			}
			
			#if defined(LoRa_Sensor_Node) /*LSN50 Preprocessor compile swicth:hw_conf.h*/
			if(Ext==0x09||time_synchronization_method==1)
			{
			  StartCalibrationUTCtime(TX_ON_EVENT);
			}
			
			if(pid_flag==1)
			{
				StartDS18B20ID(TX_ON_EVENT);
			}
			
			LoraStartTx( TX_ON_TIMER);    
			#endif
		}
		
		if(is_time_to_temp_comp==1 && work_mode==1)
		{
			is_time_to_temp_comp=0;
			sht20_temp_comp();
		}
		
		if(is_time_to_send_beyond_data==1)
		{
			if((( LoRaMacState & 0x00000001 ) != 0x00000001)&&(( LoRaMacState & 0x00000010 ) != 0x00000010))
			{
				is_time_to_send_beyond_data=0;
				Send();
			}
		}
		
		if(time_synchronization_method==1)
		{
			if(payload_oversize==1)
			{
				if((( LoRaMacState & 0x00000001 ) != 0x00000001)&&(( LoRaMacState & 0x00000010 ) != 0x00000010))
				{
					MlmeReq_t mlmeReq;
					mlmeReq.Type = MLME_DEVICE_TIME;
					LoRaMacMlmeRequest( &mlmeReq );
					payload_oversize=0;
					UTC_Request=0;//only send once 
					AppData.Buff[0]=0x00;
					AppData.BuffSize=1;
					AppData.Port = 2;
					LORA_send( &AppData, LORAWAN_UNCONFIRMED_MSG);
				}
			}
	  }
		
		if(is_time_to_send==1 || is_time_to_send_ds18b20_id==1)
		{
			if((( LoRaMacState & 0x00000001 ) != 0x00000001)&&(( LoRaMacState & 0x00000010 ) != 0x00000010))
			{
				if(time_synchronization_method==1 && payload_oversize==0)//0 via downlink,1 via devicetimereq mac command
				{
					if(UTC_Request==1)
					{
						UTC_Request=0;
						MlmeReq_t mlmeReq;
						mlmeReq.Type = MLME_DEVICE_TIME;
						LoRaMacMlmeRequest( &mlmeReq );
				  }
				}
//				is_time_to_send=0;
				Send();
      }
		}

		if(atz_flags==1)
		{
			DelayMs(500);
			AppData.Buff[0]=0x11;
			AppData.BuffSize=1;
			AppData.Port = 2;
			LORA_send( &AppData, LORAWAN_UNCONFIRMED_MSG);
			atz_flags++;
		}
		else if((atz_flags==2)&&(( LoRaMacState & 0x00000001 ) != 0x00000001))
		{
			NVIC_SystemReset();
		}
		#ifdef REGION_US915
		if(MAC_COMMAND_ANS_status==1)
		{		
				if((( LoRaMacState & 0x00000001 ) != 0x00000001)&&(( LoRaMacState & 0x00000010 ) != 0x00000010))
				{          
					MibRequestConfirm_t mib;
			
			    mib.Type=MIB_CHANNELS_DATARATE;
			    LoRaMacMibGetRequestConfirm(&mib);										
					
				  if(mib.Param.ChannelsDatarate==0)
			    {
						MAC_COMMAND_ANS_status=0;
						AppData.Buff[0]=0x00;
						AppData.BuffSize=1;
	          AppData.Port = 2;
	          LORA_send( &AppData, LORAWAN_UNCONFIRMED_MSG);
				  }
				}		  
		}
		#elif defined( REGION_AS923 )	|| defined( REGION_AU915 )		
		if((MAC_COMMAND_ANS_status==1)&&(dwelltime==1))
		{		
			if((( LoRaMacState & 0x00000001 ) != 0x00000001)&&(( LoRaMacState & 0x00000010 ) != 0x00000010))
			{          
				MibRequestConfirm_t mib;
		
				mib.Type=MIB_CHANNELS_DATARATE;
				LoRaMacMibGetRequestConfirm(&mib);
				
				MAC_COMMAND_ANS_status=0;
				
				if(mib.Param.ChannelsDatarate==2)
				{
					AppData.Buff[0]=0x00;
					AppData.BuffSize=1;
					AppData.Port = 2;							
					LORA_send( &AppData, LORAWAN_UNCONFIRMED_MSG);					
				}
			}		  
		}
		#endif
		
		if((MAC_COMMAND_ANS_status==1 && response_level==3) 
		|| (unconfirmed_downlink_data_ans_status==1 && response_level==1 && is_there_data==1 ) 
		|| (confirmed_downlink_data_ans_status==1 && response_level==2 && is_there_data==1 )
		||(((MAC_COMMAND_ANS_status==1)||(confirmed_downlink_data_ans_status==1&&is_there_data==1))&&(response_level==4)))
		{
			if((( LoRaMacState & 0x00000001 ) != 0x00000001)&&(( LoRaMacState & 0x00000010 ) != 0x00000010))
			{
				MAC_COMMAND_ANS_status=0;
				unconfirmed_downlink_data_ans_status=0;
				confirmed_downlink_data_ans_status=0;
				is_there_data=0;
				AppData.Buff[0]=0x00;
				AppData.BuffSize=1;
				AppData.Port = 2;
	      LORA_send( &AppData, LORAWAN_UNCONFIRMED_MSG);
			}
		}
		
		find_timestamp1_on_flash();
		
		Reply_to_retrieval_request();
		
		if(is_time_to_rejoin==1)
		{
			if((( LoRaMacState & 0x00000001 ) != 0x00000001)&&(( LoRaMacState & 0x00000010 ) != 0x00000010))
			{
			  is_time_to_rejoin=0;
			  LORA_Join();
			}
		}
		
		if(JoinReq_NbTrails_over==1)
		{
			JoinReq_NbTrails_over=0;
			
			rejoin_keep_status=1;
			
			if(REJOIN_TX_DUTYCYCLE==0)
			{
				REJOIN_TX_DUTYCYCLE=20;
			}
			LoraStartRejoin(TX_ON_EVENT);
		}
		
		if(rejoin_status==1)
		{
			rejoin_keep_status=1;
			TimerStop(&TxTimer);
			LORA_Join();
		}
		
//    if((Ext==0x04)&&(exit_fr==1)&&(( LoRaMacState & 0x00000001 ) == 0x00000001)&&(( LoRaMacState & 0x00000001 ) != 0x00000010))		
//		{
//			IWDG_Refresh();
//			DelayMs(8000);
//			exit_fr=0;
//			Send();			
//		}
		
//		if(Ext4_is_time_to_comp_record_status==1)
//		{
//			if((( LoRaMacState & 0x00000001 ) != 0x00000001) && (( LoRaMacState & 0x00000010 ) != 0x00000010))
//			{
//				Ext4_is_time_to_comp_record_status=0;
//				uint8_t current_level=HAL_GPIO_ReadPin(GPIO_Exit_PORT,GPIO_Exit_PIN);		
//				if(current_level!=Ext4_record_status)
//				{
//					Send();
//				}
//			}
//		}
		
		if(is_time_to_IWDG_Refresh==1)
		{
			is_time_to_IWDG_Refresh=0;
			IWDG_Refresh();
		}
		
		#if defined(LoRa_Sensor_Node)
		user_key_event();
		#endif
		
    DISABLE_IRQ( );
    /*
     * if an interrupt has occurred after DISABLE_IRQ, it is kept pending
     * and cortex will not enter low power anyway
     * don't go in low power mode if we just received a char
     */
#ifndef LOW_POWER_DISABLE
    LPM_EnterLowPower();
#endif
    ENABLE_IRQ();
    
    /* USER CODE BEGIN 2 */
    /* USER CODE END 2 */
  }
}

static void LORA_HasJoined( void )
{	
  LORA_RequestClass( LORAWAN_DEFAULT_CLASS );

	is_lora_joined=1;
}

static void Send( void )
{
  sensor_t bsp_sensor_data_buff;
  uint16_t bat_status=0;
	bool ext_connect=0;
	bool compflgas=0;
	
  if ( LORA_JoinStatus () != LORA_SET)
  {
    /*Not joined, try again later*/
    return;
  }

  Poll_Message_Flag=0;
	SysTime_t sysTimeCurrent = { 0 };
	sysTimeCurrent=SysTimeGet();
	
	HW_GetBatteryLevel( );
	BSP_sensor_Read( &bsp_sensor_data_buff);
	ext_connect=connect_flags_temp;
	
	if(is_time_to_send_ds18b20_id==0)
	{
    	if(Ext==0x05)
			{
				read_sensor_data(sysTimeCurrent.Seconds,Ext,batteryLevel_mV,bsp_sensor_data_buff.temp_sht*100,bsp_sensor_data_buff.hum_sht*10,bsp_sensor_data_buff.illuminance,0x7fff);				
			}
			else if(Ext==0x06)
			{
				read_sensor_data(sysTimeCurrent.Seconds,Ext,batteryLevel_mV,bsp_sensor_data_buff.temp_sht*100,bsp_sensor_data_buff.hum_sht*10,bsp_sensor_data_buff.adc_PA4,0x7fff);				
			}
			else if(Ext==0x07 || Ext==0x08)
			{
				read_sensor_data(sysTimeCurrent.Seconds,Ext,batteryLevel_mV,bsp_sensor_data_buff.temp_sht*100,bsp_sensor_data_buff.hum_sht*10,count,0x7fff);						
			}
			else
			{
				read_sensor_data(sysTimeCurrent.Seconds,Ext,batteryLevel_mV,bsp_sensor_data_buff.temp_sht*100,bsp_sensor_data_buff.hum_sht*10,bsp_sensor_data_buff.temp_ds*100,0x7fff);
      }
			
			write_address=find_addr(write_address);
			write_address=store_sensor_data(write_address,bsp_sensor_data);
	}
	#if defined(LoRa_Sensor_Node)
	
	uint8_t i = 0;

  AppData.Port = lora_config_application_port_get();
	
	if(batteryLevel_mV>=2650)
	{
		bat_status=0xC000;
	}
	else if(batteryLevel_mV>=2550 && batteryLevel_mV<2650)
	{
		bat_status=0x8000;
	}	
	else if(batteryLevel_mV>=2500 && batteryLevel_mV<2550)
	{
		bat_status=0x4000;
	}
	else
	{
		bat_status=0;
	}

	compflgas=comp_ds18b20_id();
	if(((Ext&0x0F)==0x01 || (Ext&0x0F)==0x09)&&(pid_flag==1)&&((compflgas==1)||(is_time_to_send_ds18b20_id==1)))
	{
		AppData.Buff[i++] =(batteryLevel_mV|bat_status)>>8;       //level of battery in mV
		AppData.Buff[i++] =batteryLevel_mV & 0xFF;
		
		is_time_to_send_ds18b20_id=0;
		
		for(int k=0;k<8;k++)// first send ID
		{
			AppData.Buff[i++]=(int) DS18B20_ID[0][k];     //DS18B20 ID  		
			if(k==3)
			{
				AppData.Buff[i++]=0x0f;
			}						
		}
		
		if(compflgas==1)
		{
		  is_time_to_send=1;//second send temp	
		}
	}
	else
	{	
		is_time_to_send=0;
		if((Ext & 0x0F)!=0x09)
		{
			AppData.Buff[i++] =(batteryLevel_mV|bat_status)>>8;       //level of battery in mV
			AppData.Buff[i++] =batteryLevel_mV & 0xFF;
		}
		else
		{
			AppData.Buff[i++]=(int)(bsp_sensor_data_buff.temp_ds*100)>>8;     //DS18B20
			AppData.Buff[i++]=(int)(bsp_sensor_data_buff.temp_ds*100);
		}
		
		if(work_mode==1)
		{
			if(beyond_status==1)
			{
				beyond_status=0;
				AppData.Buff[i++] =(int)(collection_temp_sht20*100)>>8;      //SHT20
				AppData.Buff[i++] =(int)(collection_temp_sht20*100);
			}
			else
			{
				AppData.Buff[i++] =(int)(bsp_sensor_data_buff.temp_sht*100)>>8;      //SHT20
				AppData.Buff[i++] =(int)(bsp_sensor_data_buff.temp_sht*100);
			}
		}
		else if(work_mode==0)
		{
			AppData.Buff[i++] =(int)(bsp_sensor_data_buff.temp_sht*100)>>8;      //SHT20
			AppData.Buff[i++] =(int)(bsp_sensor_data_buff.temp_sht*100);
		}
		
		if((Ext & 0x0F)!=0x09)
		{
			AppData.Buff[i++] =(int)(bsp_sensor_data_buff.hum_sht*10)>>8; 
			AppData.Buff[i++] =(int)(bsp_sensor_data_buff.hum_sht*10);
		}
		else
		{
			AppData.Buff[i++] =((int)(bsp_sensor_data_buff.hum_sht*10)|bat_status)>>8; //Ext=8
			AppData.Buff[i++] =(int)(bsp_sensor_data_buff.hum_sht*10);
		}
		
		if((Ext&0x0F)==0x09)
		{
			if(time_synchronization_method==0)
			{
				AppData.Buff[i++]=Ext | Poll_Message_Flag<<6 | UTC_Accept<<5 | UTC_Request<<4;
			}
			else
			{
				AppData.Buff[i++]=Ext;
			}
		}else
		{
			AppData.Buff[i++]=Ext|(ext_connect<<7);	
		}
		
		if((Ext&0x0F)==0x01)
		{
			AppData.Buff[i++]=(int)(bsp_sensor_data_buff.temp_ds*100)>>8;     //DS18B20
			AppData.Buff[i++]=(int)(bsp_sensor_data_buff.temp_ds*100);
			AppData.Buff[i++]=0x7F;
			AppData.Buff[i++]=0xFF;
		}
		else if((Ext&0x0F)==0x04)
		{
			level_status=HAL_GPIO_ReadPin(GPIO_Exit_PORT,GPIO_Exit_PIN);		
			
			AppData.Buff[i++]=level_status;
			
//			Ext4_record_status=level_status;
//			Ext4_is_time_to_comp_record_status=1;
			
			if(exti_flag==1)
			{			
				AppData.Buff[i++]=0x01;
				exti_flag=0;
			}
			else
			{
				AppData.Buff[i++]=0x00;
			}
			
			AppData.Buff[i++]=0x7F;
			AppData.Buff[i++]=0xFF;	
		}		
		else if((Ext&0x0F)==0x05)
		{
			AppData.Buff[i++]=(int)(bsp_sensor_data_buff.illuminance)>>8;     //bh1750
			AppData.Buff[i++]=(int)(bsp_sensor_data_buff.illuminance);
			AppData.Buff[i++]=0x7F;
			AppData.Buff[i++]=0xFF;		
		}
		else if((Ext&0x0F)==0x06)
		{
			AppData.Buff[i++]=(int)(bsp_sensor_data_buff.adc_PA4)>>8;     //ADC_PA4
			AppData.Buff[i++]=(int)(bsp_sensor_data_buff.adc_PA4);
			AppData.Buff[i++]=0x7F;
			AppData.Buff[i++]=0xFF;		
		}
		else if((Ext&0x0F)==0x07)
		{
			AppData.Buff[i++]=(int)count>>8;     //Interrupt count
			AppData.Buff[i++]=(int)count;
			AppData.Buff[i++]=0x7F;
			AppData.Buff[i++]=0xFF;				
		}
		else if((Ext&0x0F)==0x08)
		{
			AppData.Buff[i++]=(int)count>>24;     //Interrupt count
			AppData.Buff[i++]=(int)count>>16;
			AppData.Buff[i++]=(int)count>>8;     
			AppData.Buff[i++]=(int)count;			
		}
		else if((Ext&0x0F)==0x09)
		{
			AppData.Buff[i++]=sysTimeCurrent.Seconds>>24 & 0xFF;     
			AppData.Buff[i++]=sysTimeCurrent.Seconds>>16 & 0xFF;
			AppData.Buff[i++]=sysTimeCurrent.Seconds>>8 & 0xFF;
			AppData.Buff[i++]=sysTimeCurrent.Seconds & 0xFF;	
		}
	}
	AppData.BuffSize = i;
	payloadlens=i;
		
	LORA_send( &AppData, lora_config_reqack_get());
	#endif	
}

static void LORA_RxData( lora_AppData_t *AppData )
{
  is_there_data=1;
  set_at_receive(AppData->Port, AppData->Buff, AppData->BuffSize);
	
	TimerInit( &downlinkLedTimer, OndownlinkLedEvent );
  TimerSetValue(  &downlinkLedTimer, 200);
  HAL_GPIO_WritePin(LED_RGB_PORT,LED_RED_PIN|LED_BLUE_PIN,GPIO_PIN_SET); 
  TimerStart( &downlinkLedTimer );
	
	if(AppData->BuffSize<8)
	{
		AT_PRINTF("Receive data\n\r");
		AT_PRINTF("%d:",AppData->Port);
		 for (int i = 0; i < AppData->BuffSize; i++)
		{
			AT_PRINTF("%02x", AppData->Buff[i]);
		}
		AT_PRINTF("\n\r");
  }
  else
	{
		AT_PRINTF("Run AT+RECVB=? to see detail\r\n");			
	}
      switch(AppData->Buff[0] & 0xff)
      {				
				case 1:
				{
					if( AppData->BuffSize == 4 )
					{
					    ServerSetTDC=( AppData->Buff[1]<<16 | AppData->Buff[2]<<8 | AppData->Buff[3] );//S
							if(ServerSetTDC<5)
							{
							  PRINTF("TDC setting needs to be high than 4s\n\r");
							}
							else
							{
							  TDC_flag=1;
			          APP_TX_DUTYCYCLE=ServerSetTDC*1000;
							}						
					}
					break;
				}
				
        case 4:
				{
					if( AppData->BuffSize == 2 )
					{
						if(AppData->Buff[1]==0xFF)  //---->ATZ
						{
							atz_flags=1;		
						}
						else if(AppData->Buff[1]==0xFE)  //---->AT+FDR
						{	
							uint32_t status[1]={0x12};					
							EEPROM_program(DATA_EEPROM_BASE,status,1);	
							EEPROM_erase_lora_config();					
							atz_flags=1;													
						}
					}
					break;
				}	
				
        case 5:
				{
					if( AppData->BuffSize == 2 )
					{
						if(AppData->Buff[1]<2)
						{
							if(AppData->Buff[1]==0x01)
							{
								lora_config_reqack_set(LORAWAN_CONFIRMED_MSG);
							}
							else if(AppData->Buff[1]==0x00)
							{
								lora_config_reqack_set(LORAWAN_UNCONFIRMED_MSG);
							}
							EEPROM_Store_Config();
					  }
				  }
					break;
				}
				
				case 7:
				{
					if( AppData->BuffSize == 2 )
					{
						#if defined( REGION_US915 )	|| defined( REGION_AU915 )
						if(AppData->Buff[1]<9)
						{
							customize_set8channel_set(AppData->Buff[1]);
							EEPROM_Store_Config();
							DelayMs(100);
							NVIC_SystemReset();
						}
						#elif defined( REGION_CN470 )
						if(AppData->Buff[1]<13)
						{
							customize_set8channel_set(AppData->Buff[1]);
							EEPROM_Store_Config();
							DelayMs(100);
							NVIC_SystemReset();
						}
						#endif
				  }
					break;
				}
				
        case 0xA2:
				{
					if( (AppData->BuffSize == 2)&&((AppData->Buff[1]==0x00)||(AppData->Buff[1]==0x01)||(AppData->Buff[1]==0x05)||(AppData->Buff[1]==0x09)) )
					{
						Ext=AppData->Buff[1];
						GPIO_EXTI4_IoDeInit();
						
						if(Ext==0x09)
						{
							StartCalibrationUTCtime(TX_ON_EVENT);
						}
						EEPROM_Store_Config();								
					}
					else if((AppData->BuffSize == 3)&&(AppData->Buff[1]==0x04))
					{
						Ext=AppData->Buff[1];
						exitintmode=AppData->Buff[2];
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
						EEPROM_Store_Config();							
					}
					else if(((AppData->BuffSize == 4)||(AppData->BuffSize == 5))&&(AppData->Buff[1]==0x07||AppData->Buff[1]==0x08))
					{
						Ext=AppData->Buff[1];
						if(AppData->Buff[2]==0x01)
						{
							exitmode=AppData->Buff[3];
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
						else if(AppData->Buff[2]==0x02)
						{
							count=AppData->Buff[3]<<8 | AppData->Buff[4];
						}
						EEPROM_Store_Config();							
					}
					else if((AppData->BuffSize == 4)&&(AppData->Buff[1]==0x06))
					{
						Ext=AppData->Buff[1];
						power_time=AppData->Buff[2]<<8 | AppData->Buff[3];
						GPIO_EXTI4_IoDeInit();
						EEPROM_Store_Config();							
					}
					break;
				}
				
				case 0xA3:
				{
					if( AppData->BuffSize == 2 )
					{
						if(AppData->Buff[1]==0x01)
						{
							TimerStop(&TxTimer);
							PRINTF("Clear all stored sensor data...\r");
					    FLASH_erase_all_sensor_data_storage(FLASH_SENSOR_DATA_START_ADDR);
							PRINTF("OK\r");
							write_address=FLASH_SENSOR_DATA_START_ADDR;
							if(work_mode==1)
							{
								Startsht20tempcomptime(TX_ON_TIMER);
							}
							TimerStart(&TxTimer);
						}
				  }
					break;
				}
				
				case 0xA5:
				{
					if( AppData->BuffSize == 2 )
					{
						if(AppData->Buff[1]<2)
						{
              work_mode=AppData->Buff[1];
							if(work_mode==1)
							{
								Startsht20tempcomptime(TX_ON_TIMER);
							}
							else
								TimerStop( &sht20tempcompTimer);
							
							EEPROM_Store_Config();
					  }
				  }
					break;
				}
				
				case 0xA6:
				{
					if( AppData->BuffSize == 3 )
					{
						collection_interval=( AppData->Buff[1]<<8 | AppData->Buff[2] );//min
						if(collection_interval==0)
						{
							collection_interval=1;
						}					
						
						EEPROM_Store_Config();
				  }
					break;
				}
				
				case 0xA7:
				{
					if( AppData->BuffSize == 5 )
					{
						comp_temp1_sht20=( AppData->Buff[1]<<8 | AppData->Buff[2] );					
						comp_temp2_sht20=( AppData->Buff[3]<<8 | AppData->Buff[4] );
						EEPROM_Store_Config();
				  }
					break;
				}

				case 0xA8:
				{
					if(( AppData->BuffSize == 2)&&(AppData->Buff[1]<=1))
					{
						pid_flag = AppData->Buff[1];					
						EEPROM_Store_Config();
						
						if(pid_flag==0)
						{
							TimerStop( &DS18B20IDTimer );
						}
						else
						{
							if(Ext==0x01||Ext==0x09)
							{
								StartDS18B20ID(TX_ON_EVENT);
							}
						}
				  }
					break;
				}
				
				case 0x20:			
				{
					if( AppData->BuffSize == 2 )
					{		
						if((AppData->Buff[1]==0x00)||(AppData->Buff[1]==0x01))    
						{
							if(AppData->Buff[1]==0x01)       //---->AT+NJM=1
							{
								lora_config_otaa_set(LORA_ENABLE);
							}
							else                             //---->AT+NJM=0
							{
								lora_config_otaa_set(LORA_DISABLE);							
							}
							EEPROM_Store_Config();
							atz_flags=1;					
						}						 
					}
					break;				
				}	
				
				case 0x21:
				{
					if( (AppData->BuffSize == 2) && (AppData->Buff[1]<=4) )
					{
						response_level=( AppData->Buff[1] );//0~4					//---->AT+RPL
						EEPROM_Store_Config();							
					}
					break;
				}		
				
				case 0x22:			
				{
					MibRequestConfirm_t mib;
					if(( AppData->BuffSize == 2 )&&(AppData->Buff[1]==0x01))   //---->AT+ADR=1
					{		
						mib.Type = MIB_ADR;
						mib.Param.AdrEnable =AppData->Buff[1];
						LoRaMacMibSetRequestConfirm( &mib );					
						EEPROM_Store_Config();							
					}
					else if((AppData->BuffSize == 4 )&&(AppData->Buff[1]==0x00))   //---->AT+ADR=0
					{
						uint8_t downlink_data_rate=AppData->Buff[2];
						mib.Type = MIB_ADR;					
						mib.Param.AdrEnable = AppData->Buff[1];
						LoRaMacMibSetRequestConfirm( &mib );	
						
						#if defined(REGION_US915)
						if(downlink_data_rate>3)
						{
							downlink_data_rate=3;   
						}
						#elif defined(REGION_AS923) || defined(REGION_AU915)
						if(dwelltime==1)
						{
							if(downlink_data_rate>5)
							{
								downlink_data_rate=5;
							}else if(downlink_data_rate<2)
							{
								downlink_data_rate=2;
							}
						}
						#else
						if(downlink_data_rate>5)
						{
							downlink_data_rate=5;
						}
						#endif	
						
						lora_config_tx_datarate_set(downlink_data_rate) ;
						
						if(AppData->Buff[3]!=0xff)                //---->AT+TXP
						{
							mib.Type = MIB_CHANNELS_TX_POWER;						
							mib.Param.ChannelsTxPower=AppData->Buff[3];
							LoRaMacMibSetRequestConfirm( &mib );							
						}				
						EEPROM_Store_Config();									
					 }
					 break;				
				}			
				
				case 0x23:			
				{
					if( AppData->BuffSize == 2 )
					{		
						lora_config_application_port_set(AppData->Buff[1]);    //---->AT+PORT
						EEPROM_Store_Config();						 
					}
					break;					
				}		
				
				case 0x25:			
				{
				 #if defined( REGION_AS923 )	|| defined( REGION_AU915 )
				 if( AppData->BuffSize == 2 )
				 {				
					 if((AppData->Buff[1]==0x00)||(AppData->Buff[1]==0x01))   //---->AT+DWELLT
					 {
						 dwelltime=AppData->Buff[1];
						 EEPROM_Store_Config();
						 atz_flags=1;		
					 }						
				 }
				 #endif	
				 break;				
				}
				
				case 0x26:
				{
					if( AppData->BuffSize == 3 )
					{
						uint16_t value;			
						
						value=( AppData->Buff[1]<<8 | AppData->Buff[2] );//1~65535
						
						if(value>0)
						{
							REJOIN_TX_DUTYCYCLE=value;
							EEPROM_Store_Config();
						}					
					}
					break;
				}
				
				case 0x27:
				{
					if( AppData->BuffSize == 2 )
					{				
						currentLeapSecond=AppData->Buff[1];
						EEPROM_Store_Config();					
					}
					break;
				}

				case 0x28:
				{
					if( AppData->BuffSize == 2 )
					{				
						time_synchronization_method=AppData->Buff[1];
						if(time_synchronization_method>1)
						{
							time_synchronization_method=1;
						}
						
						if(Ext==0x09||time_synchronization_method==1)
						{
							StartCalibrationUTCtime(TX_ON_EVENT);
						}
						
						EEPROM_Store_Config();					
					}
					break;
				}
				
				case 0x29:
				{
					if( AppData->BuffSize == 2 )
					{						
						time_synchronization_interval=AppData->Buff[1];
						if(time_synchronization_interval==0)
						{
							time_synchronization_interval=10;
						}
						EEPROM_Store_Config();					
					}
					break;
				}

				case 0x30:
				{
					SysTime_t sysTime = { 0 };
					SysTime_t sysTimeCurrent = { 0 };
					SysTime_t downlinkTime = { 0 };
				
					if( AppData->BuffSize == 6 )
					{				
						downlinkTime.Seconds = ( uint32_t )AppData->Buff[1]<<24;
						downlinkTime.Seconds |= ( uint32_t )AppData->Buff[2]<<16;
						downlinkTime.Seconds |= ( uint32_t )AppData->Buff[3]<<8;
						downlinkTime.Seconds |= ( uint32_t )AppData->Buff[4];
						downlinkTime.SubSeconds = AppData->Buff[5];	
						downlinkTime.SubSeconds = ( int16_t )( ( ( int32_t )downlinkTime.SubSeconds * 1000 ) >> 8 );
						
						sysTime=downlinkTime;

						sysTimeCurrent = SysTimeGet( );
						sysTime = SysTimeAdd( sysTime, SysTimeSub( sysTimeCurrent, LastTxdoneTime ) );

						if(sysTime.Seconds>1611878400)//20210129 00:00:00
						{											
							SysTimeSet( sysTime );
							
							sysTimeCurrent=SysTimeGet();	
							
							UTC_Request=0;
							UTC_Accept=1;
							
							PRINTF("Set current timestamp=%u\r",sysTimeCurrent.Seconds);
						}
						else
						{
							PRINTF("timestamp error\r");
						}
					}
					break;
				}
				
				case 0x31:
				{
					if( AppData->BuffSize == 10 )
					{
							uint32_t value,time;
							timestamp1=( AppData->Buff[1]<<24 | AppData->Buff[2]<<16 | AppData->Buff[3]<<8 | AppData->Buff[4]);
							timestamp2=( AppData->Buff[5]<<24 | AppData->Buff[6]<<16 | AppData->Buff[7]<<8 | AppData->Buff[8]);
							time=AppData->Buff[9];
						
							if(time<5)
							{
								retrieval_uplink_time=5;
							}
							else
								retrieval_uplink_time=time;
							
							if(timestamp1>timestamp2)
							{
								value=timestamp1;
								timestamp2=timestamp1;
								timestamp1=value;
							}
							is_time_to_find_timestamp1_on_flash=1;
					}
					break;
				}
				
				default:
					break;
			}
	
	if(TDC_flag==1)
	{
		EEPROM_Store_Config();
		TimerInit( &TxTimer, OnTxTimerEvent );
    TimerSetValue( &TxTimer,  APP_TX_DUTYCYCLE); 
    TimerStart( &TxTimer);
		TimerStart( &IWDGRefreshTimer);				
		TDC_flag=0;
	}
}

/**
  * @brief  EXTI line detection callbacks.
  * @param  GPIO_Pin: Specifies the pins connected to the EXTI line.
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  HW_GPIO_IrqHandler( GPIO_Pin );
}

void sht20_temp_comp(void)
{
	collection_temp_sht20=SHT20_RT();
	if(collection_temp_sht20<comp_temp1_sht20||collection_temp_sht20>comp_temp2_sht20)
	{		
		beyond_status=1;
		is_time_to_send_beyond_data=1;
//		Send();
	}
}

static void OnTxTimerEvent( void )
{
	TimerSetValue( &TxTimer,  APP_TX_DUTYCYCLE);
	
  /*Wait for next tx slot*/
  TimerStart( &TxTimer);
	
	if((exti_flag==1)&&(Ext==0x04))
	{
		
	}
	else
	{
	 is_time_to_send=1;
	}
}

static void LoraStartTx(TxEventType_t EventType)
{
  if (EventType == TX_ON_TIMER)
  {
    /* send everytime timer elapses */
    TimerInit( &TxTimer, OnTxTimerEvent );
    TimerSetValue( &TxTimer,  APP_TX_DUTYCYCLE); 
    OnTxTimerEvent();
  }
}

static void OnIWDGRefreshTimeoutEvent( void )
{
	TimerSetValue( &IWDGRefreshTimer,  18000);

  TimerStart( &IWDGRefreshTimer);
	
	if(is_PDTA_command==1 || is_PLDTA_command==1)
	{
		IWDG_Refresh();
	}
	else
	  is_time_to_IWDG_Refresh=1;
}

static void StartIWDGRefresh(TxEventType_t EventType)
{
  if (EventType == TX_ON_EVENT)
  {
    /* send everytime timer elapses */
    TimerInit( &IWDGRefreshTimer, OnIWDGRefreshTimeoutEvent );
    TimerSetValue( &IWDGRefreshTimer,  18000); 
		TimerStart( &IWDGRefreshTimer);
  }
}

static void LORA_ConfirmClass ( DeviceClass_t Class )
{
  PRINTF("switch to class %c done\n\r","ABC"[Class] );

  /*Optionnal*/
  /*informs the server that switch has occurred ASAP*/
  AppData.BuffSize = 0;
  AppData.Port = LORAWAN_APP_PORT;
  
  LORA_send( &AppData, LORAWAN_UNCONFIRMED_MSG);
}

void user_key_event(void)
{
	if(user_key_exti_flag==1)
	{
		user_key_exti_flag=0;
		
		if(OnPressButtonTimeout_status==0)
		{
			OnPressButtonTimeout_status=1;
			TimerInit( &PressButtonTimeoutTimer, OnPressButtonTimeoutEvent );
			TimerSetValue( &PressButtonTimeoutTimer, 5000);
			TimerStart( &PressButtonTimeoutTimer );
		}
		
		press_button_times++;
		HAL_GPIO_WritePin(LED_RGB_PORT,LED_GREEN_PIN,GPIO_PIN_SET);
			
		uint32_t currentTime = TimerGetCurrentTime();
		
		while(HAL_GPIO_ReadPin(GPIO_USERKEY_PORT,GPIO_USERKEY_PIN)==GPIO_PIN_RESET)
		{				
			if(TimerGetElapsedTime(currentTime) >= 1000 && TimerGetElapsedTime(currentTime) < 3000)//send
			{			
			  user_key_duration=1;
			}			
			else if(TimerGetElapsedTime(currentTime) >= 3000)//system reset,Activation Mode
			{ 
        press_button_times=0;					
				for(int i=0;i<10;i++)
				{
					HAL_GPIO_TogglePin(LED_RGB_PORT,LED_GREEN_PIN);
					DelayMs(100);
				}
				user_key_duration=3;
				break;
			}			
    }
		
		HAL_GPIO_WritePin(LED_RGB_PORT,LED_GREEN_PIN,GPIO_PIN_RESET);
		
		if(press_button_times==5)
		{	
			press_button_times=0;
			user_key_duration=2;
		}
			
		switch(user_key_duration)
		{
			case 1:
			{
				user_key_duration=0;
				
				if(sleep_status==0 && (LORA_JoinStatus () == LORA_SET))
				{
					TimerStop(&TxTimer);
					TimerSetValue( &TxTimer,  APP_TX_DUTYCYCLE);
					TimerStart(&TxTimer);
				  Send();
				}
				break;
			}
			
			case 2://sleep
			{
				sleep_status=1;
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
				
				DelayMs(500);
				TimerInit( &PressButtonTimesLedTimer, OnPressButtonTimesLedEvent );
				TimerSetValue( &PressButtonTimesLedTimer, 5000);
				HAL_GPIO_WritePin(LED_RGB_PORT,LED_RED_PIN,GPIO_PIN_SET); 
				TimerStart( &PressButtonTimesLedTimer );
				user_key_duration=0;
				break;
			}
			
			case 3://system reset,Activation Mode
			{
				user_key_duration=0;
				NVIC_SystemReset();
				break;
			}
			
			default:
				break;
		}
	}
}

void read_sensor_data(uint32_t timestamp,uint8_t dev,uint16_t bat,float temp20,float hum20,float extsen,uint16_t rev)
{	
	/*
	 Year Month Day Hour | Min Dev Battery | Temp_sht20 Hum_sht20 | Rev
	 xx   xx    xx  xx   | xx  xx  xxxx    | xxxx       xxxx      | xxxx xxxx
	 total: 16bytes
	 */
	bsp_sensor_data[0]=timestamp;
	bsp_sensor_data[1]=dev<<16|(bat>>8)<<8|(bat&0xff);
	bsp_sensor_data[2] =(((short)(temp20)>>8)<<24)&0xff000000;
	bsp_sensor_data[2] +=((short)(temp20)<<16)&0x00ff0000;
	bsp_sensor_data[2] +=(((short)(hum20)>>8)<<8)&0x0000ff00;
	bsp_sensor_data[2] +=(short)(hum20)&0xff;
	
	if((dev==0x01)||(dev==0x05)||(dev==0x06)||(dev==0x09))
	{
	  bsp_sensor_data[3]=((short)(extsen)>>8)<<24|(short)(extsen)<<16|(int)(rev>>8)<<8|(rev&0xff);
	}
	else if(dev==0x04)
	{
		bsp_sensor_data[3]=(level_status<<24)|(exti_flag<<16)|(int)(rev>>8)<<8|(rev&0xff);
	}	
	else if(dev==0x07)
	{
		bsp_sensor_data[3]=(count>>8&0xff)<<24|(count&0xff)<<16|(rev>>8)<<8|(rev&0xff);
	}
	else if(dev==0x08)
	{
		bsp_sensor_data[3]=(count>>24&0xff)<<24|(count>>16&0xff)<<16|(count>>8&0xff)<<8|(count&0xff);
	}	
	else
		bsp_sensor_data[3]=0;
}

void OndownlinkLedEvent(void)
{
	TimerStop(&downlinkLedTimer);
	HAL_GPIO_WritePin(LED_RGB_PORT,LED_RED_PIN|LED_BLUE_PIN,GPIO_PIN_RESET);
}

void OnNetworkJoinedLedEvent(void)
{
	TimerStop(&NetworkJoinedLedTimer);
	HAL_GPIO_WritePin(LED_RGB_PORT,LED_RED_PIN|LED_GREEN_PIN|LED_BLUE_PIN,GPIO_PIN_RESET);
}

void OnPressButtonTimesLedEvent(void)
{
	TimerStop(&PressButtonTimesLedTimer);
	HAL_GPIO_WritePin(LED_RGB_PORT,LED_RED_PIN,GPIO_PIN_RESET);
}

void OnPressButtonTimeoutEvent(void)
{
	TimerStop(&PressButtonTimeoutTimer);
	OnPressButtonTimeout_status=0;
	press_button_times=0;
}

void send_exti(void)
{
	if((exti_flag==1)&&(Ext==0x04))
	{
			if((( LoRaMacState & 0x00000001 ) != 0x00000001) && (( LoRaMacState & 0x00000010 ) != 0x00000010))
			{
				Send();
			}
	}
	else if((exti_flag2==1)&&(Ext==0x07 || Ext==0x08))
	{
	  if(debug_flags==1)
	  {
		  PRINTF("Count:%d\r\n",count);
	  }
		exti_flag2=0;
	}
}

static void find_timestamp1_on_flash(void)//find the address where timestamp1 is stored
{
	if(is_time_to_find_timestamp1_on_flash==1)
	{
		is_time_to_find_timestamp1_on_flash=0;
		find_timestamp_addr=FLASH_SENSOR_DATA_START_ADDR;
		StartRetrieval(TX_ON_EVENT);
	}
}

static void Reply_to_retrieval_request(void)
{
	if(is_time_to_send_retrieval_data==1)
	{
		uint32_t buff[4],nextdata;
		uint8_t find_status=1,j=0;
		
		if(LoRaMacState!=0x00000010 && LoRaMacState!=0x00000001)
		{
		  is_time_to_send_retrieval_data=0;
			
			TimerSetValue( &RetrievalTimer,  retrieval_uplink_time*1000); 
			TimerStart(&RetrievalTimer);
			
			if ( LORA_JoinStatus () != LORA_SET)
			{
				/*Not joined, try again later*/
				return;
			}
		
			for(int i=0;i<4;i++)
			{
				buff[i] = *(__IO uint32_t *)find_timestamp_addr;
				find_timestamp_addr = find_timestamp_addr + 4;
			}
			
			while(buff[0]>timestamp2 || buff[0]<timestamp1 || buff[0]==0)
			{
				if(find_timestamp_addr>FLASH_SENSOR_DATA_END_ADDR)
				{		
					find_status=0;
					break;
				}
				
        if(buff[0]>=timestamp1 && buff[0]<=timestamp2 )			
				{										
          find_status=1;
					break;
				}
				
				for(int i=0;i<4;i++)
				{
					buff[i] = *(__IO uint32_t *)find_timestamp_addr;
					find_timestamp_addr = find_timestamp_addr + 4;
				}
			}
			
			if(find_status==1)
			{
				if(((buff[1]>>16)&0x0F)==0x08)
				{
					buff[3]=buff[3]%65535;
					AppData.Buff[j++] =(buff[3]>>8) & 0xFF;       //ext sensor data
					AppData.Buff[j++] =buff[3] & 0xFF;
				}
        else
				{
					AppData.Buff[j++] =(buff[3]>>24) & 0xFF;       //ext sensor data
					AppData.Buff[j++] =buff[3]>>16 & 0xFF;
				}
					AppData.Buff[j++] =(buff[2]>>24) &0xFF;        //sht_20 temp
					AppData.Buff[j++] =buff[2]>>16 & 0xFF;
				
					AppData.Buff[j++] =(buff[2]>>8) &0xFF;         //sht_20 hum
					AppData.Buff[j++] =buff[2] & 0xFF;
					
				  AppData.Buff[j++]=((buff[1]>>16) & 0x0F)|0x40; //Ext & Poll message
				
					AppData.Buff[j++] =buff[0]>>24 & 0xFF;		     //timestamp	
					AppData.Buff[j++] =buff[0]>>16 & 0xFF;
					AppData.Buff[j++] =buff[0]>>8 & 0xFF;
					AppData.Buff[j++] =buff[0] & 0xFF;
					
					AppData.BuffSize = j;
					
				  AppData.BuffSize +=11;
					while(Uplink_data_adaptive_rate(&AppData)==1)
					{
						for(int i=0;i<4;i++)
						{
							buff[i] = *(__IO uint32_t *)find_timestamp_addr;
							find_timestamp_addr = find_timestamp_addr + 4;
						}
						
						if(buff[0]>=timestamp1 && buff[0]<=timestamp2 )			
						{										
							if(((buff[1]>>16)&0x0F)==0x08)
							{
								buff[3]=buff[3]%65535;
								AppData.Buff[j++] =(buff[3]>>8) & 0xFF;       //ext sensor data
								AppData.Buff[j++] =buff[3] & 0xFF;
							}
							else
							{
								AppData.Buff[j++] =(buff[3]>>24) & 0xFF;       //ext sensor data
								AppData.Buff[j++] =buff[3]>>16 & 0xFF;
							}

							AppData.Buff[j++] =(buff[2]>>24) &0xFF;        //sht_20 temp
							AppData.Buff[j++] =buff[2]>>16 & 0xFF;
						
							AppData.Buff[j++] =(buff[2]>>8) &0xFF;         //sht_20 hum
							AppData.Buff[j++] =buff[2] & 0xFF;
							
							AppData.Buff[j++]=((buff[1]>>16) & 0x0F)|0x40;        //Ext
						
							AppData.Buff[j++] =buff[0]>>24 & 0xFF;		     //timestamp	
							AppData.Buff[j++] =buff[0]>>16 & 0xFF;
							AppData.Buff[j++] =buff[0]>>8 & 0xFF;
							AppData.Buff[j++] =buff[0] & 0xFF;
							
							AppData.BuffSize = j;
							
							AppData.BuffSize +=11;
						}												
						
						if(find_timestamp_addr>FLASH_SENSOR_DATA_END_ADDR)
						{									
							break;
						}
					}
					
					AppData.BuffSize -=11;
			}
			
			if(find_timestamp_addr>FLASH_SENSOR_DATA_END_ADDR)
			{
				PRINTF("\n\rsend retrieve data completed\r");
				TimerStop(&RetrievalTimer);
			}
			
			find_timestamp_addr_record=find_timestamp_addr;
			while(find_timestamp_addr_record<=FLASH_SENSOR_DATA_END_ADDR)
			{
				nextdata=*(__IO uint32_t *)find_timestamp_addr_record;
				find_timestamp_addr_record = find_timestamp_addr_record + 16;
				
				if(nextdata>=timestamp1 && nextdata<=timestamp2)
				{
					break;
				}
				
				if(find_timestamp_addr_record>FLASH_SENSOR_DATA_END_ADDR)
				{	
					PRINTF("\n\rsend retrieve data completed\r");					
					TimerStop(&RetrievalTimer);
					break;
				}
			}

			if(find_status==0)
			{
				TimerStop(&RetrievalTimer);
				PRINTF("\rNo data retrieved\r");
				uint8_t i = 0;

				AppData.Port = lora_config_application_port_get();
				
				for(uint8_t j=0;j<11;j++)
				{
					AppData.Buff[i++] =0;
				}					
			
				AppData.BuffSize = i;
				LORA_send( &AppData, LORAWAN_UNCONFIRMED_MSG);
			}
			else if(find_status==1)
			{
				AppData.Port = lora_config_application_port_get();
				
				if(LORA_send( &AppData, LORAWAN_UNCONFIRMED_MSG)==LORA_ERROR)
				{
					find_timestamp_addr=FLASH_SENSOR_DATA_START_ADDR;
				}
			}
		}
	}
}

static void OnReJoinTimerEvent( void )
{
	TimerStop( &ReJoinTimer);
	
	is_time_to_rejoin=1;
}

static void LoraStartRejoin(TxEventType_t EventType)
{
  if (EventType == TX_ON_EVENT)
  {
    /* send everytime timer elapses */
    TimerInit( &ReJoinTimer, OnReJoinTimerEvent );
    TimerSetValue( &ReJoinTimer,  REJOIN_TX_DUTYCYCLE*60000); 
		TimerStart( &ReJoinTimer);
  }
}

static void OnRetrievalTimeoutEvent( void )
{
  TimerStop(&RetrievalTimer);
	
	is_time_to_send_retrieval_data=1;
}

static void StartRetrieval(TxEventType_t EventType)
{
  if (EventType == TX_ON_EVENT)
  {
    /* send everytime timer elapses */
    TimerInit( &RetrievalTimer, OnRetrievalTimeoutEvent );
    TimerSetValue( &RetrievalTimer,  retrieval_uplink_time*1000); 
		OnRetrievalTimeoutEvent();
  }
}

static void StartCalibrationUTCtime(TxEventType_t EventType)
{
  if (EventType == TX_ON_EVENT)
  {
    /* send everytime timer elapses */
    TimerInit( &CalibrationUTCTimer, OnCalibrationUTCTimeoutEvent );
    TimerSetValue( &CalibrationUTCTimer,  time_synchronization_interval*86400000); 
		OnCalibrationUTCTimeoutEvent();
  }
}

static void OnCalibrationUTCTimeoutEvent( void )
{
  TimerSetValue( &CalibrationUTCTimer,  time_synchronization_interval*86400000);
	
  TimerStart( &CalibrationUTCTimer);
	
	UTC_Request=1;
	
	UTC_Accept=0;
	
	if(time_synchronization_method==1)
	{
		#if defined( REGION_US915 )
		MibRequestConfirm_t mib;
		mib.Type=MIB_CHANNELS_DATARATE;
		LoRaMacMibGetRequestConfirm(&mib);
		if(mib.Param.ChannelsDatarate==0)
		{
			payload_oversize=1;//mac command + payload>11
		}
		#elif defined ( REGION_AU915 ) || defined ( REGION_AS923 )
		if(dwelltime==1)
		{
			MibRequestConfirm_t mib;
			mib.Type=MIB_CHANNELS_DATARATE;
			LoRaMacMibGetRequestConfirm(&mib);
			if(mib.Param.ChannelsDatarate==2)
			{
				payload_oversize=1;//mac command + payload>11
			}
		}
	  #endif
	}
}

static void Startsht20tempcomptime(TxEventType_t EventType)
{
  if (EventType == TX_ON_TIMER)
  {
    /* send everytime timer elapses */
    TimerInit( &sht20tempcompTimer, onsht20tempcompTimeoutEvent );
    TimerSetValue( &sht20tempcompTimer,  collection_interval*60000); 
		TimerStart( &sht20tempcompTimer);
  }
}

static void onsht20tempcompTimeoutEvent( void )
{
  TimerSetValue( &sht20tempcompTimer,  collection_interval*60000);

  TimerStart( &sht20tempcompTimer);
	
	is_time_to_temp_comp=1;
}

void OnDS18B20IDTimeoutEvent(void)
{
   TimerSetValue( &DS18B20IDTimer,  86400000); 
//	TimerSetValue( &DS18B20IDTimer,  60000);
	 TimerStart( &DS18B20IDTimer );
	 
   if(pid_flag==1)
	 {
		 is_time_to_send_ds18b20_id=1;		 	
	 }		 
}

static void StartDS18B20ID(TxEventType_t EventType)
{
  if (EventType == TX_ON_EVENT)
  {
    /* send everytime timer elapses */
    TimerInit( &DS18B20IDTimer, OnDS18B20IDTimeoutEvent );
    TimerSetValue( &DS18B20IDTimer,  86400000); 
//TimerSetValue( &DS18B20IDTimer,  60000);		
//		TimerStart( &DS18B20IDTimer );
		OnDS18B20IDTimeoutEvent();
  }	
}

bool comp_ds18b20_id(void)
{
	bool different_ID=0;
	
	if(((Ext&0x0F)==0x01 || (Ext&0x0F)==0x09)&&(pid_flag==1))
	{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_SET);//open
		DelayMs(10);//Required	
		DS18B20_Search_Rom();
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_RESET);//close
		
		for(int i=0;i<8;i++)
		{
			if(DS18B20_ID[0][i]!=DS18B20_ID_temp[0][i])
			{
				different_ID=1;
			}
			DS18B20_ID_temp[0][i]=DS18B20_ID[0][i];
		}
	
		if(different_ID==1)
		{
//			check_ds18b20id=0;
			return 1;
		}
	}
	
	return 0;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
