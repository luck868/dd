/******************************************************************************
  * @file    lora.c
  * @author  MCD Application Team
  * @version V1.1.4
  * @date    08-January-2018
  * @brief   lora API to drive the lora state Machine
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
#include "timeServer.h"
#include "LoRaMac.h"
#include "lora.h"
#include "lora-test.h"
#include "tiny_sscanf.h"
#include "flash_eraseprogram.h"
#include "version.h"

/*
product_id
1:LTC2
2:LHT65
3:LSN50
4:LDDS75
5:LSE01
6:LDDS20
7:RS485-LN
*/
uint8_t product_id=2;
uint8_t current_fre_band=0;//AS923=2
uint8_t current_firmware_ver=0;//ver=1.0 region_info

uint8_t product_id_read_in_eeprom=0;
uint8_t fre_band_read_in_eeprom=0;//AS923=1 read in eeprom
uint8_t firmware_ver_read_in_eeprom=0;//ver=1.3
static uint8_t config_count=0;
static uint8_t key_count=0;

static uint32_t s_config[32]; //store config
static uint32_t s_key[32];    //store key
uint32_t password_get;
extern uint32_t APP_TX_DUTYCYCLE;
extern uint16_t REJOIN_TX_DUTYCYCLE;
extern uint8_t response_level;

extern bool rejoin_status;
extern uint8_t currentLeapSecond;
extern uint8_t time_synchronization_method;
extern uint8_t time_synchronization_interval;

extern bool JoinReq_NbTrails_over;
extern bool unconfirmed_downlink_data_ans_status,confirmed_downlink_data_ans_status;
uint32_t Automatic_join_network[1]={0x11};
static void new_firmware_update(void);

extern uint8_t Ext;
bool FDR_status=0;
extern uint8_t dwelltime;

extern uint8_t work_mode;
extern uint16_t collection_interval;
extern short comp_temp1_sht20,comp_temp2_sht20;

extern uint16_t power_time;
extern uint8_t exitintmode;
extern uint8_t exitmode;
extern uint8_t pid_flag;
uint8_t RX2DR_setting_status;

#define HEX16(X)  X[0],X[1], X[2],X[3], X[4],X[5], X[6],X[7],X[8],X[9], X[10],X[11], X[12],X[13], X[14],X[15]
#define HEX8(X)   X[0],X[1], X[2],X[3], X[4],X[5], X[6],X[7]
 /**
   * Lora Configuration
   */
 typedef struct
 {
   LoraState_t otaa;        /*< ENABLE if over the air activation, DISABLE otherwise */
   LoraState_t duty_cycle;  /*< ENABLE if dutycyle is on, DISABLE otherwise */
   uint8_t DevEui[8];           /*< Device EUI */
	 uint32_t DevAddr;
   uint8_t AppEui[8];           /*< Application EUI */
   uint8_t AppKey[16];          /*< Application Key */
   uint8_t NwkSKey[16];         /*< Network Session Key */
   uint8_t AppSKey[16];         /*< Application Session Key */
   int16_t Rssi;                /*< Rssi of the received packet */
   uint8_t Snr;                 /*< Snr of the received packet */
   uint8_t application_port;    /*< Application port we will receive to */
   LoraConfirm_t ReqAck;      /*< ENABLE if acknowledge is requested */
   McpsConfirm_t *McpsConfirm;  /*< pointer to the confirm structure */
   int8_t TxDatarate;
 } lora_configuration_t;
 
/**
   * Lora customize Configuration
   */
typedef struct 
{
	uint32_t freq1;
	uint8_t  set8channel;
}customize_configuration_t;

static customize_configuration_t customize_config=
{
	.freq1=0,
  .set8channel=0
};

uint32_t customize_freq1_get(void)
{
	return customize_config.freq1;
}

void customize_freq1_set(uint32_t Freq)
{
	customize_config.freq1=Freq;
}

uint32_t customize_set8channel_get(void)
{
	return customize_config.set8channel;
}

void customize_set8channel_set(uint8_t Freq)
{
	customize_config.set8channel=Freq;
}

static lora_configuration_t lora_config = 
{
  .otaa = ((OVER_THE_AIR_ACTIVATION == 0) ? LORA_DISABLE : LORA_ENABLE),
#if defined( REGION_EU868 )
  .duty_cycle = LORA_ENABLE,
#else
  .duty_cycle = LORA_DISABLE,
#endif
	.DevAddr = LORAWAN_DEVICE_ADDRESS,
  .DevEui = LORAWAN_DEVICE_EUI,
  .AppEui = LORAWAN_APPLICATION_EUI,
  .AppKey = LORAWAN_APPLICATION_KEY,
  .NwkSKey = LORAWAN_NWKSKEY,
  .AppSKey = LORAWAN_APPSKEY,
  .Rssi = 0,
  .Snr = 0,
  .ReqAck = LORAWAN_UNCONFIRMED_MSG,
  .McpsConfirm = NULL,
  .TxDatarate = 0
};


/*!
 * Join requests trials duty cycle.
 */
#define OVER_THE_AIR_ACTIVATION_DUTYCYCLE           10000  // 10 [s] value in ms

#if defined( REGION_EU868 )

#include "LoRaMacTest.h"

/*!
 * LoRaWAN ETSI duty cycle control enable/disable
 *
 * \remark Please note that ETSI mandates duty cycled transmissions. Use only for test purposes
 */
#define LORAWAN_DUTYCYCLE_ON                        true

//#define USE_SEMTECH_DEFAULT_CHANNEL_LINEUP          1

//#if( USE_SEMTECH_DEFAULT_CHANNEL_LINEUP == 1 ) 

#define LC4                { 867100000, 0, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC5                { 867300000, 0, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC6                { 867500000, 0, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC7                { 867700000, 0, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC8                { 867900000, 0, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC9                { 868800000, 0, { ( ( DR_7 << 4 ) | DR_7 ) }, 2 }
#define LC10               { 868300000, 0, { ( ( DR_6 << 4 ) | DR_6 ) }, 1 }

//#endif

#endif

static MlmeReqJoin_t JoinParameters;


//static uint32_t DevAddr = LORAWAN_DEVICE_ADDRESS;

/*
 * Defines the LoRa parameters at Init
 */
static LoRaParam_t* LoRaParamInit;
static LoRaMacPrimitives_t LoRaMacPrimitives;
static LoRaMacCallback_t LoRaMacCallbacks;
static MibRequestConfirm_t mibReq;

static LoRaMainCallback_t *LoRaMainCallbacks;
/*!
 * \brief   MCPS-Confirm event function
 *
 * \param   [IN] McpsConfirm - Pointer to the confirm structure,
 *               containing confirm attributes.
 */
static void McpsConfirm( McpsConfirm_t *mcpsConfirm )
{
    if( mcpsConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
    {
        switch( mcpsConfirm->McpsRequest )
        {
            case MCPS_UNCONFIRMED:
            {
                // Check Datarate
                // Check TxPower
                break;
            }
            case MCPS_CONFIRMED:
            {
                // Check Datarate
                // Check TxPower
                // Check AckReceived
                // Check NbTrials
                break;
            }
            case MCPS_PROPRIETARY:
            {
                break;
            }
            default:
                break;
        }
    }
}

/*!
 * \brief   MCPS-Indication event function
 *
 * \param   [IN] mcpsIndication - Pointer to the indication structure,
 *               containing indication attributes.
 */
static void McpsIndication( McpsIndication_t *mcpsIndication )
{
  lora_AppData_t AppData;
  
	rejoin_status=0;

	unconfirmed_downlink_data_ans_status=0;
	confirmed_downlink_data_ans_status=0;
    if( mcpsIndication->Status != LORAMAC_EVENT_INFO_STATUS_OK )
    {
        return;
    }

    switch( mcpsIndication->McpsIndication )
    {
        case MCPS_UNCONFIRMED:
        {
            break;
        }
        case MCPS_CONFIRMED:
        {
            break;
        }
        case MCPS_PROPRIETARY:
        {
            break;
        }
        case MCPS_MULTICAST:
        {
            break;
        }
        default:
            break;
    }

    // Check Multicast
    // Check Port
    // Check Datarate
    // Check FramePending
    // Check Buffer
    // Check BufferSize
    // Check Rssi
    // Check Snr
    // Check RxSlot
    if (certif_running() == true )
    {
      certif_DownLinkIncrement( );
    }

    if( mcpsIndication->RxData == true )
    {
      switch( mcpsIndication->Port )
      {
        case CERTIF_PORT:
          certif_rx( mcpsIndication, &JoinParameters );
          break;
        default:
          
					if(mcpsIndication->McpsIndication==MCPS_UNCONFIRMED && response_level==1)
					{
						unconfirmed_downlink_data_ans_status=1;
					}
					else if(mcpsIndication->McpsIndication==MCPS_CONFIRMED && ((response_level==2) || (response_level==4 )))
					{
						confirmed_downlink_data_ans_status=1;
					}
          AppData.Port = mcpsIndication->Port;
          AppData.BuffSize = mcpsIndication->BufferSize;
          AppData.Buff = mcpsIndication->Buffer;
          lora_config.Rssi = mcpsIndication->Rssi;
          lora_config.Snr  = mcpsIndication->Snr;
          LoRaMainCallbacks->LORA_RxData( &AppData );
          break;
      }
    }
}

/*!
 * \brief   MLME-Confirm event function
 *
 * \param   [IN] MlmeConfirm - Pointer to the confirm structure,
 *               containing confirm attributes.
 */
static void MlmeConfirm( MlmeConfirm_t *mlmeConfirm )
{
    switch( mlmeConfirm->MlmeRequest )
    {
        case MLME_JOIN:
        {
            if( mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
            {
                // Status is OK, node has joined the network
              LoRaMainCallbacks->LORA_HasJoined();
            }
            else
            {
                JoinReq_NbTrails_over=1;
            }
            break;
        }
        case MLME_LINK_CHECK:
        {
            if( mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
            {
                // Check DemodMargin
                // Check NbGateways
                if (certif_running() == true )
                {
                     certif_linkCheck( mlmeConfirm);
                }
            }
            break;
        }
        default:
            break;
    }
}
/**
 *  lora Init
 */
void LORA_Init (LoRaMainCallback_t *callbacks, LoRaParam_t* LoRaParam )
{
  /* init the Tx Duty Cycle*/
  LoRaParamInit = LoRaParam;
  
  /* init the main call backs*/
  LoRaMainCallbacks = callbacks;
  
#if (STATIC_DEVICE_EUI != 1)
  LoRaMainCallbacks->BoardGetUniqueId( lora_config.DevEui );  
#endif

#if (STATIC_DEVICE_ADDRESS != 1)
  // Random seed initialization
  srand1( LoRaMainCallbacks->BoardGetRandomSeed( ) );
  // Choose a random device address
  DevAddr = randr( 0, 0x01FFFFFF );
#endif
	
	EEPROM_Read_Config();
	
	#if defined(LoRa_Sensor_Node) || defined(AT_Data_Send)
	
	#if defined(LoRa_Sensor_Node)
	PPRINTF("\n\rDragino LHT65 Device\n\r");
	#else
	PPRINTF("\n\rLoRa ST Module\n\r");
	#endif
	
	PPRINTF("Image Version: "AT_VERSION_STRING"\n\r");
	PPRINTF("LoRaWan Stack: "AT_LoRaWan_VERSION_STRING"\n\r");	
	PPRINTF("Frequency Band: ");
	region_printf();
  new_firmware_update();
	key_printf();
	PPRINTF("Enter Password to Active AT Commands\n\n\r");
  PPRINTF("\n\rUse AT+DEBUG to see more debug info\n\r");
	#endif
	
	lora_config.otaa = LORA_ENABLE;
			
  LoRaMacPrimitives.MacMcpsConfirm = McpsConfirm;
  LoRaMacPrimitives.MacMcpsIndication = McpsIndication;
  LoRaMacPrimitives.MacMlmeConfirm = MlmeConfirm;
  LoRaMacCallbacks.GetBatteryLevel = LoRaMainCallbacks->BoardGetBatteryLevel;
#if defined( REGION_AS923 )
  LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_AS923 );
#elif defined( REGION_AU915 )
  LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_AU915 );
#elif defined( REGION_CN470 )
  LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_CN470 );
#elif defined( REGION_CN779 )
  LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_CN779 );
#elif defined( REGION_EU433 )
  LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_EU433 );
 #elif defined( REGION_IN865 )
  LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_IN865 );
#elif defined( REGION_EU868 )
  LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_EU868 );
#elif defined( REGION_KR920 )
  LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_KR920 );
#elif defined( REGION_US915 )
  LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_US915 );
#elif defined( REGION_RU864 )
  LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_RU864 );
#elif defined( REGION_KZ865 )
  LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_KZ865 );
#elif defined( REGION_MA869 )
  LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_MA869 );	
#else
    #error "Please define a region in the compiler options."
#endif

#ifdef REGION_AS923_JAPAN
  #ifndef REGION_AS923
    #error "REGION_AS923_JAPAN cannot be used without REGION_AS923; Please add macro REGION_AS923 to preprocessor."
  #endif
#endif

  mibReq.Type = MIB_ADR;
  mibReq.Param.AdrEnable = LoRaParamInit->AdrEnable;
  LoRaMacMibSetRequestConfirm( &mibReq );

  mibReq.Type = MIB_PUBLIC_NETWORK;
  mibReq.Param.EnablePublicNetwork = LoRaParamInit->EnablePublicNetwork;
  LoRaMacMibSetRequestConfirm( &mibReq );
          
  mibReq.Type = MIB_DEVICE_CLASS;
  mibReq.Param.Class= CLASS_A;
  LoRaMacMibSetRequestConfirm( &mibReq );

	#if defined( REGION_EU868 )
//	if(customize_config.freq1==0)
//	{
////	#if( USE_SEMTECH_DEFAULT_CHANNEL_LINEUP == 1 )
////		LoRaMacChannelAdd( 3, ( ChannelParams_t )LC4 );
////		LoRaMacChannelAdd( 4, ( ChannelParams_t )LC5 );
////		LoRaMacChannelAdd( 5, ( ChannelParams_t )LC6 );
////		LoRaMacChannelAdd( 6, ( ChannelParams_t )LC7 );
////		LoRaMacChannelAdd( 7, ( ChannelParams_t )LC8 );
////		LoRaMacChannelAdd( 8, ( ChannelParams_t )LC9 );
////		LoRaMacChannelAdd( 9, ( ChannelParams_t )LC10 );
//  }
				
//		mibReq.Type = MIB_RX2_DEFAULT_CHANNEL;
//		mibReq.Param.Rx2DefaultChannel = ( Rx2ChannelParams_t ){ 869525000, DR_3 };
//		LoRaMacMibSetRequestConfirm( &mibReq );

//		mibReq.Type = MIB_RX2_CHANNEL;
//	  mibReq.Param.Rx2Channel = ( Rx2ChannelParams_t ){ 869525000, DR_3 };
//		LoRaMacMibSetRequestConfirm( &mibReq );
//#endif
#endif
  lora_config.TxDatarate = LoRaParamInit->TxDatarate;
	
	/*
	 *0x00,AT+FDR
	 *0x12,FDR and join network
	 *0x13,backup Ext,TDC,add new configuration and join network
	 */
	uint32_t status=*(__IO uint32_t *)DATA_EEPROM_BASE;
	
	   if(status==0x00 || status==0x12 || status==0x13)// !=0,Automatic join network
			{
				if(status==0x00 || status==0x12)//FDR
				{
					lora_config.duty_cycle = LORA_DISABLE;
					lora_config.application_port=2;
				
					#if defined( REGION_US915 )|| defined ( REGION_AU915 )
					customize_config.set8channel=0;
					#elif defined( REGION_CN470 )
					customize_config.set8channel=11;
					#else
					customize_config.set8channel=0;
					#endif
					
					password_get=123456;
					APP_TX_DUTYCYCLE=1200000;//20min
					Ext=0x01;
					FDR_status=1;
					work_mode=0;
					collection_interval=1;
					comp_temp1_sht20=-40;
					comp_temp2_sht20=125;					
			  }
				
				if(status==0x00 || status==0x12 || status==0x13)//new firmware add new config,v1.8 now
				{
					#if defined( REGION_AS923 )	|| defined( REGION_AU915 )
					dwelltime=1;
					#endif
					REJOIN_TX_DUTYCYCLE=20;//min
					response_level=0;
					currentLeapSecond=18;//As of June 2017, the GPS time is 18seconds ahead of UTC time.
					time_synchronization_interval=10;//10day
					time_synchronization_method=1;//0 via downlink,1 via devicetimereq
					pid_flag=0;
				}		

				EEPROM_Store_Config();
				EEPROM_Read_Config();
				
				if(status==0x12 || status==0x13)
				{
					FDR_status=0;
					LORA_Join();
				}
				else
					PRINTF("Please set the parameters or reset Device to apply change\n\r");
				
				  EEPROM_program(DATA_EEPROM_BASE,Automatic_join_network,1);	  				
			}
			else 
			{					
				EEPROM_Read_Config();
				if(time_synchronization_interval==0)
				{
					time_synchronization_interval=10;//10day
				}
				
				if(collection_interval==0)
				{
					collection_interval=1;
				}
				
				if(REJOIN_TX_DUTYCYCLE==0)
				{
					REJOIN_TX_DUTYCYCLE=20;
				}
				
				if(APP_TX_DUTYCYCLE>=4000)
				{
					LORA_Join();
				}
			}
}

void region_printf(void)
{
	current_firmware_ver=18;//1.8
/*just change current_firm_ver*/
#if defined( REGION_AS923 )
	#ifdef REGION_AS923_JAPAN
	PPRINTF("AS923-JP\n\r");
	current_fre_band=1;
	#else
  PPRINTF("AS923\n\r");
	current_fre_band=2;
	#endif
#elif defined( REGION_AU915 )
  PPRINTF("AU915\n\r");
	current_fre_band=3;
#elif defined( REGION_CN470 )
  PPRINTF("CN470\n\r");
	current_fre_band=4;
#elif defined( REGION_CN779 )
  PPRINTF("CN779\n\r");
	current_fre_band=5;
#elif defined( REGION_EU433 )
  PPRINTF("EU433\n\r");
	current_fre_band=6;
#elif defined( REGION_IN865 )
  PPRINTF("IN865\n\r");
	current_fre_band=7;
#elif defined( REGION_EU868 )
  PPRINTF("EU868\n\r");
	current_fre_band=8;
#elif defined( REGION_KR920 )
  PPRINTF("KR920\n\r");
	current_fre_band=9;
#elif defined( REGION_US915 )
  PPRINTF("US915\n\r");
	current_fre_band=10;
#elif defined( REGION_RU864 )
  PPRINTF("RU864\n\r");	
	current_fre_band=11;
#elif defined( REGION_KZ865 )
  PPRINTF("KZ865\n\r");	
	current_fre_band=12;
#elif defined( REGION_MA869 )
  PPRINTF("MA869\n\r");	
	current_fre_band=13;
#else
    #error "Please define a region in the compiler options."
#endif
}

void key_printf(void)
{
//  PPRINTF("If ABP enabled\n\r"); 
  PPRINTF("DevEui= %02X %02X %02X %02X %02X %02X %02X %02X\n\r", HEX8(lora_config.DevEui));
//  PPRINTF("DevAdd=  %08X\n\r", lora_config.DevAddr) ;
//  PPRINTF("NwkSKey= %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n\r", HEX16(lora_config.NwkSKey));
//  PPRINTF("AppSKey= %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n\r", HEX16(lora_config.AppSKey));
//			
//  PPRINTF("If OTAA enabled\n\r"); 
//  PPRINTF("DevEui= %02X %02X %02X %02X %02X %02X %02X %02X\n\r", HEX8(lora_config.DevEui));
//  PPRINTF("AppEui= %02X %02X %02X %02X %02X %02X %02X %02X\n\r", HEX8(lora_config.AppEui));
//  PPRINTF("AppKey= %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n\r", HEX16(lora_config.AppKey));	
}

void LORA_Join( void)
{
  if (lora_config.otaa == LORA_ENABLE)
  {
    MlmeReq_t mlmeReq;
  
    mlmeReq.Type = MLME_JOIN;
    mlmeReq.Req.Join.DevEui = lora_config.DevEui;
    mlmeReq.Req.Join.AppEui = lora_config.AppEui;
    mlmeReq.Req.Join.AppKey = lora_config.AppKey;
    mlmeReq.Req.Join.NbTrials = LoRaParamInit->NbTrials;
  
    JoinParameters = mlmeReq.Req.Join;

    LoRaMacMlmeRequest( &mlmeReq );
  }
  else
  {
    mibReq.Type = MIB_NET_ID;
    mibReq.Param.NetID = LORAWAN_NETWORK_ID;
    LoRaMacMibSetRequestConfirm( &mibReq );

    mibReq.Type = MIB_DEV_ADDR;
    mibReq.Param.DevAddr = lora_config.DevAddr;
    LoRaMacMibSetRequestConfirm( &mibReq );

    mibReq.Type = MIB_NWK_SKEY;
    mibReq.Param.NwkSKey = lora_config.NwkSKey;
    LoRaMacMibSetRequestConfirm( &mibReq );

    mibReq.Type = MIB_APP_SKEY;
    mibReq.Param.AppSKey = lora_config.AppSKey;
    LoRaMacMibSetRequestConfirm( &mibReq );

    mibReq.Type = MIB_NETWORK_JOINED;
    mibReq.Param.IsNetworkJoined = true;
    LoRaMacMibSetRequestConfirm( &mibReq );

    LoRaMainCallbacks->LORA_HasJoined();
  }
}

LoraFlagStatus LORA_JoinStatus( void)
{
  MibRequestConfirm_t mibReq;

  mibReq.Type = MIB_NETWORK_JOINED;
  
  LoRaMacMibGetRequestConfirm( &mibReq );

  if( mibReq.Param.IsNetworkJoined == true )
  {
    return LORA_SET;
  }
  else
  {
    return LORA_RESET;
  }
}

LoraErrorStatus LORA_send(lora_AppData_t* AppData, LoraConfirm_t IsTxConfirmed)
{
    McpsReq_t mcpsReq;
    LoRaMacTxInfo_t txInfo;
  
    /*if certification test are on going, application data is not sent*/
    if (certif_running() == true)
    {
      return LORA_ERROR;
    }
    
    if( LoRaMacQueryTxPossible( AppData->BuffSize, &txInfo ) != LORAMAC_STATUS_OK )
    {
        // Send empty frame in order to flush MAC commands
        mcpsReq.Type = MCPS_UNCONFIRMED;
        mcpsReq.Req.Unconfirmed.fBuffer = NULL;
        mcpsReq.Req.Unconfirmed.fBufferSize = 0;
        mcpsReq.Req.Unconfirmed.Datarate = lora_config_tx_datarate_get() ;
    }
    else
    {
        if( IsTxConfirmed == LORAWAN_UNCONFIRMED_MSG )
        {
            mcpsReq.Type = MCPS_UNCONFIRMED;
            mcpsReq.Req.Unconfirmed.fPort = AppData->Port;
            mcpsReq.Req.Unconfirmed.fBufferSize = AppData->BuffSize;
            mcpsReq.Req.Unconfirmed.fBuffer = AppData->Buff;
            mcpsReq.Req.Unconfirmed.Datarate = lora_config_tx_datarate_get() ;
        }
        else
        {
            mcpsReq.Type = MCPS_CONFIRMED;
            mcpsReq.Req.Confirmed.fPort = AppData->Port;
            mcpsReq.Req.Confirmed.fBufferSize = AppData->BuffSize;
            mcpsReq.Req.Confirmed.fBuffer = AppData->Buff;
            mcpsReq.Req.Confirmed.NbTrials = 8;
            mcpsReq.Req.Confirmed.Datarate = lora_config_tx_datarate_get() ;
        }
    }
    if( LoRaMacMcpsRequest( &mcpsReq ) == LORAMAC_STATUS_OK )
    {
        return LORA_SUCCESS;
    }
    return LORA_ERROR;
}  

LoraErrorStatus LORA_RequestClass( DeviceClass_t newClass )
{
  LoraErrorStatus Errorstatus = LORA_SUCCESS;
  MibRequestConfirm_t mibReq;
  DeviceClass_t currentClass;
  
  mibReq.Type = MIB_DEVICE_CLASS;
  LoRaMacMibGetRequestConfirm( &mibReq );
  
  currentClass = mibReq.Param.Class;
  /*attempt to swicth only if class update*/
  if (currentClass != newClass)
  {
    switch (newClass)
    {
      case CLASS_A:
      {
        if (currentClass == CLASS_A)
        {
          mibReq.Param.Class = CLASS_A;
          if( LoRaMacMibSetRequestConfirm( &mibReq ) == LORAMAC_STATUS_OK )
          {
          /*switch is instantanuous*/
            LoRaMainCallbacks->LORA_ConfirmClass(CLASS_A);
          }
          else
          {
            Errorstatus = LORA_ERROR;
          }
        }
        break;
      }
      case CLASS_C:
      {
        if (currentClass != CLASS_A)
        {
          Errorstatus = LORA_ERROR;
        }
        /*switch is instantanuous*/
        mibReq.Param.Class = CLASS_C;
        if( LoRaMacMibSetRequestConfirm( &mibReq ) == LORAMAC_STATUS_OK )
        {
          LoRaMainCallbacks->LORA_ConfirmClass(CLASS_C);
        }
        else
        {
            Errorstatus = LORA_ERROR;
        }
        break;
      }
      default:
        break;
    } 
  }
  return Errorstatus;
}

void LORA_GetCurrentClass( DeviceClass_t *currentClass )
{
  MibRequestConfirm_t mibReq;
  
  mibReq.Type = MIB_DEVICE_CLASS;
  LoRaMacMibGetRequestConfirm( &mibReq );
  
  *currentClass = mibReq.Param.Class;
}


void lora_config_otaa_set(LoraState_t otaa)
{
  lora_config.otaa = otaa;
}


LoraState_t lora_config_otaa_get(void)
{
  return lora_config.otaa;
}

void lora_config_duty_cycle_set(LoraState_t duty_cycle)
{
  lora_config.duty_cycle = duty_cycle;
  LoRaMacTestSetDutyCycleOn((duty_cycle == LORA_ENABLE) ? 1 : 0);
}

LoraState_t lora_config_duty_cycle_get(void)
{
  return lora_config.duty_cycle;
}

uint8_t *lora_config_deveui_get(void)
{
  return lora_config.DevEui;
}

void lora_config_deveui_set(uint8_t deveui[8])
{
  memcpy1(lora_config.DevEui, deveui, sizeof(lora_config.DevEui));
}

void lora_config_devaddr_set(uint32_t devaddr)
{
	lora_config.DevAddr=devaddr;
}

void lora_config_devaddr_get()
{
	PRINTF("%08X\n\r", lora_config.DevAddr);
}

void lora_config_nwkskey_set(uint8_t nwkskey[16])
{
  memcpy1(lora_config.NwkSKey, nwkskey, sizeof(lora_config.NwkSKey));
}

uint8_t *lora_config_nwkskey_get(void)
{
  return lora_config.NwkSKey;
}

void lora_config_appskey_set(uint8_t appskey[16])
{
  memcpy1(lora_config.AppSKey, appskey, sizeof(lora_config.AppSKey));
}

uint8_t *lora_config_appskey_get(void)
{
  return lora_config.AppSKey;
}

uint8_t *lora_config_appeui_get(void)
{
  return lora_config.AppEui;
}

void lora_config_appeui_set(uint8_t appeui[8])
{
  memcpy1(lora_config.AppEui, appeui, sizeof(lora_config.AppEui));
}

uint8_t *lora_config_appkey_get(void)
{
  return lora_config.AppKey;
}

void lora_config_appkey_set(uint8_t appkey[16])
{
  memcpy1(lora_config.AppKey, appkey, sizeof(lora_config.AppKey));
}

void lora_config_reqack_set(LoraConfirm_t reqack)
{
  lora_config.ReqAck = reqack;
}

LoraConfirm_t lora_config_reqack_get(void)
{
  return lora_config.ReqAck;
}

int8_t lora_config_snr_get(void)
{
  return lora_config.Snr;
}

void lora_config_application_port_set(uint8_t application_port)
{
	lora_config.application_port=application_port;
}

uint8_t lora_config_application_port_get(void )
{
	return lora_config.application_port;
}

int16_t lora_config_rssi_get(void)
{
  return lora_config.Rssi;
}

void lora_config_tx_datarate_set(int8_t TxDataRate)
{
  lora_config.TxDatarate =TxDataRate;
}

int8_t lora_config_tx_datarate_get(void )
{
  return lora_config.TxDatarate;
}

LoraState_t lora_config_isack_get(void)
{
  if (lora_config.McpsConfirm == NULL)
  {
    return LORA_DISABLE;
  }
  else
  {
    return (lora_config.McpsConfirm->AckReceived ? LORA_ENABLE : LORA_DISABLE);
  }
}

void store_data(uint8_t size,uint8_t *data1,uint32_t data2)
{
	uint32_t data_32=0;
		switch(size)
		{
			case 1:s_key[key_count++]=data2;break;
			case 8:
             data_32|=data1[0]<<24;
	           data_32|=data1[1]<<16;
	           data_32|=data1[2]<<8;
	           data_32|=data1[3];
	           s_key[key_count++]=data_32;
	           data_32=0;
	           data_32|=data1[4]<<24;
	           data_32|=data1[5]<<16;
	           data_32|=data1[6]<<8;
	           data_32|=data1[7];
	           s_key[key_count++]=data_32;
	           data_32=0;break;
			case 16:
				     data_32|=data1[0]<<24;
	           data_32|=data1[1]<<16;
	           data_32|=data1[2]<<8;
	           data_32|=data1[3];
	           s_key[key_count++]=data_32;
	           data_32=0;
	           data_32|=data1[4]<<24;
	           data_32|=data1[5]<<16;
	           data_32|=data1[6]<<8;
	           data_32|=data1[7];
	           s_key[key_count++]=data_32;
	           data_32=0;
			       data_32|=data1[8]<<24;
	           data_32|=data1[9]<<16;
	           data_32|=data1[10]<<8;
	           data_32|=data1[11];
	           s_key[key_count++]=data_32;
	           data_32=0;
	           data_32|=data1[12]<<24;
	           data_32|=data1[13]<<16;
	           data_32|=data1[14]<<8;
	           data_32|=data1[15];
	           s_key[key_count++]=data_32;
	           data_32=0;break;
		  default:break;
		}
}

void read_data(uint8_t size,uint8_t *data1,uint32_t data3,uint32_t data4,uint32_t data5,uint32_t data6)
{
	switch(size)
		{
		case 8:
			     data1[0]=(data3>>24)&0xFF;
		       data1[1]=(data3>>16)&0xFF;
		       data1[2]=(data3>>8)&0xFF;
		       data1[3]=(data3)&0xFF;
		       data1[4]=(data4>>24)&0xFF;
		       data1[5]=(data4>>16)&0xFF;
		       data1[6]=(data4>>8)&0xFF;
		       data1[7]=(data4)&0xFF;break;
		case 16:
			     data1[0]=(data3>>24)&0xFF;
		       data1[1]=(data3>>16)&0xFF;
		       data1[2]=(data3>>8)&0xFF;
		       data1[3]=(data3)&0xFF;
		       data1[4]=(data4>>24)&0xFF;
		       data1[5]=(data4>>16)&0xFF;
		       data1[6]=(data4>>8)&0xFF;
		       data1[7]=(data4)&0xFF;
		       data1[8]=(data5>>24)&0xFF;
		       data1[9]=(data5>>16)&0xFF;
		       data1[10]=(data5>>8)&0xFF;
		       data1[11]=(data5)&0xFF;
		       data1[12]=(data6>>24)&0xFF;
		       data1[13]=(data6>>16)&0xFF;
		       data1[14]=(data6>>8)&0xFF;
		       data1[15]=(data6)&0xFF;break;
		default:break;					
		}
}
void EEPROM_Store_key(void)
{
	store_data(8,lora_config.DevEui,0);
	store_data(1,0,lora_config.DevAddr);
	store_data(16,lora_config.AppKey,0);
	store_data(16,lora_config.NwkSKey,0);
	store_data(16,lora_config.AppSKey,0);
	store_data(8,lora_config.AppEui,0);
	 
	EEPROM_program(EEPROM_USER_START_ADDR_KEY,s_key,key_count);//store key
	
	key_count=0;
}

void EEPROM_Store_Config(void)
{
	uint32_t combination_data1=0,combination_data2=0;
	
	MibRequestConfirm_t mib;
  LoRaMacStatus_t status;

  mib.Type = MIB_ADR;
  status = LoRaMacMibGetRequestConfirm(&mib);
	if(status!=LORAMAC_STATUS_OK)
	{PRINTF("LORAMAC STATUS ERROR\n\r");}
	combination_data1|=mib.Param.AdrEnable<<24;
	
	mib.Type = MIB_CHANNELS_TX_POWER;
  status = LoRaMacMibGetRequestConfirm(&mib);
	if(status!=LORAMAC_STATUS_OK)
	{PRINTF("LORAMAC STATUS ERROR\n\r");}
	combination_data1|=mib.Param.ChannelsTxPower<<16;
	
  combination_data1|=lora_config.TxDatarate<<8;
	
	combination_data1|=lora_config.duty_cycle;	
	s_config[config_count++]=combination_data1;

	 mib.Type = MIB_PUBLIC_NETWORK;
  status = LoRaMacMibGetRequestConfirm(&mib);
	if(status!=LORAMAC_STATUS_OK)
	{PRINTF("LORAMAC STATUS ERROR\n\r");}
	combination_data2|=mib.Param.EnablePublicNetwork<<24;
	
	combination_data2|=lora_config.otaa<<16;
	
	mib.Type = MIB_DEVICE_CLASS;
  status = LoRaMacMibGetRequestConfirm(&mib);
	if(status!=LORAMAC_STATUS_OK)
	{PRINTF("LORAMAC STATUS ERROR\n\r");}
	combination_data2|=mib.Param.Class<<8;//0:CLASS A
	
	combination_data2|=lora_config.ReqAck;
	s_config[config_count++]=combination_data2;
	
	mib.Type = MIB_RX2_CHANNEL;
  status = LoRaMacMibGetRequestConfirm(&mib);
	if(status!=LORAMAC_STATUS_OK)
	{PRINTF("LORAMAC STATUS ERROR\n\r");}
	s_config[config_count++]=mib.Param.Rx2Channel.Frequency;
	
	if(RX2DR_setting_status==1)
	{
		RX2DR_setting_status=0;
		mib.Type = MIB_RX2_CHANNEL;
		status = LoRaMacMibGetRequestConfirm(&mib);
		if(status!=LORAMAC_STATUS_OK)
		{PRINTF("LORAMAC STATUS ERROR\n\r");}
		s_config[config_count++]=mib.Param.Rx2Channel.Datarate;
  }
	else
	{
		mib.Type = MIB_RX2_DEFAULT_CHANNEL;
		status = LoRaMacMibGetRequestConfirm(&mib);
		s_config[config_count++]=mib.Param.Rx2DefaultChannel.Datarate;
	}	
	
	mib.Type = MIB_RECEIVE_DELAY_1;
  status = LoRaMacMibGetRequestConfirm(&mib);
	if(status!=LORAMAC_STATUS_OK)
	{PRINTF("LORAMAC STATUS ERROR\n\r");}
	s_config[config_count++]=mib.Param.ReceiveDelay1;
	
	mib.Type = MIB_RECEIVE_DELAY_2;
  status = LoRaMacMibGetRequestConfirm(&mib);
	if(status!=LORAMAC_STATUS_OK)
	{PRINTF("LORAMAC STATUS ERROR\n\r");}
	s_config[config_count++]=mib.Param.ReceiveDelay2;
	
	mib.Type = MIB_JOIN_ACCEPT_DELAY_1;
  status = LoRaMacMibGetRequestConfirm(&mib);
	if(status!=LORAMAC_STATUS_OK)
	{PRINTF("LORAMAC STATUS ERROR\n\r");}
	s_config[config_count++]=mib.Param.JoinAcceptDelay1;
	
	mib.Type = MIB_JOIN_ACCEPT_DELAY_2;
  status = LoRaMacMibGetRequestConfirm(&mib);
	if(status!=LORAMAC_STATUS_OK)
	{PRINTF("LORAMAC STATUS ERROR\n\r");}
	s_config[config_count++]=mib.Param.JoinAcceptDelay2;
	
	mib.Type = MIB_NET_ID;
  status = LoRaMacMibGetRequestConfirm(&mib);
	if(status!=LORAMAC_STATUS_OK)
	{PRINTF("LORAMAC STATUS ERROR\n\r");}
	s_config[config_count++]=mib.Param.NetID;
	
	s_config[config_count++]=APP_TX_DUTYCYCLE;
	
	s_config[config_count++]=lora_config.application_port;
	
	s_config[config_count++]=password_get;
	
	s_config[config_count++]=customize_config.freq1;
	
	s_config[config_count++]=customize_config.set8channel;
	
	s_config[config_count++]=Ext;
	
	s_config[config_count++]=0;//RTP_time reserve
	
	s_config[config_count++]=work_mode;
	
	s_config[config_count++]=collection_interval;
	
	s_config[config_count++]=comp_temp1_sht20<<16|(comp_temp2_sht20&0xffff);

	s_config[config_count++]=(exitmode<<24)|(exitintmode<<16) | power_time;
	
  s_config[config_count++]=REJOIN_TX_DUTYCYCLE<<16|(response_level<<8)|dwelltime;
	
	s_config[config_count++]=pid_flag<<24|time_synchronization_interval<<16|time_synchronization_method<<8|currentLeapSecond;
	
	if(config_count<=22)
	{
	  EEPROM_program(EEPROM_USER_START_ADDR_CONFIG,s_config,config_count);//store config
	}
	config_count=0;
}
	
void EEPROM_Read_Config(void)
{
	uint32_t start_address=0,r_config[22],r_key[17];
	
	start_address=EEPROM_USER_START_ADDR_KEY;
	/* read key*/
	for(int i=0;i<17;i++)
	{
	  r_key[i]=*(__IO uint32_t *)start_address;
		start_address+=4;
	}
	
	read_data(8 ,lora_config.DevEui,r_key[0],r_key[1],0,0);
	lora_config.DevAddr=r_key[2];
	read_data(16,lora_config.AppKey,r_key[3],r_key[4],r_key[5],r_key[6]);
	read_data(16,lora_config.NwkSKey,r_key[7],r_key[8],r_key[9],r_key[10]);
	read_data(16,lora_config.AppSKey,r_key[11],r_key[12],r_key[13],r_key[14]);
	read_data(8 ,lora_config.AppEui,r_key[15],r_key[16],0,0);
	
	start_address=EEPROM_USER_START_ADDR_CONFIG;
	for(int i=0;i<22;i++)
	{
	  r_config[i]=*(__IO uint32_t *)start_address;
		start_address+=4;
	}
	
	MibRequestConfirm_t mib;
	
  mib.Type = MIB_ADR;
	mib.Param.AdrEnable=(r_config[0]>>24)&0xFF;
	LoRaMacMibSetRequestConfirm( &mib );
	
	mib.Type = MIB_CHANNELS_TX_POWER;
	mib.Param.ChannelsTxPower=(r_config[0]>>16)&0xFF;
	LoRaMacMibSetRequestConfirm( &mib );
	
	lora_config.TxDatarate=(r_config[0]>>8)&0xFF;
	
	if((r_config[0]&0xFF)==0x01)
		lora_config.duty_cycle=LORA_ENABLE;
	  else
			lora_config.duty_cycle=LORA_DISABLE;
		
	LoRaMacTestSetDutyCycleOn((lora_config.duty_cycle == LORA_ENABLE) ? 1 : 0);
	
	mib.Type = MIB_PUBLIC_NETWORK;
	mib.Param.EnablePublicNetwork=(r_config[1]>>24)&0xFF;
	LoRaMacMibSetRequestConfirm( &mib );
	
	if(((r_config[1]>>16)&0xFF)==0x01)
		lora_config.otaa=LORA_ENABLE;
	  else
			lora_config.otaa=LORA_DISABLE;
		
	mib.Type = MIB_DEVICE_CLASS;
	mib.Param.Class=(DeviceClass_t)((r_config[1]>>8)&0xFF);
	LoRaMacMibSetRequestConfirm( &mib );
		
	if((r_config[1]&0xFF)==0x01)
		lora_config.ReqAck=LORAWAN_CONFIRMED_MSG;
	  else
			lora_config.ReqAck=LORAWAN_UNCONFIRMED_MSG;
	
	mib.Type = MIB_RX2_CHANNEL;
	mib.Param.Rx2Channel.Frequency=r_config[2];
	LoRaMacMibSetRequestConfirm( &mib );
	
	mib.Type = MIB_RX2_CHANNEL;
	mib.Param.Rx2Channel.Datarate=r_config[3];
	LoRaMacMibSetRequestConfirm( &mib );
	
	mibReq.Type = MIB_RX2_DEFAULT_CHANNEL;
	mibReq.Param.Rx2DefaultChannel = ( Rx2ChannelParams_t ){ r_config[2], r_config[3] };
	LoRaMacMibSetRequestConfirm( &mibReq );
	
  mib.Type = MIB_RECEIVE_DELAY_1;
	mib.Param.ReceiveDelay1=r_config[4];
	LoRaMacMibSetRequestConfirm( &mib );
	
	mib.Type = MIB_RECEIVE_DELAY_2;
	mib.Param.ReceiveDelay2=r_config[5];
	LoRaMacMibSetRequestConfirm( &mib );
	
	mib.Type = MIB_JOIN_ACCEPT_DELAY_1;
	mib.Param.JoinAcceptDelay1=r_config[6];
	LoRaMacMibSetRequestConfirm( &mib );
	
	mib.Type = MIB_JOIN_ACCEPT_DELAY_2;
	mib.Param.JoinAcceptDelay2=r_config[7];
	LoRaMacMibSetRequestConfirm( &mib );
	
	mib.Type = MIB_NET_ID;
	mib.Param.NetID=r_config[8];
	LoRaMacMibSetRequestConfirm( &mib );
	
	APP_TX_DUTYCYCLE=r_config[9];
	
	lora_config.application_port=r_config[10];
	
	password_get=r_config[11];
	
	customize_config.freq1=r_config[12];
	
	customize_config.set8channel=r_config[13];
	
	Ext=r_config[14];
	
//	RTP_time=r_config[15];//reserve
	
	work_mode=r_config[16];
	
	collection_interval=r_config[17];
	
	comp_temp1_sht20=r_config[18]>>16;
	
	if(comp_temp1_sht20<0)
  comp_temp1_sht20=-1.0*(~comp_temp1_sht20+1);
		
	comp_temp2_sht20=r_config[18]&0xFFFF;
	if(comp_temp2_sht20<0)
	comp_temp2_sht20=-1.0*(~comp_temp2_sht20+1);
	
	power_time=(r_config[19])&0xFFFF;	
	
	exitintmode=((r_config[19])>>16)&0xFF;
	
	exitmode=((r_config[19])>>24)&0xFF;

	REJOIN_TX_DUTYCYCLE=((r_config[20])>>16)&0xFFFF;
	
	response_level=((r_config[20])>>8)&0xFF;
	
	dwelltime=r_config[20]&0xFF;//AS923,AU915
	
	currentLeapSecond=r_config[21]&0xFF;
	
	time_synchronization_method=(r_config[21]>>8)&0xFF;
	
	time_synchronization_interval=(r_config[21]>>16)&0xFF;
	
	pid_flag=(r_config[21]>>24)&0xFF;
}

static void new_firmware_update(void)
{
	uint32_t firmware_data[1],firmware_data_read;
	uint32_t status[1]={0x12};
	
	firmware_data_read=*(__IO uint32_t *)0x808005C;//EEPROM_USER_START_ADDR_CONFIG-0x04
	
	product_id_read_in_eeprom=firmware_data_read>>16 & 0xFF;
	firmware_ver_read_in_eeprom=firmware_data_read>>8 & 0xFF;
	fre_band_read_in_eeprom=firmware_data_read & 0xFF;
	
	firmware_data[0]=product_id<<16 | current_firmware_ver<<8 | current_fre_band;
	if((product_id!=product_id_read_in_eeprom) || (current_fre_band!=fre_band_read_in_eeprom))
	{
		EEPROM_program(0x808005C,firmware_data,1);
		EEPROM_program(DATA_EEPROM_BASE,status,1);
	}
	else if((current_firmware_ver!=firmware_ver_read_in_eeprom))
	{
		//add new configuration
		status[0]=0x13;
		EEPROM_program(0x808005C,firmware_data,1);
		EEPROM_program(DATA_EEPROM_BASE,status,1);
	}
}

void clr_all_sensor_data(void)
{
	uint32_t firmware_data_read=*(__IO uint32_t *)0x808005C;//EEPROM_USER_START_ADDR_CONFIG-0x04
	firmware_ver_read_in_eeprom=firmware_data_read>>8 & 0xFF;
	
	if(firmware_ver_read_in_eeprom==0)//after v1.7
	{
	  FLASH_erase_all_sensor_data_storage(FLASH_SENSOR_DATA_START_ADDR);
	}
}

uint8_t Uplink_data_adaptive_rate(lora_AppData_t* AppData)
{
	LoRaMacTxInfo_t txInfo;
	if( LoRaMacQueryTxPossible( AppData->BuffSize, &txInfo ) != LORAMAC_STATUS_OK )
	{
		return 0;
	}
	else
		return 1;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

