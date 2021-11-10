 /******************************************************************************
  * @file    npn_output.c
  * @author  MCD Application Team
  * @version V1.1.1
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
#include "ds18b20.h"
#include "delay.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#define MaxSensorNum 1  

uint8_t DS18B20_ID[1][8]; 

bool ds18b20_connect_status=0;
void DS18B20_delay(uint16_t time)
{
  uint8_t i;

  while(time)
  {    
          for (i = 0; i < 4; i++)
    {
      
    }
    time--;
  }
}

uint8_t DS18B20_Init(void)
{
  DOUT_CLK_ENABLE();
  
  DS18B20_Mode_Out_PP();
        
  DOUT_1;
        
  DS18B20_Rst();
  
  return DS18B20_Presence();
}

void DS18B20_Mode_IPU(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  DOUT_CLK_ENABLE();
  GPIO_InitStruct.Pin   = DOUT_PIN;
  GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull  = GPIO_PULLUP;
  HAL_GPIO_Init(DOUT_PORT, &GPIO_InitStruct);      
}

void DS18B20_Mode_Out_PP(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  DOUT_CLK_ENABLE();
 
  GPIO_InitStruct.Pin = DOUT_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(DOUT_PORT, &GPIO_InitStruct);          
}

void DS18B20_IoDeInit(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
  DOUT_CLK_ENABLE();

  GPIO_InitStruct.Pin = DOUT_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DOUT_PORT, &GPIO_InitStruct);
}

void DS18B20_Rst(void)
{
        DS18B20_Mode_Out_PP();
        
        DOUT_0;
  
        DS18B20_delay(750);
        
        DOUT_1;
               
        DS18B20_delay(15);
}

 uint8_t DS18B20_Presence(void)
{
        uint8_t pulse_time = 0;
        
        DS18B20_Mode_IPU();

        while( DOUT_READ() && pulse_time<100 )
        {
                pulse_time++;
                DS18B20_delay(1);
        }        

        if( pulse_time >=100 )
                return 1;
        else
                pulse_time = 0;
        
				
        while( !DOUT_READ() && pulse_time<240 )
        {
                pulse_time++;
                DS18B20_delay(1);
        }        
        if( pulse_time >=240 )
                return 1;
        else
                return 0;
}

uint8_t DS18B20_ReadBit(void)
{
        uint8_t dat;
               
        DS18B20_Mode_Out_PP();

        DOUT_0;
        DS18B20_delay(10);
        //DOUT_1;
        DS18B20_Mode_IPU();
        //Delay_us(2);
        
        if( DOUT_READ()==SET)
                dat = 1;
        else
                dat = 0;
        

        DS18B20_delay(45);
        
        return dat;
}

uint8_t DS18B20_Read2Bit(void)
{
        uint8_t dat,i;
        
	   for(i=0;i<2;i++)
	    {
				dat=dat<<1;
				
        DS18B20_Mode_Out_PP();

        DOUT_0;
        DS18B20_delay(10);
        //DOUT_1;
        DS18B20_Mode_IPU();
        //Delay_us(2);
        
        if( DOUT_READ()==SET)
                dat |= 0x01;
        
        DS18B20_delay(45);
			}
        return dat;
}

uint8_t DS18B20_ReadByte(void)
{
        uint8_t i, j, dat = 0;        
        
        for(i=0; i<8; i++) 
        {
                j = DS18B20_ReadBit();                
                dat = (dat) | (j<<i);
        }
        
        return dat;
}

void DS18B20_WriteBit(uint8_t dat)  
{  
    DS18B20_Mode_Out_PP();  
    if (dat)  
    {  
        DOUT_0; 
        DS18B20_delay(8);  
        DOUT_1;  
        DS18B20_delay(58);  
    }  
    else  
    {  
        DOUT_0; 
        DS18B20_delay(70);  
        DOUT_1;  
        DS18B20_delay(2);  
    }  
}

void DS18B20_WriteByte(uint8_t dat)
{
        uint8_t i, testb;
        DS18B20_Mode_Out_PP();
        
        for( i=0; i<8; i++ )
        {
                testb = dat&0x01;
                dat = dat>>1;                
               
                if (testb)
                {                        
                        DOUT_0;
                        /* 1us < ???? < 15us */
                        DS18B20_delay(8);
                        
                        DOUT_1;
                        DS18B20_delay(58);
                }                
                else
                {                        
                        DOUT_0;
                        /* 60us < Tx 0 < 120us */
                        DS18B20_delay(70);
                        
                        DOUT_1;                
                        /* 1us < Trec(????) < ???*/
                        DS18B20_delay(2);
                }
         }
}

void DS18B20_SkipRom( void )
{
        DS18B20_Rst();                   
        DS18B20_Presence();                 
        DS18B20_WriteByte(0XCC);       
}

float DS18B20_GetTemp_SkipRom ( void )
{
	    uint8_t j=0;
			uint8_t tpmsb, tplsb;
			short s_tem;
			float f_tem;
 
			do
			{
				j++;
				
				if(DS18B20_Init()==0)
				{	
					DS18B20_SkipRom();
					DS18B20_WriteByte(0X44); 
					
					ds18b20_connect_status=1;
					
					DelayMs(750);
					DS18B20_SkipRom ();
					DS18B20_WriteByte(0XBE);                                
					
					tplsb = DS18B20_ReadByte();                 
					tpmsb = DS18B20_ReadByte();         
					
					s_tem = tpmsb<<8;
					s_tem = s_tem | tplsb;
					
					if( s_tem < 0 )                
									f_tem = (~s_tem+1) * -0.0625;        
					else
									f_tem = s_tem * 0.0625;
					
					if(f_tem<-55 || f_tem>125)
					{
						ds18b20_connect_status=0;
						f_tem=327.67;
					}					
				}
				else
				{
					ds18b20_connect_status=0;
	//				PRINTF("ds18b20 not connected\r");
					f_tem=327.67;
					break;
				}	
		 }while(f_tem==85 && j<2);

//		 while(j<2)
//		 {		 
//			 if(DS18B20_Init()==0)
//				{	
//					DS18B20_SkipRom();
//					DS18B20_WriteByte(0X44); 
//					
//					ds18b20_connect_status=1;
//					HAL_GPIO_WritePin(LED_RGB_PORT,LED_GREEN_PIN,GPIO_PIN_SET);
//					DelayMs(750);
//					HAL_GPIO_WritePin(LED_RGB_PORT,LED_GREEN_PIN,GPIO_PIN_RESET);
//					DS18B20_SkipRom ();
//					DS18B20_WriteByte(0XBE);                                
//					
//					tplsb = DS18B20_ReadByte();                 
//					tpmsb = DS18B20_ReadByte();         
//					
//					s_tem = tpmsb<<8;
//					s_tem = s_tem | tplsb;
//					
//					if( s_tem < 0 )                
//									f_tem = (~s_tem+1) * -0.0625;        
//					else
//									f_tem = s_tem * 0.0625;
//					
//					if(f_tem<-55 || f_tem>125)
//					{
//						ds18b20_connect_status=0;
//						f_tem=327.67;
//					}
//          else if(f_tem!=85)
//					{
//						break;
//					}						
//				}
//				else
//				{
//					ds18b20_connect_status=0;
//	//				PRINTF("ds18b20 not connected\r");
//					f_tem=327.67;
//					j=2;
//				}	
//				j++;
//		 }

//      if(DS18B20_Init()==0)
//			{
//        DS18B20_SkipRom();
//        DS18B20_WriteByte(0X44); 
//				
//				ds18b20_connect_status=1;
//				DelayMs(750);
//        DS18B20_SkipRom ();
//        DS18B20_WriteByte(0XBE);                                
//        
//        tplsb = DS18B20_ReadByte();                 
//        tpmsb = DS18B20_ReadByte();         
//        
//        s_tem = tpmsb<<8;
//        s_tem = s_tem | tplsb;
//        
//        if( s_tem < 0 )                
//                f_tem = (~s_tem+1) * -0.0625;        
//        else
//                f_tem = s_tem * 0.0625;
//        
//				if(f_tem==85)
//				{
//					  if(DS18B20_Init()==0)
//						{
//							DS18B20_SkipRom();
//							DS18B20_WriteByte(0X44); 
//							
//							ds18b20_connect_status=1;
//							DelayMs(750);
//							DS18B20_SkipRom ();
//							DS18B20_WriteByte(0XBE);                                
//							
//							tplsb = DS18B20_ReadByte();                 
//							tpmsb = DS18B20_ReadByte();         
//							
//							s_tem = tpmsb<<8;
//							s_tem = s_tem | tplsb;
//							
//							if( s_tem < 0 )                
//											f_tem = (~s_tem+1) * -0.0625;        
//							else
//											f_tem = s_tem * 0.0625;
//						}
//						else
//						{
//							ds18b20_connect_status=0;
//			//				PRINTF("ds18b20 not connected\r");
//							f_tem=327.67;
//						}
//				}
//				if(f_tem<-55 || f_tem>125)
//				{
//					ds18b20_connect_status=0;
//					f_tem=327.67;
//				}		
//			}
//      else
//			{
//				ds18b20_connect_status=0;
////				PRINTF("ds18b20 not connected\r");
//				f_tem=327.67;
//			}
			
     return f_tem;			
}

float DS18B20_Get_Temp(uint8_t i)  
{   
    uint8_t j; 
    uint8_t tpmsb, tplsb;  
    short Tem;  
    float Tem1;  
	
	  DS18B20_SkipRom();
    DS18B20_WriteByte(0X4E);
	  DS18B20_WriteByte(0X4B);
	  DS18B20_WriteByte(0X46);
	  DS18B20_WriteByte(0X3F);//0X7F=12bits,0X5F=11bits,0X3F=10bits,0X1F=9bits
	
    DS18B20_SkipRom();
    DS18B20_WriteByte(0X44);
	
	  DelayMs(190);
	
    DS18B20_Rst();  
    DS18B20_Presence();  
   
    DS18B20_WriteByte(0X55);  
    for (j = 0; j < 8; j++)  
    {  
        DS18B20_WriteByte(DS18B20_ID[i][j]);  
    }  
  
    DS18B20_WriteByte(0xBE); 
    tplsb = DS18B20_ReadByte(); // LSB     
    tpmsb = DS18B20_ReadByte(); // MSB    
    if (tpmsb & 0XFC)  
    {   
        Tem = (tpmsb << 8) | tplsb;  
        Tem1 = (~Tem) + 1;  
        Tem1 *= -0.0625;  
    }  
    else  
    {  
        Tem1 = ((tpmsb << 8) | tplsb)*0.0625;  
    }  
    return Tem1;  
}

void DS18B20_Search_Rom(void)  
{  
    uint8_t k, l, chongtuwei, m, n, num;
  
    uint8_t zhan[5];  
    uint8_t ss[64];  
    uint8_t tempp;  
    l = 0;  
    num = 0;  
    do  
    {  
        DS18B20_Rst();  
			  DS18B20_Presence();
			  DS18B20_delay(480);
        DS18B20_WriteByte(0XF0);  
			
        for (m = 0; m < 8; m++)  
        {  
            uint8_t s = 0;  
            for (n = 0; n < 8; n++)  
            {  
                k = DS18B20_Read2Bit();
  
                k = k & 0x03;  
                s=s >> 1;  
                if (k == 0x01)  
                {  
                    DS18B20_WriteBit(0);  
                    ss[(m * 8 + n)] = 0;  
                }  
                else if (k == 0x02)  
                {  
                    s = s | 0x80;  
                    DS18B20_WriteBit(1);  
                    ss[(m * 8 + n)] = 1;  
                }  
                else if (k == 0x00)
                {  
                    
                    chongtuwei = m * 8 + n + 1;  
                    if (chongtuwei > zhan[l])  
                    {  
                        DS18B20_WriteBit(0);  
                        ss[(m * 8 + n)] = 0;  
                        zhan[++l] = chongtuwei;  
                    }  
                    else if (chongtuwei < zhan[l])  
                    {  
                        s = s | ((ss[(m * 8 + n)] & 0x01) << 7);  
                        DS18B20_WriteBit(ss[(m * 8 + n)]);  
                    }  
                    else if (chongtuwei == zhan[l])  
                    {  
                        s = s | 0x80;  
                        DS18B20_WriteBit(1);  
                        ss[(m * 8 + n)] = 1;  
                        l = l - 1;  
                    }  
                }  
                else  
                {  
               
                }  
            }  
            tempp = s;  
            DS18B20_ID[num][m] = tempp;   
        }  
        num = num + 1;  
    } while (zhan[l] != 0 && (num < MaxSensorNum));
}  

//uint8_t DS18B20_Num(void)
//{
//		int flag=0,flag2=0;
//		int i=0;
//		uint8_t num1=0;
//		for(i=0;i<8;i++)
//		{
//			if(DS18B20_ID[0][i]!=DS18B20_ID[2][i])
//			{
//				if(DS18B20_ID[1][7]==0x00)
//					num1=1;
//				else if(DS18B20_ID[2][7]==0x00)
//					num1=2;
//				  else
//				   num1=3;
//			}
//			else
//				flag++;			  
//		}
//		
//		for(i=0;flag==8&&i<8;i++)
//		{
//			if(DS18B20_ID[0][i]!=DS18B20_ID[1][i])
//			{
//				num1=2;
//			}
//			else 
//				flag2++;
//		}
//		
//		if(flag2==8)
//		{
//			num1=1;
//		}

//		return num1;
////    DS18B20_SensorNum = num1;		
//}

/*******END OF FILE********/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions ---------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
