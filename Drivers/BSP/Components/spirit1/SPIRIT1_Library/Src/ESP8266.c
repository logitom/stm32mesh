/*
 ******************************************************************************
 * @file    ESP8266.c
 * @author  Thomas_Chiu
 * @date    20-June-2018
 * @brief   ESP8266 wifi module driver file
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2018 Well-electronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "ESP8266.h"
#include "stm32l1xx_hal.h"
#include "net/ip/uip.h"
#include "hw-config.h"
#include "cube_hal.h"
#include <stdint.h>
#include <stdlib.h>
#include "stm32l1xx_hal_flash.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

#define DEBUG DEBUG_PRINT //DEBUG_NONE print
#include "net/ip/uip-debug.h"

/****** Json parser head files ******/
#include <ctype.h>
#include "tiny-json.h"
/****** Json parser head files ******/

#define SEND_INTERVAL		(10 * CLOCK_SECOND)

/* Private variables ---------------------------------------------------------*/
extern UART_HandleTypeDef huart4;   
extern uip_ipaddr_t server_ip;// for registration server
/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/


volatile uint8_t UART4_RxBuffer[UART_RxBufferSize];

//const uint8_t DISCONNECT_AP[]={"AT+CWQAP\\r\\n"};
//const uint8_t CONNECT_AP[]={"\\well\\,\\26426588\\\\n\\r"};
//const uint8_t TCP_CONNECTION[]="AT+CIPSTART=\"TCP\",\"192.168.2.161\",8888\\n\\r";
/* USER CODE END PV */

/* ESP8266 module structure */
Wifi_t	Wifi;

volatile Register_Svr_t Reg_Pkt={NULL};
bool new_data=false;



void ESP8266_Init(void)
{
     MX_UART4_UART_Init(); 
     new_data=false;
     //EEPROM_Reset();   
     Wifi.IsAPConnected=false;
     HAL_UART_Receive_DMA(&_WIFI_USART,(uint8_t*)&Wifi.RxBuffer,_WIFI_RX_SIZE);
     if(Wifi.Mode==_WIFI_REG_PROJECT_CODE)
     {ESP8266_APInit();}
     // HAL_Delay(2000);
    
    
}


/* USART1 init function */
static void MX_UART4_UART_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStruct;
  /* USER CODE END USART1_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_UART4_CLK_ENABLE();
  
    /**USART1 GPIO Configuration    
    PC10     ------> UART4_TX
    PC11     ------> UART4_RX 
    */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    ;//_Error_Handler(__FILE__, __LINE__);
  }
  
   //huart4.pRxBuffPtr = (uint8_t*)Wifi.RxBuffer;
   //huart4.RxXferSize = _WIFI_RX_SIZE;
   huart4.ErrorCode = HAL_UART_ERROR_NONE;
  
  
  
   //HAL_UART_Receive_DMA(&huart1,(uint8_t*)DEST_ADDRESS,5); 
}

void ESP8266_Write(const uint8_t *inst)
{
  uint16_t StrLen;
  StrLen=strlen((const char*)inst);
  //static uint8_t test[]="AT\r\n";
 // HAL_UART_Transmit_IT(&huart1,inst,StrLen);
  HAL_UART_Transmit(&huart4,(uint8_t *)inst,StrLen,200);
}  

void ESP8266_APInit(void)
{
  //  ESP8266_Write("AT+CWQAP\r\n"); //disconnect with ap
 //   HAL_Delay(1000);
 //   ESP8266_Write("AT+CWJAP=\"well\"\,\"26426588\"\r\n"); //connect to well ap
 //  ESP8266_Write("AT+CWJAP=\"CPE_CF58A0\"\,\"freewifi\"\r\n");
 //   HAL_Delay(1000); 
    #if 1
    //HAL_UART_Receive_DMA(&huart4,(uint8_t*)&Wifi.usartBuff,UART_RxBufferSize); 
    // load eeprom default value
    Wifi_Init();
   
    #endif
}

void ESP8266_SendData(uint8_t *sender_addr,const uint8_t * data)
{
   
    uint8_t DataLen; 
    uint16_t Content_len;  
    uint8_t buffer2[512];
    uint8_t lladdr[17];
    uint8_t buffer[1024];   
    uint16_t Total_Len;
    int result=0;
          
  
    /* get lladdress */
    for(int i=0;i<16;i++)
    {
      lladdr[i]=sender_addr[i]; 
      printf("ith: %d ip:%x \r\n",i,lladdr[i]);
    }
    lladdr[16]='\0';  
     
  //data[4] alarm
  sprintf((char*)buffer2,"_type=ALARM&_group_id=%d",Wifi.GroupID);  
  sprintf((char*)buffer2,"%s&_device=[{\"_address\":\"%x%x%x%x%x%x%x%x%x%x%x%x%x%x%x%x\",\"_type\":%d,\"_status\":%d,\"_battery\":%d,\"_alarm\":true}]",buffer2,sender_addr[0],sender_addr[1],sender_addr[2],sender_addr[3],sender_addr[4],sender_addr[5],sender_addr[6],sender_addr[7],sender_addr[8],sender_addr[9],sender_addr[10],sender_addr[11],sender_addr[12],sender_addr[13],sender_addr[14],sender_addr[15],data[1],data[2],data[3]);  
  Content_len=strlen((const char*)buffer2);   
    
  sprintf((char*)buffer,"POST");
  sprintf((char*)buffer,"%s /smart_security/device/feedback HTTP/1.1\r\n",buffer);    
  sprintf((char*)buffer,"%sHost: %s:7070\r\n",buffer,Reg_Pkt.Svr_URL);
  sprintf((char*)buffer,"%sContent-Length: %d\r\n",buffer,Content_len);
  sprintf((char*)buffer,"%sContent-Type: application/x-www-form-urlencoded\r\n",buffer);
  sprintf((char*)buffer,"%sAccept: /\r\n",buffer);   
  sprintf((char*)buffer,"%sConnection:close \r\n\r\n",buffer);
  sprintf((char*)buffer,"%s%s",buffer,buffer2);   
  Total_Len=strlen((const char*)buffer);   
     
  
  
    Wifi_TcpIp_Close(0);
    while(Wifi_TcpIp_StartTcpConnection(0,(char *)Reg_Pkt.Svr_URL,7070,7000)==false);
    //HAL_Delay(1000);
    sprintf((char*)buffer2,"AT+CIPSEND=0,%d\r\n",Total_Len); 
   
    ESP8266_Write(buffer2);
    HAL_Delay(1000);   
  
    ESP8266_Write(buffer); 
    HAL_Delay(1000);  
  
    printf("web buffer:%s \n",buffer);
    
    if(Wifi_WaitForString(_WIFI_WAIT_TIME_LOW,&result,1,"\"message\":1")==true)
    {
                
        printf("\r\n sensor alarm pushed\r\n");
      //  
    }
  
   //Wifi_TcpIp_Close(0);
  /* close a TCP connection */  
  //ESP8266_Write("AT+CIPCLOSE=1\n\r");
  //HAL_Delay(1000);
}



#if 0
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
 // HAL_UART_Receive_IT(huart,(uint8_t*)huart->pRxBuffPtr,1);
  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_UART_RxCpltCallback can be implemented in the user file
   */
}
#endif


//#######################################################################################
void	Wifi_UserInit(void)
{
	Wifi_SetMode(WifiMode_StationAndSoftAp);
  
  //while (Wifi_Station_ConnectToAp("well","26426588",NULL) == false);
 
    
}
//#######################################################################################
void  Wifi_UserProcess(void)
{
  static uint8_t last=0;
  if(strstr(Wifi.MyIP,"0.0.0.0")!=NULL)
  {    
    if(last!=1)
	{
		
	}
    last=1;
  }
  else
  {   
    if(last!=2)
    {
      //Wifi_TcpIp_StartTcpConnection(0,Wifi.MyGateWay,8888,10);
      //  Wifi_TcpIp_StartTcpConnection(1,"192.168.2.161",16888,10);
      //  Wifi_TcpIp_StartUdpConnection(1,"192.168.2.161",16888,1003);
    }
    last=2;
  }
}
//#######################################################################################
void  Wifi_UserGetUdpData(uint8_t LinkId,uint16_t DataLen,uint8_t *Data)
{
  Wifi_TcpIp_SendDataUdp(LinkId,2,(uint8_t*)"OK");
}
//#######################################################################################
void  Wifi_UserGetTcpData(uint8_t LinkId,uint16_t DataLen,uint8_t *Data)
{
  Wifi_TcpIp_SendDataTcp(LinkId,2,(uint8_t*)"OK");
}
//#######################################################################################
//#########################################################################################################
void	Wifi_RxClear(void)
{
	memset(Wifi.RxBuffer,0,_WIFI_RX_SIZE);
	Wifi.RxIndex=0;	
 // HAL_UART_Receive_DMA(&_WIFI_USART,(uint8_t*)&Wifi.RxBuffer,_WIFI_RX_SIZE); 
}
//#########################################################################################################
void	Wifi_TxClear(void)
{
	memset(Wifi.TxBuffer,0,UART_RxBufferSize);
}
//#########################################################################################################




//#########################################################################################################
bool	Wifi_SendRaw(uint8_t *data,uint16_t len)
{
	if(len <= _WIFI_TX_SIZE)
	{
		memcpy(Wifi.TxBuffer,data,len);
		if(HAL_UART_Transmit(&_WIFI_USART,data,len,100) == HAL_OK)
			return true;
		else
			return false;
	}
	else
		return false;
}
//#########################################################################################################
bool	Wifi_SendString(char *data)
{
	return Wifi_SendRaw((uint8_t*)data,strlen(data));
}
//#########################################################################################################
bool	Wifi_SendStringAndWait(char *data,uint16_t DelayMs)
{
	if(Wifi_SendRaw((uint8_t*)data,strlen(data))==false)
		return false;
	HAL_Delay(DelayMs);
	return true;
}
//#########################################################################################################
bool	Wifi_WaitForString(uint32_t TimeOut_ms,uint8_t *result,uint8_t CountOfParameter,...)
{
	
	if(result == NULL)
		return false;
	if(CountOfParameter == 0)
		return false;

	*result=0;

  va_list tag;
	va_start (tag,CountOfParameter);
	char *arg[CountOfParameter];
	for(uint8_t i=0; i<CountOfParameter ; i++)
		arg[i] = va_arg (tag, char *);	
  va_end (tag);
	
		
	//////////////////////////////////	
	for(uint32_t t=0 ; t<TimeOut_ms ; t+=50)
	{
		HAL_Delay(50);
		for(uint8_t	mx=0 ; mx<CountOfParameter ; mx++)
		{			
			if(strstr((char*)Wifi.RxBuffer,arg[mx])!=NULL)
			{
				*result = mx+1;
				//if(*result>=2)
        return true;
			}				
		}				
	}
	// timeout
	return false;
	
}
//#########################################################################################################
bool	Wifi_ReturnString(char *result,uint8_t WantWhichOne,char *SplitterChars)
{
	if(result == NULL) 
		return false;
	if(WantWhichOne==0)
		return false;

	char *str = (char*)Wifi.RxBuffer;
	

	str = strtok (str,SplitterChars);
	if(str == NULL)
	{
		strcpy(result,"");
		return false;
	}
	while (str != NULL)
  {
    str = strtok (NULL,SplitterChars);
		if(str != NULL)
			WantWhichOne--;
		if(WantWhichOne==0)
		{
			strcpy(result,str);
			return true;
		}
  }
	strcpy(result,"");
	return false;	
}

//#########################################################################################################
bool	Wifi_ReturnStrings(char *InputString,char *SplitterChars,uint8_t CountOfParameter,...)
{
	if(CountOfParameter == 0)
		return false;
	va_list tag;
	va_start (tag,CountOfParameter);
	char *arg[CountOfParameter];
	for(uint8_t i=0; i<CountOfParameter ; i++)
		arg[i] = va_arg (tag, char *);	
  va_end (tag);
	
	char *str;
	str = strtok (InputString,SplitterChars);
	if(str == NULL)
		return false;
	uint8_t i=0;
	while (str != NULL)
  {
    str = strtok (NULL,SplitterChars);
		if(str != NULL)
			CountOfParameter--;
		strcpy(arg[i],str);
		i++;
		if(CountOfParameter==0)
		{
			return true;
		}
  }
	return false;	
	
}
//#########################################################################################################
bool	Wifi_ReturnInteger(int32_t	*result,uint8_t WantWhichOne,char *SplitterChars)
{
	if((char*)Wifi.RxBuffer == NULL)
		return false;
	if(Wifi_ReturnString((char*)Wifi.RxBuffer,WantWhichOne,SplitterChars)==false)
		return false;
	*result = atoi((char*)Wifi.RxBuffer);
	return true;
}
//#########################################################################################################

bool	Wifi_ReturnFloat(float	*result,uint8_t WantWhichOne,char *SplitterChars)
{	
	if((char*)Wifi.RxBuffer == NULL)
		return false;
	if(Wifi_ReturnString((char*)Wifi.RxBuffer,WantWhichOne,SplitterChars)==false)
		return false;
	*result = atof((char*)Wifi.RxBuffer);
	return true;
}
//#########################################################################################################
void Wifi_RemoveChar(char *str, char garbage)
{
	char *src, *dst;
  for (src = dst = str; *src != '\0'; src++)
	{
		*dst = *src;
		if (*dst != garbage)
			dst++;
  }
  *dst = '\0';
}

//#########################################################################################################
void	Wifi_RxCallBack(void)
{
 // get data format
 //    Wifi.RxBuffer[Wifi.RxIndex];    
     
     //parsing 
  
#if 0
  //+++ at command buffer
  if(Wifi.RxIsData==false)                                              
  {
    Wifi.RxBuffer[Wifi.RxIndex] = Wifi.usartBuff;
    if(Wifi.RxIndex < _WIFI_RX_SIZE)
      Wifi.RxIndex++; 
  //--- at command buffer
  //+++  data buffer
  else                                                                  
  {
    if( HAL_GetTick()-Wifi.RxDataLastTime > 50)
      Wifi.RxIsData=false;
    //+++ Calculate Data len after +IPD
    if(Wifi.RxDataLen==0)
    {
      //+++ Calculate Data len after +IPD ++++++ Multi Connection OFF
      if (Wifi.TcpIpMultiConnection==false)
      {
        Wifi.RxBufferForDataTmp[Wifi.RxIndexForDataTmp] = Wifi.usartBuff;
        Wifi.RxIndexForDataTmp++;
        if(Wifi.RxBufferForDataTmp[Wifi.RxIndexForDataTmp-1]==':')
        {
          Wifi.RxDataConnectionNumber=0;
          Wifi.RxDataLen=atoi((char*)&Wifi.RxBufferForDataTmp[1]);
        }
      }
      //--- Calculate Data len after +IPD ++++++ Multi Connection OFF
      //+++ Calculate Data len after +IPD ++++++ Multi Connection ON
      else
      {
        Wifi.RxBufferForDataTmp[Wifi.RxIndexForDataTmp] = Wifi.usartBuff;
        Wifi.RxIndexForDataTmp++; // need to add total received data amount Thomas
        if(Wifi.RxBufferForDataTmp[2]==',')
        {
          Wifi.RxDataConnectionNumber=Wifi.RxBufferForDataTmp[1]-48;
        }
        if((Wifi.RxIndexForDataTmp>3) && (Wifi.RxBufferForDataTmp[Wifi.RxIndexForDataTmp-1]==':'))
          Wifi.RxDataLen=atoi((char*)&Wifi.RxBufferForDataTmp[3]);
      }
      //--- Calculate Data len after +IPD ++++++ Multi Connection ON
    }
    //--- Calculate Data len after +IPD
    //+++ Fill Data Buffer
    else  
    {      
      Wifi.RxBufferForData[Wifi.RxIndexForData] = Wifi.usartBuff;
      if(Wifi.RxIndexForData < _WIFI_RX_FOR_DATA_SIZE)
        Wifi.RxIndexForData++;
      if( Wifi.RxIndexForData>= Wifi.RxDataLen)
      {
        Wifi.RxIsData=false;         
        Wifi.GotNewData=true;
      }
    }
    //--- Fill Data Buffer    
  }           
  //--- data buffer
	//HAL_UART_Receive_DMA(&_WIFI_USART,(uint8_t*)&Wifi.RxBuffer,_WIFI_RX_SIZE);
  //HAL_UART_Receive_DMA(&_WIFI_USART,(uint8_t*)&Wifi.RxBuffer,1);
  //+++ check +IPD in At command buffer
  if(Wifi.RxIndex>4)
  {
    if( (Wifi.RxBuffer[Wifi.RxIndex-4]=='+') && (Wifi.RxBuffer[Wifi.RxIndex-3]=='I') && (Wifi.RxBuffer[Wifi.RxIndex-2]=='P') && (Wifi.RxBuffer[Wifi.RxIndex-1]=='D'))
    {
      memset(Wifi.RxBufferForDataTmp,0,sizeof(Wifi.RxBufferForDataTmp));
      Wifi.RxBuffer[Wifi.RxIndex-4]=0;
      Wifi.RxBuffer[Wifi.RxIndex-3]=0;
      Wifi.RxBuffer[Wifi.RxIndex-2]=0;
      Wifi.RxBuffer[Wifi.RxIndex-1]=0;
      Wifi.RxIndex-=4;
      Wifi.RxIndexForData=0;
      Wifi.RxIndexForDataTmp=0;
      Wifi.RxIsData=true;
      Wifi.RxDataLen=0;  
      Wifi.RxDataLastTime = HAL_GetTick();      
    }
  }
  //--- check +IPD in At command buffer  
#endif
}
//#########################################################################################################
//#########################################################################################################
//#########################################################################################################
//#########################################################################################################
void WifiTask(void)
{
	
  Wifi_SoftAp_Create("Well Intelligence2","111111ap",1,3,4,0);
  
  Wifi_SendStringAndWait("AT+RST\r\n",3000);
 	//HAL_Delay(5000);
  Wifi_SetRfPower(82);
 // Wifi_TcpIp_GetMultiConnection();
  Wifi_TcpIp_Close(0);
  Wifi_TcpIp_Close(1);
  Wifi_TcpIp_Close(2);
  Wifi_TcpIp_Close(3);
  Wifi_TcpIp_Close(4);
  Wifi_TcpIp_SetMultiConnection(1);
	Wifi_GetMode();
	Wifi_Station_DhcpIsEnable();
	Wifi_SetMode(WifiMode_StationAndSoftAp);
  //Wifi_UserInit();  

  while(Wifi_TcpIp_StartUdpConnection(0,"192.168.4.2",1678,3000)==false);
  while(Wifi_TcpIp_StartUdpConnection(1,"192.168.4.3",1678,3000)==false);
 // while(Wifi_TcpIp_StartUdpConnection(2,"192.168.4.4",1678,3000)==false);
 // while(Wifi_TcpIp_StartUdpConnection(3,"192.168.4.5",1678,3000)==false);
 // while(Wifi_TcpIp_StartUdpConnection(4,"192.168.4.6",1678,3000)==false);
 //Wifi_TcpIp_StartUdpConnection(3,"127.0.0.1",1678,3000);   
}
//#########################################################################################################
//#########################################################################################################
//#########################################################################################################
//#########################################################################################################
void	Wifi_Init(void)
{
	
	Wifi_RxClear();
	Wifi_TxClear();
  WifiTask();
  
}
//#########################################################################################################
//#########################################################################################################
//#########################################################################################################
//#########################################################################################################
bool	Wifi_Restart(void)
{
	
	uint8_t result;
	bool		returnVal=false;
	do
	{
		Wifi_RxClear();
		sprintf((char*)Wifi.TxBuffer,"AT+RST\r\n");
		if(Wifi_SendString((char*)Wifi.TxBuffer)==false)
			break;
		if(Wifi_WaitForString(_WIFI_WAIT_TIME_LOW,&result,2,"OK","ERROR")==false)
			break;
		if(result == 2)
			break;			
		returnVal=true;	
	}while(0);
	
	return returnVal;	
}
//#########################################################################################################
bool	Wifi_DeepSleep(uint16_t DelayMs)
{
	
	uint8_t result;
	bool		returnVal=false;
	do
	{
		Wifi_RxClear();
		sprintf((char*)Wifi.TxBuffer,"AT+GSLP=%d\r\n",DelayMs);
		if(Wifi_SendString((char*)Wifi.TxBuffer)==false)
			break;
		if(Wifi_WaitForString(_WIFI_WAIT_TIME_LOW,&result,2,"OK","ERROR")==false)
			break;
		if(result == 2)
			break;			
		returnVal=true;	
	}while(0);
	
	return returnVal;		
}
//#########################################################################################################
bool	Wifi_FactoryReset(void)
{
	
	uint8_t result;
	bool		returnVal=false;
	do
	{
		Wifi_RxClear();
		sprintf((char*)Wifi.TxBuffer,"AT+RESTORE\r\n");
		if(Wifi_SendString((char*)Wifi.TxBuffer)==false)
			break;
		if(Wifi_WaitForString(_WIFI_WAIT_TIME_LOW,&result,2,"OK","ERROR")==false)
			break;
		if(result == 2)
			break;			
		returnVal=true;	
	}while(0);

	return returnVal;		
}
//#########################################################################################################
bool	Wifi_Update(void)
{
	
	uint8_t result;
	bool		returnVal=false;
	do
	{
		Wifi_RxClear();
		sprintf((char*)Wifi.TxBuffer,"AT+CIUPDATE\r\n");
		if(Wifi_SendString((char*)Wifi.TxBuffer)==false)
			break;
		if(Wifi_WaitForString(1000*60*5,&result,2,"OK","ERROR")==false)
			break;
		if(result == 2)
			break;			
		returnVal=true;	
	}while(0);
	
	return returnVal;			
}
//#########################################################################################################
bool	Wifi_SetRfPower(uint8_t Power_0_to_82)
{
	
	uint8_t result;
	bool		returnVal=false;
	do
	{
		HAL_Delay(2000);
    Wifi_RxClear();
		sprintf((char*)Wifi.TxBuffer,"AT+RFPOWER=%d\r\n",Power_0_to_82);
		if(Wifi_SendString((char*)Wifi.TxBuffer)==false)
			break;
		if(Wifi_WaitForString(_WIFI_WAIT_TIME_LOW,&result,2,"OK","ERROR")==false)
			break;
		if(result == 2)
			break;			
		returnVal=true;	
	}while(0);
	
	return returnVal;		
}
//#########################################################################################################
//#########################################################################################################
//#########################################################################################################
//#########################################################################################################
bool	Wifi_SetMode(WifiMode_t	WifiMode_)
{
	
	uint8_t result;
	bool		returnVal=false;
	do
	{
		Wifi_RxClear();
		sprintf((char*)Wifi.TxBuffer,"AT+CWMODE_CUR=%d\r\n",WifiMode_);
		if(Wifi_SendString((char*)Wifi.TxBuffer)==false)
			break;
		if(Wifi_WaitForString(_WIFI_WAIT_TIME_LOW,&result,2,"OK","ERROR")==false)
			break;
		if(result == 2)
			break;			
		Wifi.Mode = WifiMode_;
		returnVal=true;	
	}while(0);
	
	return returnVal;		
}
//#########################################################################################################
bool	Wifi_GetMode(void)
{
	
	uint8_t result;
	bool		returnVal=false;
	do
	{
		Wifi_RxClear();
		sprintf((char*)Wifi.TxBuffer,"AT+CWMODE_CUR?\r\n");
		if(Wifi_SendString((char*)Wifi.TxBuffer)==false)
			break;
		if(Wifi_WaitForString(_WIFI_WAIT_TIME_LOW,&result,2,"OK","ERROR")==false)
			break;
		if(result == 2)
			break;			
		if(Wifi_ReturnInteger((int32_t*)&result,1,":"))
			Wifi.Mode = (WifiMode_t)result ;
		else
			Wifi.Mode = WifiMode_Error;
		returnVal=true;	
	}while(0);
	
	return returnVal;		
}
//#########################################################################################################
bool	Wifi_GetMyIp(void)
{	
	
	uint8_t result;
	bool		returnVal=false;
	do
	{
		Wifi_RxClear();
		sprintf((char*)Wifi.TxBuffer,"AT+CIFSR\r\n");
		if(Wifi_SendString((char*)Wifi.TxBuffer)==false)
			break;
		if(Wifi_WaitForString(_WIFI_WAIT_TIME_LOW,&result,2,"OK","ERROR")==false)
			break;
		if(result == 2)
			break;		
		sscanf((char*)Wifi.RxBuffer,"AT+CIFSR\r\r\n+CIFSR:APIP,\"%[^\"]",Wifi.MyIP);
    sscanf((char*)Wifi.RxBuffer,"AT+CIFSR\r\r\n+CIFSR:STAIP,\"%[^\"]",Wifi.MyIP);			
    
    
    Wifi_RxClear();
		sprintf((char*)Wifi.TxBuffer,"AT+CIPSTA?\r\n");
		if(Wifi_SendString((char*)Wifi.TxBuffer)==false)
			break;
		if(Wifi_WaitForString(_WIFI_WAIT_TIME_LOW,&result,2,"OK","ERROR")==false)
			break;
		if(result == 2)
			break;	
    
    char *str=strstr((char*)Wifi.RxBuffer,"gateway:");
    if(str==NULL)
      break;
    if(Wifi_ReturnStrings(str,"\"",1,Wifi.MyGateWay)==false)
      break;    
    
		returnVal=true;	
	}while(0);
	
	return returnVal;		
}

//#########################################################################################################
//#########################################################################################################
//#########################################################################################################
//#########################################################################################################
bool	Wifi_Station_ConnectToAp(char *SSID,char *Pass,char *MAC)
{
	
	uint8_t result=0;
	bool		returnVal=false;
	do
	{
		Wifi_RxClear();
		if(MAC==NULL)
			sprintf((char*)Wifi.TxBuffer,"AT+CWJAP=\"%s\",\"%s\"\r\n",SSID,Pass);
    else
			sprintf((char*)Wifi.TxBuffer,"AT+CWJAP=\"%s\",\"%s\",\"%s\"\r\n",SSID,Pass,MAC);
		if(Wifi_SendString((char*)Wifi.TxBuffer)==false)
			break;
    
     HAL_Delay(300);	
    //if(Wifi_WaitForString(_WIFI_WAIT_TIME_LOW,&result,2,"\r\nCONNECTED\r\n","\r\nOK\r\n","\r\nERROR\r\n","\r\nFAIL\r\n")==false)
		if(Wifi_WaitForString(_WIFI_WAIT_TIME_LOW,&result,3,"OK","CONNECTED","WIFI GOT IP")==false)
		{
      printf("\r\n Can't connect to AP: %s",(char*)SSID);
      // printf("\r\n %s",(char*)Wifi.RxBuffer);
      //Wifi_RxClear();      
      break;
	  }
   
    //printf("\r\n %s",(char*)Wifi.RxBuffer);    
    if( result > 0)
		{  	
      returnVal=true;	
      break;
    }
   
		//returnVal=result;	
	}while(0);
	
	return returnVal;		
}
//#########################################################################################################
bool	Wifi_Station_Disconnect(void)
{
	
	uint8_t result;
	bool		returnVal=false;
	do
	{
		Wifi_RxClear();
		sprintf((char*)Wifi.TxBuffer,"AT+CWQAP\r\n");
		if(Wifi_SendString((char*)Wifi.TxBuffer)==false)
			break;
		if(Wifi_WaitForString(_WIFI_WAIT_TIME_LOW,&result,2,"OK","ERROR")==false)
			break;
		if(result == 2)
			break;				
		returnVal=true;	
	}while(0);
	
	return returnVal;		
}
//#########################################################################################################
bool	Wifi_Station_DhcpEnable(bool Enable)
{         
	
	uint8_t result;
	bool		returnVal=false;
	do
	{
		Wifi_RxClear();
		sprintf((char*)Wifi.TxBuffer,"AT+CWDHCP_CUR=1,%d\r\n",Enable);
		if(Wifi_SendString((char*)Wifi.TxBuffer)==false)
			break;
		if(Wifi_WaitForString(_WIFI_WAIT_TIME_LOW,&result,2,"OK","ERROR")==false)
			break;
		if(result == 2)
			break;		
		Wifi.StationDhcp=Enable;		
		returnVal=true;	
	}while(0);
	
	return returnVal;		
}
//#########################################################################################################
bool	Wifi_Station_DhcpIsEnable(void)
{
	
	uint8_t result;
	bool		returnVal=false;
	do
	{
		Wifi_RxClear();
		sprintf((char*)Wifi.TxBuffer,"AT+CWDHCP_CUR?\r\n");
		if(Wifi_SendString((char*)Wifi.TxBuffer)==false)
			break;
		if(Wifi_WaitForString(_WIFI_WAIT_TIME_LOW,&result,2,"OK","ERROR")==false)
			break;
		if(result == 2)
			break;		
		if(Wifi_ReturnInteger((int32_t*)&result,1,":")==false)
			break;
		switch(result)
		{
			case 0:
				Wifi.StationDhcp=false;
				Wifi.SoftApDhcp=false;				
			break;
			case 1:
				Wifi.StationDhcp=false;
				Wifi.SoftApDhcp=true;				
			break;
			case 2:
				Wifi.StationDhcp=true;
				Wifi.SoftApDhcp=false;				
			break;
			case 3:
				Wifi.StationDhcp=true;
				Wifi.SoftApDhcp=true;				
			break;			
		}
		returnVal=true;	
	}while(0);
	
	return returnVal;		
}
//#########################################################################################################
bool	Wifi_Station_SetIp(char *IP,char *GateWay,char *NetMask)
{
	
	uint8_t result;
	bool		returnVal=false;
	do
	{
		Wifi_RxClear();
		sprintf((char*)Wifi.TxBuffer,"AT+CIPSTA_CUR=\"%s\",\"%s\",\"%s\"\r\n",IP,GateWay,NetMask);
		if(Wifi_SendString((char*)Wifi.TxBuffer)==false)
			break;
		if(Wifi_WaitForString(_WIFI_WAIT_TIME_LOW,&result,2,"OK","ERROR")==false)
			break;
		if(result == 2)
			break;		
		Wifi.StationDhcp=false;		
		returnVal=true;	
	}while(0);
	
	return returnVal;		
}
//#########################################################################################################

//#########################################################################################################

//#########################################################################################################
//#########################################################################################################
//#########################################################################################################
//#########################################################################################################
bool  Wifi_SoftAp_GetConnectedDevices(void)
{
 
	uint8_t result;
	bool		returnVal=false;
	do
	{
		Wifi_RxClear();
		sprintf((char*)Wifi.TxBuffer,"AT+CWLIF\r\n");
		if(Wifi_SendString((char*)Wifi.TxBuffer)==false)
			break;
		if(Wifi_WaitForString(_WIFI_WAIT_TIME_MED,&result,2,"OK","ERROR")==false)
			break;
		if(result == 2)
			break;		
		Wifi_RemoveChar((char*)Wifi.RxBuffer,'\r');
    Wifi_ReturnStrings((char*)Wifi.RxBuffer,"\n,",10,Wifi.SoftApConnectedDevicesIp[0],Wifi.SoftApConnectedDevicesMac[0],Wifi.SoftApConnectedDevicesIp[1],Wifi.SoftApConnectedDevicesMac[1],Wifi.SoftApConnectedDevicesIp[2],Wifi.SoftApConnectedDevicesMac[2],Wifi.SoftApConnectedDevicesIp[3],Wifi.SoftApConnectedDevicesMac[3],Wifi.SoftApConnectedDevicesIp[4],Wifi.SoftApConnectedDevicesMac[4]);
		for(uint8_t i=0 ; i<6 ; i++)
    {
      if( (Wifi.SoftApConnectedDevicesIp[i][0]<'0') || (Wifi.SoftApConnectedDevicesIp[i][0]>'9'))
        Wifi.SoftApConnectedDevicesIp[i][0]=0;      
      if( (Wifi.SoftApConnectedDevicesMac[i][0]<'0') || (Wifi.SoftApConnectedDevicesMac[i][0]>'9'))
        Wifi.SoftApConnectedDevicesMac[i][0]=0;      
    }
    
		returnVal=true;	
	}while(0);
	
	return returnVal;			
}
//#########################################################################################################
bool  Wifi_SoftAp_Create(char *SSID,char *password,uint8_t channel,WifiEncryptionType_t WifiEncryptionType,uint8_t MaxConnections_1_to_4,bool HiddenSSID)
{
 
	uint8_t result;
	bool		returnVal=false;
	do
	{
		HAL_Delay(2000);
    Wifi_RxClear();
		sprintf((char*)Wifi.TxBuffer,"AT+CWSAP=\"%s\",\"%s\",%d,%d,%d,%d\r\n",SSID,password,channel,WifiEncryptionType,MaxConnections_1_to_4,HiddenSSID);
		if(Wifi_SendString((char*)Wifi.TxBuffer)==false)
			break;
		if(Wifi_WaitForString(_WIFI_WAIT_TIME_LOW,&result,2,"OK","ERROR")==false)
			break;
		if(result == 2)
			break;		  
		returnVal=true;	
	}while(0);
	
	return returnVal;		  
}
//#########################################################################################################
//#########################################################################################################
//#########################################################################################################
//#########################################################################################################
bool  Wifi_TcpIp_GetConnectionStatus(void)
{
	
	uint8_t result;
	bool		returnVal=false;
	do
	{
		Wifi_RxClear();
		sprintf((char*)Wifi.TxBuffer,"AT+CIPSTATUS\r\n");
		if(Wifi_SendString((char*)Wifi.TxBuffer)==false)
			break;
		if(Wifi_WaitForString(_WIFI_WAIT_TIME_LOW,&result,2,"OK","ERROR")==false)
			break;
		if(result == 2)
			break;		
		
    
		char *str = strstr((char*)Wifi.RxBuffer,"\nSTATUS:");
    if(str==NULL)
      break;
    str = strchr(str,':');
    str++;
    for(uint8_t i=0 ; i<5 ;i++)
      Wifi.TcpIpConnections[i].status=(WifiConnectionStatus_t)atoi(str);
    str = strstr((char*)Wifi.RxBuffer,"+CIPSTATUS:");
    for(uint8_t i=0 ; i<5 ;i++)
    {
      sscanf(str,"+CIPSTATUS:%d,\"%3s\",\"%[^\"]\",%d,%d,%d",(int*)&Wifi.TcpIpConnections[i].LinkId,Wifi.TcpIpConnections[i].Type,Wifi.TcpIpConnections[i].RemoteIp,(int*)&Wifi.TcpIpConnections[i].RemotePort,(int*)&Wifi.TcpIpConnections[i].LocalPort,(int*)&Wifi.TcpIpConnections[i].RunAsServer);
      str++;
      str = strstr(str,"+CIPSTATUS:");
      if(str==NULL)
        break;
    }
		returnVal=true;	
	}while(0);
	
	return returnVal;			
}
//#########################################################################################################
bool  Wifi_TcpIp_Ping(char *PingTo)
{
 
	uint8_t result;
	bool		returnVal=false;
	do
	{
		Wifi_RxClear();
		sprintf((char*)Wifi.TxBuffer,"AT+PING=\"%s\"\r\n",PingTo);
		if(Wifi_SendString((char*)Wifi.TxBuffer)==false)
			break;
		if(Wifi_WaitForString(_WIFI_WAIT_TIME_MED,&result,3,"OK","ERROR")==false)
			break;
		if(result == 2)
			break;		
    if(Wifi_ReturnInteger((int32_t*)&Wifi.TcpIpPingAnswer,2,"+")==false)
      break;    
		returnVal=true;	
	}while(0);
	
	return returnVal;		
}
//#########################################################################################################
bool  Wifi_TcpIp_SetMultiConnection(bool EnableMultiConnections)
{
 
	uint8_t result;
	bool		returnVal=false;
	do
	{
		HAL_Delay(700);
    Wifi_RxClear();
		sprintf((char*)Wifi.TxBuffer,"AT+CIPMUX=%d\r\n",EnableMultiConnections);
		if(Wifi_SendString((char*)Wifi.TxBuffer)==false)
			break;
		if(Wifi_WaitForString(_WIFI_WAIT_TIME_LOW,&result,2,"OK","ERROR")==false)
			break;
		if(result == 2)
			break;				
    Wifi.TcpIpMultiConnection=EnableMultiConnections;		
		returnVal=EnableMultiConnections;	
	}while(0);
	
	return returnVal;			
}
//#########################################################################################################
bool  Wifi_TcpIp_GetMultiConnection(void)
{
  
 
	uint8_t result;
	bool		returnVal=false;
	do
	{
		HAL_Delay(2000);
    Wifi_RxClear();
		sprintf((char*)Wifi.TxBuffer,"AT+CIPMUX?\r\n");
		if(Wifi_SendString((char*)Wifi.TxBuffer)==false)
			break;
		if(Wifi_WaitForString(_WIFI_WAIT_TIME_LOW,&result,2,"OK","ERROR")==false)
			break;
		if(result == 2)
			break;				
    if(Wifi_ReturnInteger((int32_t*)&result,1,":")==false)
      break;
    Wifi.TcpIpMultiConnection=(bool)result;		
		returnVal=true;	
	}while(0);
	
	return returnVal;			
}
//#########################################################################################################
bool  Wifi_TcpIp_StartTcpConnection(uint8_t LinkId,char *RemoteIp,uint16_t RemotePort,uint16_t TimeOut)
{
 
	uint8_t result;
	bool		returnVal=false;
	do
	{
    #if 0
    Wifi_RxClear();
    sprintf((char*)Wifi.TxBuffer,"AT+CIPSERVER=1,%d\r\n",RemotePort);
		if(Wifi_SendString((char*)Wifi.TxBuffer)==false)
			break;
		if(Wifi_WaitForString(_WIFI_WAIT_TIME_LOW,&result,2,"OK","ERROR")==false)
			break;
		if(result == 2)
			break;
    #endif		
		Wifi_RxClear();
    if(Wifi.TcpIpMultiConnection==false)
      //sprintf((char*)Wifi.TxBuffer,"AT+CIPSTART=\"TCP\",\"%s\",%d,%d\r\n",RemoteIp,RemotePort,TimeOut);
      sprintf((char*)Wifi.TxBuffer,"AT+CIPSTART=\"TCP\",\"%s\",%d,%d\r\n",RemoteIp,RemotePort,TimeOut);
    else
      //sprintf((char*)Wifi.TxBuffer,"AT+CIPSTART=%d,\"TCP\",\"%s\",%d,%d\r\n",LinkId,RemoteIp,RemotePort,TimeOut);
       sprintf((char*)Wifi.TxBuffer,"AT+CIPSTART=%d,\"TCP\",\"%s\",%d,%d\r\n",LinkId,RemoteIp,RemotePort,TimeOut);
		if(Wifi_SendString((char*)Wifi.TxBuffer)==false)
			break;
		if(Wifi_WaitForString(_WIFI_WAIT_TIME_LOW,&result,3,"OK","CONNECT","ERROR")==false)
			break;
		if(result == 3)
			break;		
		returnVal=true;	
	}while(0);
	
	return returnVal;		
}
//#########################################################################################################
bool  Wifi_TcpIp_StartUdpConnection(uint8_t LinkId,char *RemoteIp,uint16_t RemotePort,uint16_t LocalPort)
{ 
 
	uint8_t result;
	bool		returnVal=false;
  
   Wifi_RxClear();
   HAL_Delay(700);
  
	do
	{
	  //if(Wifi.TcpIpMultiConnection==false)
     // sprintf((char*)Wifi.TxBuffer,"AT+CIPSTART=\"UDP\",\"%s\",%d,%d\r\n",RemoteIp,RemotePort,LocalPort);
    //else
      sprintf((char*)Wifi.TxBuffer,"AT+CIPSTART=%d,\"UDP\",\"%s\",%d,%d\r\n",LinkId,RemoteIp,RemotePort,LocalPort);
		if(Wifi_SendString((char*)Wifi.TxBuffer)==false)
		break;
    //Wifi_RxClear();
    //ESP8266_Write(Wifi.TxBuffer); 
    //printf("\r\n Txbuffer: %s",Wifi.TxBuffer);
    //HAL_Delay(1000);  
    
    //HAL_Delay(1000);
   // printf("\r\n rxbuffer: %s",Wifi.RxBuffer);
		if(Wifi_WaitForString(_WIFI_WAIT_TIME_LOW,&result,3,"OK","CONNECT","ALREADY")==false)
		{
      // sprintf((char*)Wifi.TxBuffer,"AT+CIPSTART=%d,\"UDP\",\"%s\",%d,%d\r\n",LinkId,RemoteIp,RemotePort,LocalPort);
      // Wifi_SendString((char*)Wifi.TxBuffer);      
      //HAL_UART_Receive_DMA(&_WIFI_USART,(uint8_t*)&Wifi.RxBuffer,_WIFI_RX_SIZE);
      break;
		}//if(result == 3)
		else
    {  
		    printf("\r\n ok create new connection: %d",LinkId);
        returnVal=true;	
    }
  }while(0);
	
	return returnVal;		
}
//#########################################################################################################
bool  Wifi_TcpIp_Close(uint8_t LinkId)
{
 
	uint8_t result;
	bool		returnVal=false;
	do
	{
		Wifi_RxClear();
    if(Wifi.TcpIpMultiConnection==false)
      sprintf((char*)Wifi.TxBuffer,"AT+CIPCLOSE\r\n");
    else
      sprintf((char*)Wifi.TxBuffer,"AT+CIPCLOSE=%d\r\n",LinkId);
		if(Wifi_SendString((char*)Wifi.TxBuffer)==false)
			break;
		if(Wifi_WaitForString(_WIFI_WAIT_TIME_LOW,&result,2,"OK","ERROR")==false)
			break;
		if(result == 2)
			break;		
		returnVal=true;	
	}while(0);
	
	return returnVal;		
}
//#########################################################################################################
bool  Wifi_TcpIp_SetEnableTcpServer(uint16_t PortNumber)
{
 
	uint8_t result;
	bool		returnVal=false;
	do
	{
		Wifi_RxClear();
    if(Wifi.TcpIpMultiConnection==false)
    {
      sprintf((char*)Wifi.TxBuffer,"AT+CIPMUX=1\r\n");
      if(Wifi_SendString((char*)Wifi.TxBuffer)==false)
        break;
      if(Wifi_WaitForString(_WIFI_WAIT_TIME_LOW,&result,2,"OK","ERROR")==false)
        break;    
      Wifi.TcpIpMultiConnection=true;
      Wifi_RxClear();      
    }
    else
      sprintf((char*)Wifi.TxBuffer,"AT+CIPSERVER=1,%d\r\n",PortNumber);
		if(Wifi_SendString((char*)Wifi.TxBuffer)==false)
			break;
		if(Wifi_WaitForString(_WIFI_WAIT_TIME_LOW,&result,2,"OK","ERROR")==false)
			break;
		if(result == 2)
			break;		
		returnVal=true;	
	}while(0);
	
	return returnVal;		
}
//#########################################################################################################
bool  Wifi_TcpIp_SetDisableTcpServer(uint16_t PortNumber)
{
 
	uint8_t result;
	bool		returnVal=false;
	do
	{
		Wifi_RxClear();
    sprintf((char*)Wifi.TxBuffer,"AT+CIPSERVER=0,%d\r\n",PortNumber);
		if(Wifi_SendString((char*)Wifi.TxBuffer)==false)
			break;
		if(Wifi_WaitForString(_WIFI_WAIT_TIME_LOW,&result,2,"OK","ERROR")==false)
			break;
		if(result == 2)
			break;		
		returnVal=true;	
	}while(0);
	
	return returnVal;	
}
//#########################################################################################################
bool  Wifi_TcpIp_SendDataUdp(uint8_t LinkId,uint16_t dataLen,uint8_t *data)
{
 
	uint8_t result;
	bool		returnVal=false;
	do
	{
		Wifi_RxClear();
    if(Wifi.TcpIpMultiConnection==false)
      sprintf((char*)Wifi.TxBuffer,"AT+CIPSERVER=0\r\n");
    else
      sprintf((char*)Wifi.TxBuffer,"AT+CIPSEND=%d,%d\r\n",LinkId,dataLen);
		if(Wifi_SendString((char*)Wifi.TxBuffer)==false)
			break;
		if(Wifi_WaitForString(_WIFI_WAIT_TIME_LOW,&result,2,">","ERROR")==false)
			break;
		if(result == 2)
			break;		
    Wifi_RxClear();
    Wifi_SendRaw(data,dataLen);
    if(Wifi_WaitForString(_WIFI_WAIT_TIME_LOW,&result,2,"OK","ERROR")==false)
			break;
		returnVal=true;	
	}while(0);
	
	return returnVal;	
  
}
//#########################################################################################################
bool  Wifi_TcpIp_SendDataTcp(uint8_t LinkId,uint16_t dataLen,uint8_t *data)
{
 
	uint8_t result;
	bool		returnVal=false;
	do
	{
		Wifi_RxClear();
    if(Wifi.TcpIpMultiConnection==false)
      sprintf((char*)Wifi.TxBuffer,"AT+CIPSENDBUF=%d\r\n",dataLen);
    else
      sprintf((char*)Wifi.TxBuffer,"AT+CIPSENDBUF=%d,%d\r\n",LinkId,dataLen);
		if(Wifi_SendString((char*)Wifi.TxBuffer)==false)
			break;
		if(Wifi_WaitForString(_WIFI_WAIT_TIME_LOW,&result,2,"OK","ERROR")==false)
			break;
			if(Wifi_WaitForString(_WIFI_WAIT_TIME_LOW,&result,3,">","ERROR","busy")==false)
			break;
		if(result > 1)
			break;		
    Wifi_RxClear();
    Wifi_SendRaw(data,dataLen);
    if(Wifi_WaitForString(_WIFI_WAIT_TIME_LOW,&result,2,"OK","ERROR")==false)
			break;
		returnVal=true;	
	}while(0);
	
	return returnVal;	  
}
//#########################################################################################################

//#########################################################################################################
uint8_t Reg_Server_Account(void)
{
    int i=0;
    uint8_t tmp2[6];
    uint8_t tmp1[30];
    uint8_t buff2[30];
    uint16_t total_len;
    //volatile Register_Svr_t Reg_Pkt={NULL};    
    
    //total_len=atoi((char*)&Wifi.RxBuffer[9]); //ipd total len
    Reg_Pkt.Header=Wifi.RxBuffer[REG_INDEX]; //header
    Reg_Pkt.Cmd=Wifi.RxBuffer[REG_INDEX+1];
    Reg_Pkt.AP_SSID_Len=Wifi.RxBuffer[REG_INDEX+2]^PRIVATE_KEY;
    Reg_Pkt.AP_Pwd_Len=Wifi.RxBuffer[REG_INDEX+3]^PRIVATE_KEY;
    Reg_Pkt.Svr_URL_Len=Wifi.RxBuffer[REG_INDEX+4]^PRIVATE_KEY;
    Reg_Pkt.Port_len=Wifi.RxBuffer[REG_INDEX+5]^PRIVATE_KEY;
    Reg_Pkt.Google_ID_len=Wifi.RxBuffer[REG_INDEX+6]^PRIVATE_KEY;
    Reg_Pkt.Svr_UserName_Len=Wifi.RxBuffer[REG_INDEX+7]^PRIVATE_KEY;  
    Reg_Pkt.Google_Token_Len=Wifi.RxBuffer[REG_INDEX+8]^PRIVATE_KEY; 
    
    for(i=REG_INDEX+8;i<=REG_INDEX+8+Reg_Pkt.AP_SSID_Len+Reg_Pkt.AP_Pwd_Len+Reg_Pkt.Svr_URL_Len+Reg_Pkt.Google_ID_len+Reg_Pkt.Svr_UserName_Len+Reg_Pkt.Google_Token_Len+4;i++)
    Wifi.RxBuffer[i]=Wifi.RxBuffer[i]^PRIVATE_KEY;
    
    memcpy((void *)Reg_Pkt.AP_SSID,&Wifi.RxBuffer[REG_INDEX+9],Reg_Pkt.AP_SSID_Len);
    memcpy((void *)Reg_Pkt.AP_Pwd,&Wifi.RxBuffer[REG_INDEX+9+Reg_Pkt.AP_SSID_Len],Reg_Pkt.AP_Pwd_Len);
    memcpy((void *)Reg_Pkt.Svr_URL,&Wifi.RxBuffer[REG_INDEX+9+Reg_Pkt.AP_SSID_Len+Reg_Pkt.AP_Pwd_Len],Reg_Pkt.Svr_URL_Len);
    memcpy((void *)Reg_Pkt.Port,&Wifi.RxBuffer[REG_INDEX+9+Reg_Pkt.AP_SSID_Len+Reg_Pkt.Svr_URL_Len+Reg_Pkt.AP_Pwd_Len],Reg_Pkt.Port_len);
    memcpy((void *)Reg_Pkt.Google_ID,&Wifi.RxBuffer[REG_INDEX+9+Reg_Pkt.AP_SSID_Len+Reg_Pkt.AP_Pwd_Len+Reg_Pkt.Svr_URL_Len+Reg_Pkt.Port_len],Reg_Pkt.Google_ID_len);
    memcpy((void *)Reg_Pkt.Svr_UserName,&Wifi.RxBuffer[REG_INDEX+9+Reg_Pkt.AP_SSID_Len+Reg_Pkt.AP_Pwd_Len+Reg_Pkt.Svr_URL_Len+Reg_Pkt.Port_len+Reg_Pkt.Google_ID_len],Reg_Pkt.Svr_UserName_Len); // user name
    memcpy((void *)Reg_Pkt.Google_Token,&Wifi.RxBuffer[REG_INDEX+9+Reg_Pkt.AP_SSID_Len+Reg_Pkt.AP_Pwd_Len+Reg_Pkt.Svr_URL_Len+Reg_Pkt.Port_len+Reg_Pkt.Google_ID_len+Reg_Pkt.Svr_UserName_Len],Reg_Pkt.Google_Token_Len); // token
    //Reg_Pkt.Google_ID_len=Wifi.RxBuffer[REG_INDEX+4]^PRIVATE_KEY;   
    // checksum
    Reg_Pkt.AP_SSID[Reg_Pkt.AP_SSID_Len+1]=NULL;
    Reg_Pkt.AP_Pwd[Reg_Pkt.AP_Pwd_Len+1]=NULL;
    Reg_Pkt.Svr_URL[Reg_Pkt.Svr_URL_Len+1]=NULL; 
    Reg_Pkt.Port[Reg_Pkt.Port_len+1]=NULL; 
    Reg_Pkt.Google_ID[Reg_Pkt.Google_ID_len+1]=NULL;
    Reg_Pkt.Svr_UserName[Reg_Pkt.Svr_UserName_Len+1]=NULL;
    Reg_Pkt.Google_Token[Reg_Pkt.Google_Token_Len+1]=NULL;
    

    //Registration ack packet
    tmp2[0]=0xa1;
    tmp2[1]=0x03;
    tmp2[2]=0xb0;
    tmp2[3]=tmp2[1]+tmp2[2];
    tmp2[4]=0xa3;
    tmp2[5]=0x00;    
    //for(i=0;i<4;i++)
   // {
      sprintf((char*)tmp1,"AT+CIPSEND=0,5\r\n"); 
      ESP8266_Write((uint8_t*)tmp1);
      //Wifi_TcpIp_SendDataUdp(i,6,buffer);
      HAL_Delay(1000);
      sprintf((char*)buff2,"%s",tmp2);
      ESP8266_Write((uint8_t*)buff2);
      HAL_Delay(1000);      
    //}
    
    
    // 1. connect to AP
    if(Reg_Connect_AP()!=REGISTER_SERVER_SUCCEFULL)
    {return  CONNECT_TO_AP_FAILED;}
    
    Wifi.Mode=_WIFI_SERVER_MODE;
    
    return REGISTER_SERVER_SUCCEFULL;
}

//#########################################################################################################
bool	Reg_Project_Check(void)
{
    int i;
    uint8_t result;
    uint8_t header;
    uint8_t cmd;
    uint8_t EOP;
    uint8_t ip[4];
    char *ptr=NULL;
    uint16_t DataLen;
    uint16_t total_len;
    Register_Svr_t reg_pkt;    

  
    uint8_t ack_pkt[4]; //ack packet for registration mode
    char buffer[120];
    char buffer2[20];
  // parsing IPD+ len
   // Wifi_RxClear(); 
#if 0  
  if(Wifi_WaitForString(_WIFI_WAIT_TIME_LOW,&result,2,"OK","IPD+:")==true)
    {
      ptr=strstr((const char *)Wifi.RxBuffer,"IPD+:"); 
    }
#endif    
    // parsing command type
  //  total_len=atoi((char*)&Wifi.RxBuffer[9]); //ipd total len
    header=Wifi.RxBuffer[11];    //command header
    cmd=Wifi.RxBuffer[12];
    ip[0]=Wifi.RxBuffer[13];
    ip[1]=Wifi.RxBuffer[14];
    ip[2]=Wifi.RxBuffer[15];
    ip[3]=Wifi.RxBuffer[16];
    
   EOP=Wifi.RxBuffer[19];
    printf("\r\n len,:%d, cmd: %x",total_len,cmd); 
    printf("\r\n header,:%x,EOP:%x",header,EOP);     
    printf("\r\n ip:%d.%d.%d.%d",ip[0],ip[1],ip[2],ip[3]); 
   // sprintf((char*)buffer,"AT+CIPSTART=3,\"UDP\",\"%d.%d.%d.%d\",1678,3000,0\r\n",ip[0],ip[1],ip[2],ip[3]); 
    //DataLen=strlen((const char*)buffer);
    //ESP8266_Write((uint8_t*)buffer);
   // printf("web buffer:%s \n",buffer);
   // for(i=0;i<=4;i++)
   // {
   // HAL_Delay(2000);
    sprintf((char*)buffer2,"AT+CIPSEND=%d,3\r\n",i); 
    ESP8266_Write((uint8_t*)buffer2);
    HAL_Delay(300);
     
    ack_pkt[0]=0xa1;
    ack_pkt[1]=0x05;
    ack_pkt[2]=0xa3;    
    ack_pkt[3]=0x00; //NULL
    sprintf((char*)buffer2,"%s",ack_pkt);
    //printf("\r\n%s\r\n",ack_pkt); 
    ESP8266_Write((uint8_t*)buffer2);
    HAL_Delay(500);
   // } 
   // if(Wifi_WaitForString(_WIFI_WAIT_TIME_LOW,&result,1,"OK")==true)
    //Wifi.Mode=_WIFI_REG_AP_ACCOUNT;

    return true;
}

uint8_t Reg_Server_Registration(void)
{
     int i;
     uint8_t result; 
     uint16_t a;
     char kk[20]={NULL};    
     char buffer[1024];
     char buffer2[512];
     //uint16_t server[8];
     uint16_t Content_len; 
     uint16_t Total_Len;
     
   #if 0  
     for(i = 0 ; i < sizeof(uip_ipaddr_t); i += 2)
     {
         server[i]=(server_ip.u8[i] << 8) + server_ip.u8[i + 1];
     }
   #endif  
     // send device to server HTTP Host command
   
  sprintf((char*)buffer2,"_type=SIGN_IN&_google_id=%s&_name=%s&_token=%s",Reg_Pkt.Google_ID,Reg_Pkt.Svr_UserName,Reg_Pkt.Google_Token);  
  sprintf((char*)buffer2,"%s&_device=[{\"_address\":\"%x%x%x%x%x%x%x%x%x%x%x%x%x%x%x%x\",\"_type\":2,\"_status\":3,\"_battery\":56,\"_alarm\":false}]",buffer2,server_ip.u8[0],server_ip.u8[1],server_ip.u8[2],server_ip.u8[3],server_ip.u8[4],server_ip.u8[5],server_ip.u8[6],server_ip.u8[7],server_ip.u8[8],server_ip.u8[9],server_ip.u8[10],server_ip.u8[11],server_ip.u8[12],server_ip.u8[13],server_ip.u8[14],server_ip.u8[15]);  
  Content_len=strlen((const char*)buffer2);   
  
    
  sprintf((char*)buffer,"POST");
  sprintf((char*)buffer,"%s /smart_security/device/register HTTP/1.1\r\n",buffer);    
  sprintf((char*)buffer,"%sHost: %s:7070\r\n",buffer,Reg_Pkt.Svr_URL);
  sprintf((char*)buffer,"%sContent-Length: %d\r\n",buffer,Content_len);
  sprintf((char*)buffer,"%sContent-Type: application/x-www-form-urlencoded\r\n",buffer);
  sprintf((char*)buffer,"%sAccept: /\r\n",buffer);   
  sprintf((char*)buffer,"%sConnection:close \r\n\r\n",buffer);
  
  sprintf((char*)buffer,"%s%s",buffer,buffer2);   
  Total_Len=strlen((const char*)buffer);   
  
 
 
 // Wifi_TcpIp_SendDataUdp(0,Total_Len,(uint8_t *)buffer);
    //Wifi_TcpIp_Close(0);
    //while(Wifi_TcpIp_StartTcpConnection(0,(char *)Reg_Pkt.Svr_URL,7070,7000)==false); 
  
 //    sprintf((char*)Wifi.TxBuffer,"AT+CIPSEND=%d,%d\r\n",LinkId,dataLen);
    sprintf((char*)buffer2,"AT+CIPSEND=0,%d\r\n",Total_Len); 
    ESP8266_Write(buffer2);
    HAL_Delay(1000);   
  
    ESP8266_Write(buffer); 
    HAL_Delay(1000);  
  
    //strstr();
     
    if(Wifi_WaitForString(_WIFI_WAIT_TIME_LOW,&result,1,"\"message\":1")==true)
    {
        Wifi.Mode=_WIFI_REPORT_MODE;
        // save ssid & pwd to eeprom
        // parsing group ID
        Get_Group_ID();
        Save_AP_Setting();
        Wifi.IsAPConnected=true;
        printf("post:\r\n%s",buffer);
        return REGISTER_SERVER_SUCCEFULL;
    }
    
    return REGISTER_SERVER_FAILED;
}

uint8_t Reg_Connect_AP(void)
{
     // 1. connect to AP
    int i;     
    
    Wifi_SetMode(WifiMode_StationAndSoftAp);
     
    Wifi_SendStringAndWait("AT+RST\r\n",3000);
    Wifi_TcpIp_SetMultiConnection(1);
    
    HAL_Delay(1000); 
    for(i=0;i<_WIFI_RETRY_TIMES;i++)
    {
        if(Wifi_Station_ConnectToAp((char *)Reg_Pkt.AP_SSID,(char *)Reg_Pkt.AP_Pwd,NULL)==true)
        { 
           printf("\r\n wifi connected \r\n"); //add a ap connected flag
           break;
        }else if(i==_WIFI_RETRY_TIMES-1)
        {
            return REGISTER_WIFI_FAILED;
        }
    }       
  
   // Wifi_SendStringAndWait("AT+RST\r\n",3000);
   	HAL_Delay(1000);
    
    Wifi_TcpIp_Close(0);
    //Wifi_TcpIp_Close(1);
   // HAL_Delay(1000);
    
    //if(Wifi.Mode==_WIFI_REG_PROJECT_CODE)
    while(Wifi_TcpIp_StartTcpConnection(0,(char *)Reg_Pkt.Svr_URL,7070,7000)==false);
    
    return REGISTER_SERVER_SUCCEFULL;
}


uint8_t Report_Connect_AP(void)
{
     // 1. connect to AP
    int i;     
    
    Wifi_SetMode(WifiMode_Station);
    
    Wifi_SendStringAndWait("AT+RST\r\n",3000);
    Wifi_TcpIp_SetMultiConnection(1);
    
    HAL_Delay(1000); 
    for(i=0;i<_WIFI_RETRY_TIMES;i++)
    {
        if(Wifi_Station_ConnectToAp((char *)Reg_Pkt.AP_SSID,(char *)Reg_Pkt.AP_Pwd,NULL)==true)
        { 
           printf("\r\n wifi connected \r\n"); //add a ap connected flag
           break;
        }else if(i==_WIFI_RETRY_TIMES-1)
        {
            return REGISTER_WIFI_FAILED;
        }
    }       
  
   // Wifi_SendStringAndWait("AT+RST\r\n",3000);
   //	HAL_Delay(1000);
    
    //Wifi_TcpIp_Close(0);
    Wifi_TcpIp_Close(0);
   // HAL_Delay(1000);
    
    //if(Wifi.Mode==_WIFI_REG_PROJECT_CODE)
    while(Wifi_TcpIp_StartTcpConnection(0,(char *)Reg_Pkt.Svr_URL,7070,7000)==false);
    
    return REGISTER_SERVER_SUCCEFULL;
}




HAL_StatusTypeDef writeEEPROMWord(uint32_t address, uint32_t data)
 {
    HAL_StatusTypeDef  status;
    address = address + 0x08080000;
    HAL_FLASHEx_DATAEEPROM_Unlock();  //Unprotect the EEPROM to allow writing
    //status=FLASH_DATAEEPROM_FastProgramWord(address,data);
    status = HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_FASTWORD, address, data);
    HAL_FLASHEx_DATAEEPROM_Lock();  // Reprotect the EEPROM
    return status;
}

HAL_StatusTypeDef writeEEPROMByte(uint32_t address, uint8_t data)
 {
    HAL_StatusTypeDef  status;
    address = address + 0x08080000;
    HAL_FLASHEx_DATAEEPROM_Unlock();  //Unprotect the EEPROM to allow writing
    status = HAL_FLASHEx_DATAEEPROM_Program(TYPEPROGRAMDATA_BYTE, address, data);
    HAL_FLASHEx_DATAEEPROM_Lock();  // Reprotect the EEPROM
    return status;
}


uint8_t readEEPROMByte(uint32_t address) {
    uint8_t tmp = 0;
    address = address + 0x08080000;
    tmp = *(__IO uint32_t*)address;
    
    return tmp;
}



uint32_t readEEPROMWord(uint32_t address) {
    
    int i;
    uint32_t data;
    uint8_t tmp[4] ={0,0,0,0};
    for(i=0;i<4;i++)
    {
        address = address+i+ 0x08080000;
        tmp[i] = *(__IO uint32_t*)address;
    }
    data=data|tmp[0]<<24;
    data=data|tmp[1]<<16;
    data=data|tmp[2]<<8;
    data=data|tmp[3];
    
    return data;
}



uint8_t Save_AP_Setting(void)
{
    int i;
    int index=0;
    
    // write mode        
    writeEEPROMByte(index,Wifi.Mode);
    index++;
    
      
    //ssid length
    writeEEPROMByte(index,Reg_Pkt.AP_SSID_Len);
    index++;  
  
    //ssid  
    for(i=0;i<Reg_Pkt.AP_SSID_Len;i++)
    writeEEPROMByte(index+i,Reg_Pkt.AP_SSID[i]);
   
    index=index+Reg_Pkt.AP_SSID_Len;
    
    //pwd length 
    writeEEPROMByte(index,Reg_Pkt.AP_Pwd_Len);  
    index++;  
     // pwd
    for(i=0;i<Reg_Pkt.AP_Pwd_Len;i++)
    writeEEPROMByte(index+i,Reg_Pkt.AP_Pwd[i]); 
   
    index=index+Reg_Pkt.AP_Pwd_Len;
    
    //url length
    writeEEPROMByte(index,Reg_Pkt.Svr_URL_Len);  
    index++;  
    
    // url
    for(i=0;i<Reg_Pkt.Svr_URL_Len;i++)
    writeEEPROMByte(index+i,Reg_Pkt.Svr_URL[i]);
    
     //group ID
    index=index+Reg_Pkt.Svr_URL_Len;
    writeEEPROMWord(index,Wifi.GroupID);  
  
}  

void Load_AP_Setting(void)
{
    int i;
    int index=0;       
    uint8_t groupID[4]={0,0,0,0};   
  
    // device mode
    Wifi.Mode=readEEPROMByte(index);
    index++;
  
    // ssid length  
    Reg_Pkt.AP_SSID_Len=readEEPROMByte(index);
    index++;
  
    // read AP ssid
    for(i=0;i<Reg_Pkt.AP_SSID_Len;i++)  
    Reg_Pkt.AP_SSID[i]=readEEPROMByte(index+i);
       
    index=index+Reg_Pkt.AP_SSID_Len;
  
     //pwd length
     Reg_Pkt.AP_Pwd_Len=readEEPROMByte(index);
  
     index++;
    // AP pwd
    for(i=0;i<Reg_Pkt.AP_Pwd_Len;i++)
    Reg_Pkt.AP_Pwd[i]=readEEPROMByte(index+i);
 
    index=index+Reg_Pkt.AP_Pwd_Len;
    
    // URL length
    Reg_Pkt.Svr_URL_Len=readEEPROMByte(index);
    index++;
    
    // Server URL    
    for(i=0;i<Reg_Pkt.Svr_URL_Len;i++)
    Reg_Pkt.Svr_URL[i]=readEEPROMByte(index+i);

 
    //group ID
    index=index+Reg_Pkt.Svr_URL_Len;
  
    for(i=0;i<sizeof(uint32_t);i++)
    {groupID[i]=readEEPROMByte(index+i);}  
     
    Wifi.GroupID|=groupID[3]<<24;
    Wifi.GroupID|=groupID[2]<<16;
    Wifi.GroupID|=groupID[1]<<8;
    Wifi.GroupID|=groupID[0];
}  

void EEPROM_Reset(void)
{
    
    writeEEPROMByte(0,_WIFI_REG_PROJECT_CODE);   //set mode:_WIFI_REG_PROJECT_CODE 
   
    //read  device mode
    Wifi.Mode=readEEPROMByte(0);  
  
}  

uint8_t Get_Group_ID(void)
{
    enum { MAX_FIELDS = 4 };
    json_t pool[ MAX_FIELDS ]; 
   
    
    char *group_id; 
    group_id=strstr((char*)Wifi.RxBuffer,"{\"");  
    
   
    json_t const* parent = json_create( group_id, pool, MAX_FIELDS );
    if ( parent == NULL ) return EXIT_FAILURE;


    json_t const* groupID = json_getProperty( parent, "value" );
    if ( groupID == NULL ) return EXIT_FAILURE;
    if ( json_getType( groupID ) != JSON_INTEGER ) return EXIT_FAILURE;   
 

    char const* namevalue = json_getValue( groupID );
    Wifi.GroupID= atoi((char*)namevalue);
    //printf( "%s%s%s", "Name: '", namevalue, "'.\n" ); 
    printf( "group id:%d \r\n",Wifi.GroupID);    
   // *result = atoi((char*)Wifi.RxBuffer);
    return EXIT_SUCCESS;  
  
}  

