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
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

#define DEBUG DEBUG_PRINT //DEBUG_NONE print
#include "net/ip/uip-debug.h"


/* Private variables ---------------------------------------------------------*/
extern UART_HandleTypeDef huart4;   

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/


volatile uint8_t UART4_RxBuffer[UART_RxBufferSize];

const uint8_t DISCONNECT_AP[]={"AT+CWQAP\\r\\n"};
const uint8_t CONNECT_AP[]={"\\well\\,\\26426588\\\\n\\r"};
const uint8_t TCP_CONNECTION[]="AT+CIPSTART=\"TCP\",\"192.168.2.161\",8888\\n\\r";
/* USER CODE END PV */

/* ESP8266 module structure */
Wifi_t	Wifi;
bool Server_Parsing_Flag=false;

void ESP8266_Init(void)
{
    
     MX_UART4_UART_Init(); 
     ESP8266_APInit();
     HAL_Delay(2000);
     Wifi.Mode=_WIFI_REGISTRATION_MODE;
     //ESP8266_Write();
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
  
   huart4.pRxBuffPtr = (uint8_t*)UART4_RxBuffer;
   huart4.RxXferSize = UART_RxBufferSize;
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
   
    Wifi_Init();
    
    #endif
}

void ESP8266_SendData(uint8_t *sender_addr,const uint8_t * data)
{
    uint8_t tmp; //device id
    uint8_t humidity; //door status
    uint8_t DataLen; 
    uint8_t Content_len;  
    uint8_t buffer2[30];
    uint8_t lladdr[16];
    uint8_t buffer[250];   
    
    /* get Device ID  */
    tmp=data[0];  
      
  
    /* get lladdress */
    for(int i=0;i<16;i++)
    {
      lladdr[i]=sender_addr[i]; 
      printf("ith: %d ip:%x \r\n",i,lladdr[i]);
    }
    //lladdr[16]='\0';  
     
    /* get door status */    
    humidity=data[1];  
      
    // ESP8266_Write((const uint8_t*)"AT+GMR\r\n");
    /* creat a TCP connection */
  //  ESP8266_Write((const uint8_t*)"AT+CIPMUX=1\r\n");
  //  HAL_Delay(500); 
  //  ESP8266_Write((const uint8_t*)"AT+CWMODE=3\r\n");
  //  HAL_Delay(500); 
   // ESP8266_Write((const uint8_t*)"AT+CIPSTART=\"TCP\",\"192.168.2.148\",80\r\n");
    //  ESP8266_Write((const uint8_t*)"AT+CIPSTART=\"TCP\",\"192.168.2.53\",8080\r\n");   
     ESP8266_Write((const uint8_t*)"AT+CIPSTART=\"TCP\",\"well-electronics.asuscomm.com\",80\r\n");
    // ESP8266_Write((const uint8_t*)"AT+CIPSTART=\"TCP\",\"www.google.com\",80\r\n");
    //ESP8266_Write((const uint8_t*)"AT+CIPSTART=\"TCP\",\"36.239.116.51\",8888\r\n"); //36.239.116.51 NAS  192.168.2.148
    //well-electronics.asuscomm.com/test_server/test_send_data.php
   // ESP8266_Write((const uint8_t*)"AT+CIPSTART=\"TCP\",\"well-electronics.asuscomm.com\/test_server\/test_send_data.php\",80\r\n");
  // ESP8266_Write((const uint8_t*)"AT+CIPSTART=\"TCP\",\"well-electronics.asuscomm.com\",80\r\n");
   // http://well-electronics.asuscomm.com/test_server/test_send_data.php
    HAL_Delay(500);  
    
    /* send data */   
   // sprintf((char*)buffer,"GET \/test_server\/test_send_data.php/test_sigfox/keep_history.php\?_temperature=%d\&_humidity=%d\&_mac=%x%x%
   // sprintf((char*)buffer,"GET /test_sigfox/keep_history.php\?_temperature=32.11\&_humidity=77\&_mac=112233\r\n");    
  //  sprintf((char*)buffer,"GET well-electronics.asuscomm.com/test_server/test_send_data.php\?temp=%d\&_humidity=%d\&_mac=%x%x%x%x%x%x%x%x%x%x%x%x%x%x%x%x\r\n",tmp,humidity,
    // sprintf((char*)buffer,"GET /test_sigfox/get_data.php");
#if 0
    sprintf((char*)buffer,"temp=%d_humidity=%d_mac=%x%x%x%x%x%x%x%x%x%x%x%x%x%x%x%x HTTP/1.1\r\n",tmp,humidity,   
    lladdr[0],lladdr[1],lladdr[2],lladdr[3],lladdr[4], 
            lladdr[5],lladdr[6],lladdr[7],lladdr[8],lladdr[9],
            lladdr[10],lladdr[11],lladdr[12],lladdr[13],lladdr[14],
            lladdr[15]);  
#endif
 //   sprintf((char*)buffer,"GET /test_sigfox/get_data.php\r\n");
  //  sprintf((char*)buffer,"%s Host: 192.168.2.148\r\n",buffer);
#if 1   
    sprintf((char*)buffer,"temp=%d&_humidity=%d&_mac=%x%x%x%x%x%x%x%x%x%x%x%x%x%x%x%x \r\n",tmp,humidity,   
    lladdr[0],lladdr[1],lladdr[2],lladdr[3],lladdr[4], 
            lladdr[5],lladdr[6],lladdr[7],lladdr[8],lladdr[9],
            lladdr[10],lladdr[11],lladdr[12],lladdr[13],lladdr[14],
            lladdr[15]);
    
    Content_len=strlen((const char*)buffer);
    
    sprintf((char*)buffer,"POST");
    sprintf((char*)buffer,"%s /test_server/test_send_data.php HTTP/1.1\r\n",buffer);    

  //  sprintf((char*)buffer,"%s / HTTP/1.1\r\n",buffer); 
  //  sprintf((char*)buffer,"%sHost: 192.168.2.148:80\r\n\r\n",buffer);
    sprintf((char*)buffer,"%sAccept: /\r\n",buffer);
    sprintf((char*)buffer,"%sHost: well-electronics.asuscomm.com\r\n",buffer);
    sprintf((char*)buffer,"%sContent-Type: application/x-www-form-urlencoded\r\n",buffer);
    sprintf((char*)buffer,"%sContent-Length: %d\r\n\r\n",buffer,Content_len);
    
   
    sprintf((char*)buffer,"%stemp=%d&_humidity=%d&_mac=%x%x%x%x%x%x%x%x%x%x%x%x%x%x%x%x",buffer,tmp,humidity,   
    lladdr[0],lladdr[1],lladdr[2],lladdr[3],lladdr[4], 
            lladdr[5],lladdr[6],lladdr[7],lladdr[8],lladdr[9],
            lladdr[10],lladdr[11],lladdr[12],lladdr[13],lladdr[14],
            lladdr[15]);
    
    
       
   // sprintf((char*)buffer,"%sHost: www.google.com:80\r\n",buffer);
 //   sprintf((char*)buffer,"%sUser-Agent: Mozilla/5.0 (Windows NT 10.0; Win64; x64)",buffer);
  //  sprintf((char*)buffer,"%s AppleWebKit/537.36 (KHTML, like Gecko) Chrome/68.0.3440.106 Safari/537.36\r\n",buffer);
 //   sprintf((char*)buffer,"%sAccept: /\r\n",buffer);
 //   sprintf((char*)buffer,"%sAccept-Encoding: gzip, deflate\r\n\r\n",buffer);
   
 //   sprintf((char*)buffer,"%sAccept-Language: zh-TW,zh;q=0.9,en-US;q=0.8\r\n",buffer);
 //    sprintf((char*)buffer,"%sAccept-Language: zh-TW,zh;q=0.9,en-US;q=0.8,en;q=0.7\r\n",buffer);
#endif
 
    // sprintf((char*)buffer,"AT+PING=\"tw.yahoo.com\"\r\n");
   //  ESP8266_Write((const uint8_t*)"AT+PING=\"tw.yahoo.com\"\r\n");   
  //   HAL_Delay(1000); 
//    sprintf((char*)buffer,"%sHost: 192.168.2.53\r\n",buffer);//well-electronics.asuscomm.com  36.239.116.51
//    sprintf((char*)buffer,"%s User-Agent: ESP8266/1.3\r\n",buffer);
 //   sprintf((char*)buffer,"%s Connection: close\r\n\r\n",buffer);
   
#if 0  
    printf("web ip:%x%x%x%x%x%x%x%x%x%x%x%x%x%x%x%x \r\n",lladdr[15],lladdr[14],lladdr[13],lladdr[12],lladdr[11], 
           lladdr[0],lladdr[1],lladdr[2],lladdr[3],lladdr[4], 
            lladdr[5],lladdr[6],lladdr[7],lladdr[8],lladdr[9],
            lladdr[10],lladdr[11],lladdr[12],lladdr[13],lladdr[14],
            lladdr[15]);
#endif    
    
    DataLen=strlen((const char*)buffer);
    printf("web buffer:%s \n",buffer);
    sprintf((char*)buffer2,"AT+CIPSEND=%d\r\n",DataLen); 
    ESP8266_Write(buffer2);
    HAL_Delay(1000);   
  
    ESP8266_Write(buffer); 
    HAL_Delay(6000);  
  
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
  
  while (Wifi_Station_ConnectToAp("well","26426588",NULL) == false);
 
    
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
	
  
  Wifi.Mode=_WIFI_CONFIG_MODE;
  Wifi_SoftAp_Create("Well Intelligence2","111111ap",1,3,4,0);
  
  Wifi_SendStringAndWait("AT+RST\r\n",1000);
 	HAL_Delay(3000);
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
	
  Wifi_UserInit();  
  while(Wifi_TcpIp_StartUdpConnection(0,"192.168.4.2",1678,3000)==false);
  while(Wifi_TcpIp_StartUdpConnection(1,"192.168.4.3",1678,3000)==false);
  while(Wifi_TcpIp_StartUdpConnection(2,"192.168.4.4",1678,3000)==false);
  while(Wifi_TcpIp_StartUdpConnection(3,"192.168.4.5",1678,3000)==false);
  while(Wifi_TcpIp_StartUdpConnection(4,"192.168.4.6",1678,3000)==false);
 //Wifi_TcpIp_StartUdpConnection(3,"127.0.0.1",1678,3000);   
}
//#########################################################################################################
//#########################################################################################################
//#########################################################################################################
//#########################################################################################################
void	Wifi_Init(void)
{
	HAL_UART_Receive_DMA(&_WIFI_USART,(uint8_t*)&Wifi.RxBuffer,_WIFI_RX_SIZE);
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
		//Wifi_RxClear();
		if(MAC==NULL)
			sprintf((char*)Wifi.TxBuffer,"AT+CWJAP=\"%s\",\"%s\"\r\n",SSID,Pass);
    else
			sprintf((char*)Wifi.TxBuffer,"AT+CWJAP=\"%s\",\"%s\",\"%s\"\r\n",SSID,Pass,MAC);
		if(Wifi_SendString((char*)Wifi.TxBuffer)==false)
			break;
    
    //HAL_Delay(2000);	
    //if(Wifi_WaitForString(_WIFI_WAIT_TIME_LOW,&result,2,"\r\nCONNECTED\r\n","\r\nOK\r\n","\r\nERROR\r\n","\r\nFAIL\r\n")==false)
		if(Wifi_WaitForString(_WIFI_WAIT_TIME_LOW,&result,3,"OK","CONNECTED","WIFI GOT IP")==false)
		{
      printf("\r\n Can't connect to AP: %s",(char*)SSID);
       printf("\r\n %s",(char*)Wifi.RxBuffer);
      Wifi_RxClear();      
      break;
	  }
      printf("\r\n %s",(char*)Wifi.RxBuffer);    
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
		HAL_Delay(2000);
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
	do
	{
		HAL_Delay(2000);
    Wifi_RxClear();
    if(Wifi.TcpIpMultiConnection==false)
      sprintf((char*)Wifi.TxBuffer,"AT+CIPSTART=\"UDP\",\"%s\",%d,%d\r\n",RemoteIp,RemotePort,LocalPort);
    else
      sprintf((char*)Wifi.TxBuffer,"AT+CIPSTART=%d,\"UDP\",\"%s\",%d,%d,0\r\n",LinkId,RemoteIp,RemotePort,LocalPort);
		if(Wifi_SendString((char*)Wifi.TxBuffer)==false)
			break;
    //Wifi_RxClear();
    //HAL_Delay(3000);
    printf("\r\n create new connection: %d",LinkId);
		if(Wifi_WaitForString(_WIFI_WAIT_TIME_MED,&result,2,"OK","CONNECT")==false)
		{
        break;
		}//if(result == 3)
		else
    {  
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
void	Server_Reg_Parsing(void)
{
    int i;
    uint8_t result;
    uint8_t header;
    uint8_t cmd;
    uint8_t EOP;
    char ip[4];
    char *ptr=NULL;
    uint16_t DataLen;
    uint16_t total_len;
     
    uint8_t ack_pkt[4]; //ack packet for registration mode
    char buffer[120];
    char buffer2[20];
  // parsing IPD+ len
   // Wifi_RxClear(); 
    if(Wifi_WaitForString(_WIFI_WAIT_TIME_LOW,&result,2,"OK","IPD+:")==true)
    {
      ptr=strstr((const char *)Wifi.RxBuffer,"IPD+:"); 
    }
    // parsing command type
    total_len=atoi((char*)&Wifi.RxBuffer[9]); //ipd total len
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
    for(i=0;i<=4;i++)
    {
   // HAL_Delay(2000);
    sprintf((char*)buffer2,"AT+CIPSEND=%d,3\r\n",i); 
    ESP8266_Write((uint8_t*)buffer2);
    HAL_Delay(1000);
     
    ack_pkt[0]=0xa1;
    ack_pkt[1]=0x05;
    ack_pkt[2]=0xa3;    
    ack_pkt[3]=0x00; //NULL
    sprintf((char*)buffer2,"%s",ack_pkt);
    printf("\r\n%s\r\n",ack_pkt); 
    ESP8266_Write((uint8_t*)buffer2);
    HAL_Delay(1000);
    } 
    Wifi.Mode=_WIFI_REGISTRATION_MODE;    
    
}

