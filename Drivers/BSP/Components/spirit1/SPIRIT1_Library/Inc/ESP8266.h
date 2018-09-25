/*
 ******************************************************************************
 * @file    hts221_reg.c
 * @author  MEMS Software Solution Team
 * @date    21-September-2017
 * @brief   HTS221 driver file
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
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
#ifndef __ESP8266_DRIVER__H
#define __ESP8266_DRIVER__H

#include "stm32l1xx_hal.h"
#include <stdint.h>
#include "net/ip/uip.h"
#include "hw-config.h"



/* From ESP8266 driver module */ 
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
/* From ESP8266 driver module */

#ifdef __cplusplus
  extern "C" {
#endif

#define OFFSET 0x800
#define DEST_ADDRESS (SRAM_BASE+OFFSET)


#define		_WIFI_USART									huart4

#define		_WIFI_TX_SIZE								256
#define		_WIFI_RX_SIZE								512
#define		_WIFI_RX_FOR_DATA_SIZE			1024    
#define		_WIFI_WAIT_TIME_LOW					1000
#define		_WIFI_WAIT_TIME_MED					10000
#define		_WIFI_WAIT_TIME_HIGH				25000
#define		_WIFI_WAIT_TIME_VERYHIGH		60000    
    

static void MX_UART4_UART_Init(void);
void ESP8266_Init(void);
void ESP8266_Write(const uint8_t *inst);
void ESP8266_APInit(void);
void ESP8266_SendData(uint8_t *sender_addr,const uint8_t * data);
void ESP8266_SendCommandToNode(uint8_t *Server_Command,uint8_t Command_Length);
    

//###################################################################################################
typedef	enum
{
	WifiMode_Error                          =     0,
	WifiMode_Station                        =     1,
	WifiMode_SoftAp                         =     2,
	WifiMode_StationAndSoftAp               =     3,	
}WifiMode_t;

typedef enum
{
  WifiEncryptionType_Open                 =     0,
  WifiEncryptionType_WPA_PSK              =     2,
  WifiEncryptionType_WPA2_PSK             =     3,
  WifiEncryptionType_WPA_WPA2_PSK         =     4,
}WifiEncryptionType_t;
typedef enum
{
  WifiConnectionStatus_Error              =     0, 
  WifiConnectionStatus_GotIp              =     2,
  WifiConnectionStatus_Connected          =     3,
  WifiConnectionStatus_Disconnected       =     4,
  WifiConnectionStatus_ConnectionFail     =     5,
}WifiConnectionStatus_t;

typedef struct
{
  WifiConnectionStatus_t      status;
  uint8_t                     LinkId;
  char                        Type[4];
  char                        RemoteIp[17];
  uint16_t                    RemotePort;
  uint16_t                    LocalPort;
  bool                        RunAsServer;    
}WifiConnection_t;
//###################################################################################################
typedef struct
{
	//----------------Usart	Paremeter
	uint8_t                       usartBuff;
	uint8_t                       RxBuffer[UART_RxBufferSize];
	uint8_t                       TxBuffer[UART_TxBufferSize];
	uint16_t                      RxIndex;
  uint8_t                       RxBufferForData[UART_RxBufferSize];
  uint8_t                       RxBufferForDataTmp[8];
  uint8_t                       RxIndexForDataTmp;
  uint16_t                      RxIndexForData;
  uint16_t                      RxDataLen;
  uint8_t                       RxDataConnectionNumber;
  uint32_t                      RxDataLastTime;
  bool                          RxIsData;  
  bool                          GotNewData;  
	//----------------General	Parameter			
	WifiMode_t                    Mode;
	char                          MyIP[16];	
  char                          MyGateWay[16];
	//----------------Station	Parameter
	bool                          StationDhcp;
	char                          StationIp[16];	
	//----------------SoftAp Parameter
	bool                          SoftApDhcp;
	char                          SoftApConnectedDevicesIp[6][16];	
	char                          SoftApConnectedDevicesMac[6][18];	
	//----------------TcpIp Parameter
  bool                          TcpIpMultiConnection;
  uint16_t                      TcpIpPingAnswer;
  WifiConnection_t              TcpIpConnections[5];
  //----------------
}Wifi_t;
//###################################################################################################
extern Wifi_t	Wifi;



//###################################################################################################
void	Wifi_UserInit(void);
void  Wifi_UserProcess(void);
void  Wifi_UserGetUdpData(uint8_t LinkId,uint16_t DataLen,uint8_t *Data);
void  Wifi_UserGetTcpData(uint8_t LinkId,uint16_t DataLen,uint8_t *Data);

//###################################################################################################
void	Wifi_RxCallBack(void);
//###################################################################################################
void	Wifi_Init(void);
//###################################################################################################
bool	Wifi_Restart(void);
bool	Wifi_DeepSleep(uint16_t DelayMs);
bool	Wifi_FactoryReset(void);
bool	Wifi_Update(void);
bool	Wifi_SetRfPower(uint8_t Power_0_to_82);
//###################################################################################################
bool	Wifi_SetMode(WifiMode_t	WifiMode_);
bool	Wifi_GetMode(void);
bool	Wifi_GetMyIp(void);
//###################################################################################################
bool	Wifi_Station_ConnectToAp(char *SSID,char *Pass,char *MAC);
bool	Wifi_Station_Disconnect(void);
bool	Wifi_Station_DhcpEnable(bool Enable);
bool	Wifi_Station_DhcpIsEnable(void);
bool	Wifi_Station_SetIp(char *IP,char *GateWay,char *NetMask);
//###################################################################################################
bool  Wifi_SoftAp_GetConnectedDevices(void);
bool  Wifi_SoftAp_Create(char *SSID,char *password,uint8_t channel,WifiEncryptionType_t WifiEncryptionType,uint8_t MaxConnections_1_to_4,bool HiddenSSID);
//###################################################################################################
bool  Wifi_TcpIp_GetConnectionStatus(void);
bool  Wifi_TcpIp_Ping(char *PingTo);
bool  Wifi_TcpIp_SetMultiConnection(bool EnableMultiConnections);
bool  Wifi_TcpIp_GetMultiConnection(void);
bool  Wifi_TcpIp_StartTcpConnection(uint8_t LinkId,char *RemoteIp,uint16_t RemotePort,uint16_t TimeOut_S);
bool  Wifi_TcpIp_StartUdpConnection(uint8_t LinkId,char *RemoteIp,uint16_t RemotePort,uint16_t LocalPort);
bool  Wifi_TcpIp_Close(uint8_t LinkId);
bool  Wifi_TcpIp_SetEnableTcpServer(uint16_t PortNumber);
bool  Wifi_TcpIp_SetDisableTcpServer(uint16_t PortNumber);
bool  Wifi_TcpIp_SendDataUdp(uint8_t LinkId,uint16_t dataLen,uint8_t *data);
bool  Wifi_TcpIp_SendDataTcp(uint8_t LinkId,uint16_t dataLen,uint8_t *data);
//###################################################################################################
void	Wifi_RxClear(void);
void	Wifi_TxClear(void);
//###################################################################################################
        
    
#ifdef __cplusplus
}
#endif

#endif /*__HTS221_DRIVER__H */

