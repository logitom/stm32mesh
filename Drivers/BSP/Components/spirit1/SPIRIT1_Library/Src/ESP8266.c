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

/* Private variables ---------------------------------------------------------*/
extern UART_HandleTypeDef huart1;   

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
volatile uint8_t UART1_RxBuffer[UART_RxBufferSize];
volatile uint8_t UART1_TxBuffer[UART_RxBufferSize];

const uint8_t DISCONNECT_AP[]={"AT+CWQAP\\r\\n"};
const uint8_t CONNECT_AP[]={"\\well\\,\\26426588\\\\n\\r"};
const uint8_t TCP_CONNECTION[]="AT+CIPSTART=\"TCP\",\"192.168.2.161\",8888\\n\\r";

/* USER CODE END PV */

void ESP8266_Init(void)
{
     MX_USART1_UART_Init(); 
     ESP8266_APInit();
    // ESP8266_Write();
}


/* USART1 init function */
static void MX_USART1_UART_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStruct;
  /* USER CODE END USART1_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();
  
    /**USART1 GPIO Configuration    
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX 
    */
   huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    ;//_Error_Handler(__FILE__, __LINE__);
  }
}

void ESP8266_Write(const uint8_t *inst)
{
  uint16_t StrLen;
  StrLen=strlen((const char*)inst);
  //static uint8_t test[]="AT\r\n";
 // HAL_UART_Transmit_IT(&huart1,inst,StrLen);
  HAL_UART_Transmit(&huart1,(uint8_t *)inst,StrLen,200);
}  

void ESP8266_APInit(void)
{
    ESP8266_Write("AT+CWQAP\r\n"); //disconnect with ap
    HAL_Delay(1000);
    ESP8266_Write("AT+CWJAP=\"well\"\,\"26426588\"\r\n"); //connect to well ap
    HAL_Delay(1000);  
}

void ESP8266_SendData(uint8_t *sender_addr,const uint8_t * data)
{
    uint8_t tmp[6];
    uint8_t humidity[6];
    uint8_t DataLen;  
    uint8_t buffer2[30];
    uint8_t lladdr[16];
    uint8_t buffer[150];   
    
    /* get temperature data */
    memcpy(tmp,&data[2],5);  
    tmp[5]='\0';  
  
    /* get lladdress */
    for(int i=0;i<16;i++)
    {
      lladdr[i]=sender_addr[i]; 
      printf("ith: %d ip:%x \r\n",i,lladdr[i]);
    }
    //lladdr[16]='\0';  
     
    /* get humidity data */    
    memcpy(humidity,&data[10],5);
    humidity[5]='\0';  
  
    /* creat a TCP connection */
    ESP8266_Write("AT+CIPSTART=\"TCP\",\"192.168.2.161\",8888\r\n");
    HAL_Delay(500);  
    
    /* send data */   
   //sprintf((char*)buffer,"GET /test_sigfox/keep_history.php\?_temperature=32.77\&_humidity=50.89\&_mac=999\r\n");
   // sprintf((char*)buffer,"GET /test_sigfox/keep_history.php\?_temperature=32.11\&_humidity=77\&_mac=112233\r\n");    
    sprintf((char*)buffer,"GET /test_sigfox/keep_history.php\?_temperature=%s\&_humidity=%s\&_mac=%x%x%x%x%x%x%x%x%x%x%x%x%x%x%x%x\r\n",tmp,humidity,
      lladdr[0],lladdr[1],lladdr[2],lladdr[3],lladdr[4], 
            lladdr[5],lladdr[6],lladdr[7],lladdr[8],lladdr[9],
            lladdr[10],lladdr[11],lladdr[12],lladdr[13],lladdr[14],
            lladdr[15]);  
    DataLen=strlen((const char*)buffer);
    printf("web ip:%x%x%x%x%x%x%x%x%x%x%x%x%x%x%x%x \r\n",lladdr[15],lladdr[14],lladdr[13],lladdr[12],lladdr[11], 
           lladdr[0],lladdr[1],lladdr[2],lladdr[3],lladdr[4], 
            lladdr[5],lladdr[6],lladdr[7],lladdr[8],lladdr[9],
            lladdr[10],lladdr[11],lladdr[12],lladdr[13],lladdr[14],
            lladdr[15]);
    sprintf((char*)buffer2,"AT+CIPSEND=%d\r\n",DataLen); 
    ESP8266_Write(buffer2);
    HAL_Delay(1000);   
  
    ESP8266_Write(buffer); 
    HAL_Delay(1000);  
  
  /* close a TCP connection */  
   // ESP8266_Write("AT+CIPCLOSE\n\r");
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