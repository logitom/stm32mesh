/**
* @file    wmbus_tim.c
* @author  VMA division - AMS/System Lab - NOIDA
* @version V2.4.1
* @date    1-Mar-2016
* @brief   This file contains routine for Timer. It is an implementation of
*          the timer interface specified by the header file wmbus_tim.h
*          of wM-BUS_Library_2013.
* @details
*
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
* TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
* DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
* FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
* CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
* THIS SOURCE CODE IS PROTECTED BY A LICENSE.
* FOR MORE INFORMATION PLEASE CAREFULLY READ THE LICENSE AGREEMENT FILE LOCATED
* IN THE ROOT DIRECTORY OF THIS FIRMWARE PACKAGE.
*
* <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
*/


#include "SDK_EVAL_Timers.h"
#include "SPIRIT_SDK_Util.h"


/**
* @addtogroup Application Application
* @{
*/

/**
* @addtogroup wmbus_user_application    WMBus User Application
* @{
*/

/**
* @addtogroup wmbus_tim    WMBus Timer Implementation
* @brief An implementation of the timer interface.
* @details See the file <i>WMBUS_Library_2013/wmbus_tim.h</i> for more details.
* @{
*/


/**
* @defgroup wmbus_tim_Private_Variables      WMBus Timer Private Variables
* @{
*/

static uint32_t NOverflows=0;

/**
*@}
*/


/**
* @defgroup wmbus_tim_Private_Defines        WMBus Timer Private Defines
* @{
*/

#define PROTOCOL_TIMER          TIM2
#define PROTOCOL_TIMER_ISR      TIM2_IRQHandler
TIM_HandleTypeDef               ProtocolTim_Handler={.Instance=PROTOCOL_TIMER};

#define NOW()          (NOverflows*0x10000+SdkEvalTimersGetCounter(&ProtocolTim_Handler))

/**
*@}
*/


/**
* @defgroup wmbus_tim_Exported_Functions         WMBus Timer Exported Functions
* @{
*/


/**
* @brief  Timer Interrupt Handler.
* @param  None
* @retval None
*/
void PROTOCOL_TIMER_ISR(void)   
{
  if(__HAL_TIM_GET_IT_SOURCE(&ProtocolTim_Handler, TIM_IT_UPDATE) !=RESET)
  {
    NOverflows++;
    __HAL_TIM_CLEAR_IT(&ProtocolTim_Handler, TIM_IT_UPDATE);
  }
}

/**
* @brief  Initialize Timer.
* @param  None
* @retval None
*/
void WMBus_TimInit(void)  
{
  /* Timer Time Period 100us resolution */
  SdkEvalTimersTimConfig(&ProtocolTim_Handler,3200-1,0xffff);
  SdkEvalTimersState(&ProtocolTim_Handler, ENABLE);
}

/**
* @brief  Get Time Stamp.
* @param  None
* @retval Returns the current value of the Timer-Counter Register
*/
uint32_t WMBus_TimGetTimestamp(void)
{
  return NOW();
}


/**
* @brief  Calculates the Time difference from a given time Stamp.
* @param  t0: Initial Time Stamp
* @retval Returns the Time Difference.
*/
uint32_t WMBus_TimDiffFromNow(uint32_t t0)
{
  volatile uint32_t now=NOW();
  
  if(now<t0)
  {
    /*
    case in which the current now is after the 32 bits max
             t0       2^32|0        now 
    ---------|------------|----------|---- 
    */
    return(0xffffffff-t0+now);
  }
  
  /* else return the simple difference */
  return(now-t0);
}


/**
* @brief  Inserts a time delay from a Initial Time Stamp
* @param  InitialTimeStamp: Initial Time Stamp 
* @param  ReqTimeDelay: The required Time Delay.
* @retval 0/1: Returns 1 When a delay is given else returns 0.
*/
uint8_t TimeDelay(uint32_t InitialTimeStamp,uint32_t ReqTimeDelay)
{
  uint32_t timeDiff;
  
  timeDiff = WMBus_TimDiffFromNow(InitialTimeStamp);
  
  if(timeDiff < ReqTimeDelay)
  {
    /*Return FALSE*/
    return 0;
  }
  else
  {
    /* Return TRUE*/
    return 1;
  }
}

/**
*@}
*/

/**
*@}
*/

/**
*@}
*/

/**
*@}
*/


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
