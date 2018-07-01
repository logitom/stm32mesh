/**
***********************************************************************************
* @file    radio_hal.c 
* @author  System Lab - NOIDA
* @version V1.0.0
* @date    15-May-2014 
* @brief   This file provides the initializations for TI, ST and Semtech RF shields
************************************************************************************
@attention
*
* COPYRIGHT(c) 2014 STMicroelectronics
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
*
******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/

#include "radio_hal.h"
#include "radio_shield_config.h"
#include "MCU_Interface.h" 
#include "SPIRIT_Config.h"
#include "cube_hal.h"
#include "SPIRIT1_Util.h"
#include "wmbus_phy.h"
#include "wmbus_link.h"
#include "wmbus_linkcommand.h"
/** @addtogroup BSP
* @{
*/

/** @addtogroup X-NUCLEO-IDS01Ax
* @{
*/

/** @addtogroup RADIO_HAL_LOW_LEVEL
* @brief This file provides set of firmware functions to manage
* RF tranceivers       
* @{
*/   

/* Private typedef -----------------------------------------------------------*/

RadioDriver_t radio_cb =
{
  .Init = Spirit1InterfaceInit, 
  .StartRx = Spirit1StartRx,
  .GetRxPacket = Spirit1GetRxPacket,
  .SetTxPacket = Spirit1StartTx,
};

MCULowPowerMode_t MCU_LPM_cb =
{
  .McuStopMode = MCU_Enter_StopMode,
  .McuStandbyMode = MCU_Enter_StandbyMode,
  .McuSleepMode = MCU_Enter_SleepMode
}; 

RadioLowPowerMode_t Radio_LPM_cb =
{
  .RadioShutDown = RadioPowerOFF,
  .RadioStandBy = RadioStandBy,
  .RadioPowerON = RadioPowerON
};


/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

uint8_t SW1_Clicked = 0x00;
WMBusDeviceType_t WMBusDeviceType;
uint32_t VolumeInL = 0x1234;
uint64_t accNum = 0x1E;
RadioDriver_t *pRadioDriver;
MCULowPowerMode_t *pMCU_LPM_Comm;
RadioLowPowerMode_t  *pRadio_LPM_Comm;

#if (DEVICE_METER_TYPE == 0x02) /*ElectricityMeter*/
/*Encryption Key for meter and meter id*/
  uint8_t aManufIdBuff[8] = {0x4E,0x8D,0x12,0x45,0x63,0x12,0x10,0x02};
  uint8_t aMeter_EnKey[16]={0xc2,0x86,0x69,0x6d,0x88,0x7c,0x9a,0xa0,0x61,\
    0x1b,0xbb,0x3e,0x20,0x25,0xa4,0x5a};
#elif (DEVICE_METER_TYPE == 0x03)/*GasMeter*/
/*Encryption Key for meter and meter id*/
uint8_t aManufIdBuff[8] = {0x4E,0x8d,0x11,0x45,0x63,0x17,0x10,0x03};
uint8_t aMeter_EnKey[16]={0x03,0x02,0x01,0x00,0x07,0x06,0x05,0x04,0x0b,\
  0x0a,0x09,0x08,0x0f,0x0e,0x0d,0x0c};
#elif (DEVICE_METER_TYPE == 0x04)/*Heatmeter*/
/*Encryption Key for meter and meter id for Heat meter*/
uint8_t aManufIdBuff[8] = {0x4E,0x8D,0x12,0x45,0x63,0x17,0x10,0x04};
uint8_t aMeter_EnKey[16]={0x0A,0x0B,0x0C,0x0D,0x0E,0x0F,0x10,0x11,0x12,\
  0x13,0x014,0x15,0x16,0x17,0x18,0x19};
#endif
/* Private function prototypes -----------------------------------------------*/

void Radio_HAL_Init(void);
void Radio_WMBus_Init(void);
void Radio_WMBus_On(void);
void Enter_LP_mode(void);
void Exit_LP_mode(void);
void MCU_Enter_StopMode(void);
void MCU_Enter_StandbyMode(void);
void MCU_Enter_SleepMode(void);
void RadioPowerON(void);
void RadioPowerOFF(void);
void RadioStandBy(void);
/* Private functions ---------------------------------------------------------*/

/**
* @brief  Initializes RF Transceiver's HAL.
* @param  None
* @retval None.
*/
void Radio_HAL_Init(void)
{
  pRadioDriver = &radio_cb;
  pRadioDriver->Init( );  
}

/**
* @brief  Initializes WMBus Layers.
* @param  None
* @retval None.
*/
void Radio_WMBus_Init(void)
{    
  uint8_t a_field[6]={0x11,0x45,0x63,0x1F,0x10,0x04};
  uint8_t m_field[2]={0x8D,0x4E};
  
  /* Initialization of the link Layer first */
  WMBus_LinkInit(m_field, a_field, METER, WMBUS_FRAME_FORMAT);
  
  /* Set output power*/
  WMBus_PhySetAttr(WMBUS_PHY_OP_POWER, TX_OUTPUT_POWER);
  WMBus_PhySetAttr(WMBUS_PHY_RF_OFFSET,SpiritManagementGetOffset());
  
  /* Initialization of the physical Layer */ 
  WMBus_PhyInit(DEVICE_WMBUS_MODE, METER); 
  
  WMBus_LinkSetAttribute(LINK_ATTR_ID_LINK_TIMEOUT,3000); //300ms
  
  WMBusDeviceType = METER;
}


/**
* @brief  SPIRIT1 Data Transfer Routine.
* @param  None
* @retval None.
*/
void Radio_WMBus_On(void)
{ 
  /* If the device is a meter*/ 
  if (METER == WMBusDeviceType)
  {      
    if(SW1_Clicked)
    {
      SW1_Clicked = 0x00;
      WMBus_LinkSendInstallationRequest(NULL, 0, 0x78); 
      HAL_Delay(1000); 
    }
  }
}

/**
* @brief  This routine will put the radio and mcu in LPM
* @param  None
* @retval None
*/
void Enter_LP_mode(void)
{
  
  pMCU_LPM_Comm = &MCU_LPM_cb;
  pRadio_LPM_Comm = &Radio_LPM_cb;
  
#if defined(MCU_STOP_MODE)&&defined(RF_SHUTDOWN) 
  {
    pRadio_LPM_Comm->RadioShutDown();  
    pMCU_LPM_Comm->McuStopMode();
  }
#elif defined(MCU_STOP_MODE)&&defined(RF_STANDBY) 
  {
    pRadio_LPM_Comm->RadioStandBy();
    pMCU_LPM_Comm->McuStopMode();
  }  
#elif defined(MCU_STOP_MODE)&&defined(RF_SLEEP) 
  {
    pRadio_LPM_Comm->RadioSleep();
    pMCU_LPM_Comm->McuStopMode();
  }   
#elif defined(MCU_STANDBY_MODE)&&defined(RF_SHUTDOWN) 
  {
    pRadio_LPM_Comm->RadioShutDown(); 
    pMCU_LPM_Comm->McuStandbyMode();
  } 
#elif defined(MCU_STANDBY_MODE)&&defined(RF_STANDBY) 
  {
    pRadio_LPM_Comm->RadioStandBy();  
    pMCU_LPM_Comm->McuStandbyMode();
  }
#elif defined(MCU_STANDBY_MODE)&&defined(RF_SLEEP) 
  {
    pRadio_LPM_Comm->RadioSleep();
    pMCU_LPM_Comm->McuStandbyMode();
  }  
#elif defined(MCU_SLEEP_MODE)&&defined(RF_SHUTDOWN) 
  {
    pRadio_LPM_Comm->RadioShutDown(); 
    pMCU_LPM_Comm->McuSleepMode();
  }
#elif defined(MCU_SLEEP_MODE)&&defined(RF_STANDBY) 
  {
    pRadio_LPM_Comm->RadioStandBy(); 
    pMCU_LPM_Comm->McuSleepMode();
  }
#elif defined(MCU_SLEEP_MODE)&&defined(RF_SLEEP) 
  {
    pRadio_LPM_Comm->RadioSleep();
    pMCU_LPM_Comm->McuSleepMode();
  }
#elif defined(MCU_STOP_MODE)
  pMCU_LPM_Comm->McuStopMode();
  
#elif defined(MCU_STANDBY_MODE)
  pMCU_LPM_Comm->McuStandbyMode();
  
#else
  pMCU_LPM_Comm->McuSleepMode();
#endif
}

/**
* @brief  This routine wake-up the mcu and radio from LPM
* @param  None
* @retval None
*/
void Exit_LP_mode(void)
{
  pRadio_LPM_Comm = &Radio_LPM_cb;
  pRadio_LPM_Comm->RadioPowerON();  
}

/**
* @brief  This routine puts the MCU in stop mode
* @param  None
* @retval None
*/
void MCU_Enter_StopMode(void)
{
  HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);  /* Infinite loop */
}

/**
* @brief  This routine puts the MCU in standby mode
* @param  None
* @retval None
*/
void MCU_Enter_StandbyMode(void)
{
  HAL_PWR_EnterSTANDBYMode();  /* Infinite loop */
}

/**
* @brief  This routine puts the MCU in sleep mode
* @param  None
* @retval None
*/
void MCU_Enter_SleepMode(void)
{
  HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);  /* Infinite loop */
}

/**
* @brief  This function will turn on the radio and waits till it enters the Ready state.
* @param  Param:None. 
* @retval None
*                       
*/
void RadioPowerON(void)
{
  SpiritCmdStrobeReady();   
  do{
    /* Delay for state transition */
    for(volatile uint8_t i=0; i!=0xFF; i++);
    
    /* Reads the MC_STATUS register */
    SpiritRefreshStatus();
  }
  while(g_xStatus.MC_STATE!=MC_STATE_READY);
}


/**
* @brief  This function will Shut Down the radio.
* @param  Param:None. 
* @retval None
*                       
*/
void RadioPowerOFF(void)
{
  SpiritEnterShutdown();
}


/**
* @brief  This function will put the radio in standby state.
* @param  Param:None. 
* @retval None
*                       
*/
void RadioStandBy(void)
{
  SpiritCmdStrobeStandby();  
#if 0  
  do{
    /* Delay for state transition */
    for(volatile uint8_t i=0; i!=0xFF; i++);
    
    /* Reads the MC_STATUS register */
    SpiritRefreshStatus();
  }
  while(g_xStatus.MC_STATE!=MC_STATE_STANDBY);
#endif
}

/**
  * @brief GPIO EXTI callback
  * @param uint16_t GPIO_Pin
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  
   SW1_Clicked = 0x01;
    
#if defined(LPM_ENABLE)  
#if defined(MCU_STOP_MODE)/*if MCU is in stop mode*/        

  /* Clear Wake Up Flag */
  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
      
  /* Configures system clock after wake-up from STOP: enable HSE, PLL and select
    PLL as system clock source (HSE and PLL are disabled in STOP mode) */
    SystemClockConfig_STOP(); 
#endif
#if defined(MCU_SLEEP_MODE) 
    /* Resume Tick interrupt if disabled prior to sleep mode entry*/
    HAL_ResumeTick();
#endif 
    
    /* Initialize LEDs*/
    RadioShieldLedInit(RADIO_SHIELD_LED);
    BSP_LED_Init(LED2);
    
#if defined(RF_STANDBY)    
    Exit_LP_mode();
#endif
#endif
}

/**
* @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
