/**
******************************************************************************
* @file    stm32l1xx_it.c
* @author  System LAB
* @version V1.0.0
* @date    17-June-2015
* @brief   Main Interrupt Service Routines
******************************************************************************
* @attention
*
* <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
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
******************************************************************************
*/
/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx_it.h"
#include "stm32l1xx_nucleo.h"
#include "radio_gpio.h"
#include "slip.h"
#include "spirit1.h"
#include "main.h"
#include "sensors.h"



/** @addtogroup L1
 *  @ingroup Border_router
 *  @{
 */


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

extern UART_HandleTypeDef UartHandle;
extern TIM_HandleTypeDef htim2;
extern const struct sensors_sensor button_sensor;
extern volatile unsigned long seconds;
extern volatile clock_time_t ticks;
extern volatile uint32_t rtimer_clock;
extern UART_HandleTypeDef huart1;

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
* @brief   This function handles NMI exception.
* @param  None
* @retval None
*/
void NMI_Handler(void)
{
}
void WWDG_IRQHandler(void)
{
  while(1);
}
/**
* @brief  This function handles Hard Fault exception.
* @param  None
* @retval None
*/
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
* @brief  This function handles Memory Manage exception.
* @param  None
* @retval None
*/
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
* @brief  This function handles Bus Fault exception.
* @param  None
* @retval None
*/
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
* @brief  This function handles Usage Fault exception.
* @param  None
* @retval None
*/
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
* @brief  This function handles SVCall exception.
* @param  None
* @retval None
*/
void SVC_Handler(void)
{
}

/**
* @brief  This function handles Debug Monitor exception.
* @param  None
* @retval None
*/
void DebugMon_Handler(void)
{
}

/**
* @brief  This function handles PendSVC exception.
* @param  None
* @retval None
*/
void PendSV_Handler(void)
{
}


/******************************************************************************/
/*                 STM32L1xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32l1xx.s).                                               */
/******************************************************************************/


/**
* @brief  This function handles External lines 15 to 4 interrupt request.
* @param  None
* @retval None
*/
void EXTI0_IRQHandler(void)
{
  /* EXTI line interrupt detected */
  if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_0) != RESET) 
  { 
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);
  }
  while(1);
}

void EXTI1_IRQHandler(void)
{
  /* EXTI line interrupt detected */
  if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_1) != RESET) 
  { 
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_1);
  }  
  while(1);
}

void EXTI2_IRQHandler(void)
{
  /* EXTI line interrupt detected */
  if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_2) != RESET) 
  { 
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_2);
  }
  while(1);
}

void EXTI3_IRQHandler(void)
{
  /* EXTI line interrupt detected */
  if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_3) != RESET) 
  { 
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_3);
  }
  while(1);
}


/**
* @brief  This function handles External lines 15 to 4 interrupt request.
* @param  None
* @retval None
*/
void EXTI9_5_IRQHandler(void)
{
  /* EXTI line 7 interrupt detected */
  if(__HAL_GPIO_EXTI_GET_IT(RADIO_GPIO_3_EXTI_LINE))
  {
    __HAL_GPIO_EXTI_CLEAR_IT(RADIO_GPIO_3_EXTI_LINE);

    HAL_GPIO_EXTI_Callback(RADIO_GPIO_3_EXTI_LINE);

    spirit1_interrupt_callback();
  }
  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_9);
  
  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_8);
  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_7);
  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_6);
  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_5);
  
#ifndef  LPM_ENABLE
  
  //  if(__HAL_GPIO_EXTI_GET_IT(KEY_BUTTON_EXTI_LINE) != RESET)
  //  {
  //    __HAL_GPIO_EXTI_CLEAR_IT(KEY_BUTTON_EXTI_LINE);
  //
  // Set_KeyStatus(SET);
  //  }
  //
#else /*Low Power mode enabled*/ 
  
#if defined(RF_STANDBY)/*if spirit1 is in standby*/
  
  if(EXTI->PR & KEY_BUTTON_EXTI_LINE)
  {
    HAL_GPIO_EXTI_Callback(KEY_BUTTON_EXTI_LINE);
    /* EXTI line 13 interrupt detected */
    if(HAL_GPIO_ReadPin(KEY_BUTTON_GPIO_PORT, KEY_BUTTON_PIN) == 0x01) //0x00
    {
      HAL_GPIO_EXTI_Callback(KEY_BUTTON_EXTI_LINE);
      
      PushButtonStatusWakeup = SET;
      PushButtonStatusData = RESET;
      wakeupCounter = LPM_WAKEUP_TIME; 
      dataSendCounter = DATA_SEND_TIME;
      dataSendCounter++;
    }
    __HAL_GPIO_EXTI_CLEAR_IT(KEY_BUTTON_EXTI_LINE);
  } 
#else /*if spirit1 is not in standby or sleep mode but MCU is in LPM*/
  
  if(__HAL_GPIO_EXTI_GET_IT(KEY_BUTTON_EXTI_LINE) != RESET) 
  { 
    __HAL_GPIO_EXTI_CLEAR_IT(KEY_BUTTON_EXTI_LINE);
    
    HAL_GPIO_EXTI_Callback(KEY_BUTTON_EXTI_LINE);
    
    Set_KeyStatus(SET);
  }
#endif
#endif
}



/**
* @brief  This function handles EXTI15_10_IRQHandler
* @param  None
* @retval None
*/
void EXTI15_10_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(USER_BUTTON_PIN);
}


/**
* @brief  This function handles UART interrupt request.
* @param  None
* @retval None
*/

void USART2_IRQHandler(void)
{
	UART_HandleTypeDef *huart = &UartHandle;

	if(__HAL_UART_GET_FLAG(huart, UART_FLAG_PE)){
		__HAL_UART_CLEAR_PEFLAG(huart);
	}

	if(__HAL_UART_GET_FLAG(huart, UART_FLAG_FE)){
		__HAL_UART_CLEAR_FEFLAG(huart);
	}

	if(__HAL_UART_GET_FLAG(huart, UART_FLAG_NE)){
		__HAL_UART_CLEAR_NEFLAG(huart);
	}

	if(__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE)){
		__HAL_UART_CLEAR_OREFLAG(&UartHandle);
	}

	if(__HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE)){
		//slip_input_byte(UartHandle.Instance->DR);/*Uncomment this line if slip needs to be enabled*/
	}
}

/**
* @brief  This function handles TIM2 interrupt request.
* @param  None
* @retval None
*/
void TIM2_IRQHandler(void)
{
  /* clear interrupt pending flag */
  __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);

  rtimer_clock++;
}


/**
* @brief  Tx Transfer completed callback
* @param  UartHandle: UART handle.
* @note   This example shows a simple way to report end of IT Tx transfer, and
*         you can add your own implementation.
* @retval None
*/
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  

}

/**
* @brief This function handles USART1 global interrupt.
*/
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */
  
  //if(__HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE)){
  
  //ESP8266_Write();
  //HAL_UART_Receive_IT(&huart1,(uint8_t*)&huart1.pRxBuffPtr,1);
 // __HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_RXNE);
  /* USER CODE END USART1_IRQn 1 */
}

/**
* @brief  Systick Handler
* @param  None
* @retval None
*/
void SysTick_Handler(void)
{
  HAL_IncTick();
  Contiki_SysTick_Handler();
}


/**
* @brief  Systick Handler for Contiki timings
* @param  None
* @retval None
*/
void Contiki_SysTick_Handler(void)
{
  ticks++;
  if((ticks % CLOCK_SECOND) == 0) {
    seconds++;
    energest_flush();
  }

  if(etimer_pending()) {
    etimer_request_poll();
  }
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
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


  if (GPIO_Pin == USER_BUTTON_PIN)
  {
    sensors_changed(&button_sensor);
  }

}

/**
* @}
*/


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
