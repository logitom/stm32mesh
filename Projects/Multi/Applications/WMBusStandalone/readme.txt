/**
  @page SPIRIT1 Expansion Board for STM32 Nucleo Boards wM-Bus meter example

  @verbatim
  ******************** (C) COPYRIGHT  STMicroelectronics ***********************
  * @file    WMBusExample/readme.txt
  * @author  Central Lab
  * @version V1.2.0
  * @date    10-Sep-2016
  * @brief   This application is an example to use wM-Bus implementation between 
             a STM32 Nucleo boards and SPIRIT1 expansion boards acting like a meter
	     and a concentrator (SPIRIT1 Boards) connected to a PC GUI
  ******************************************************************************
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
  @endverbatim

@par Example Description

This example is a part of the WMBus Library package using STM32Cube firmware. 

This is a demo implementation of the standard WMBUS protocol. It takes in account the physical and data link layers
defined by the standard.

The example shows how to configure the packets in order to transmit or receive WMBUS packets.



@par Hardware and Software environment

  - This example runs on STM32 Nucleo devices with SPIRIT1 expansion board
    (X-NUCLEO-IDS01A4 for 868 MHz 
     Note: wM-Bus standard do not specify any communication on 915 MHz
           X-NUCLEO-IDS01A5 for 915 MHz is not used for testing)
  - This example has been tested with STMicroelectronics:
    - NUCLEO-L053R8 RevC board
    - NUCLEO-L152RE RevC board
    - NUCLEO-F401RE RevC board
    and can be easily tailored to any other supported device and development board.
      
  - For WMBus application demonstration One Spirit1 Expansion Board + STM32 Nucleo is programmed as meter 
  - Another side, STEVAL-IKR001Vx is used as Concentrator or data collector connected to PC
   
  - Operating frequency is supported for this WMBus demo is:
    => 868 MHz 

  
  - User can select desired frequency by defining:
    => DEVICE_BAND_868MHz : Supports S/T/R modes of WMBus
  
  - Desired WMBus features can be selected in rf_hal.h file by changing below section of code:
  
    /* Wireless Meter Bus Specific Configuration----------------------------------*/

	#define TX_OUTPUT_POWER ((float)3.0)/* Between [-3.0 , +27.0] if the RF board 
	has an external PA. Otherwise between [-30.0 , +13.0].*/
	#define RX_TIMEOUT ((uint32_t)1000)

	/* Private macro -------------------------------------------------------------*/
	#define     DEVICE_BAND_868MHz
    
    
	#ifdef DEVICE_BAND_868MHz
	#define DEVICE_WMBUS_MODE S1_MODE    /*S1_MODE/ S1m_MODE/ S2_MODE/ T1_MODE/\
                                              T2_MODE/ R2_MODE*/
	#define DEVICE_WMBUS_CHANNEL CHANNEL_1 
	#define WMBUS_FRAME_FORMAT FRAME_FORMAT_A
	#endif

	#define  DEVICE_TYPE_METER  
    
	#ifdef DEVICE_TYPE_METER
	#define DEVICE_TYPE         METER
	#define DEVICE_METER_TYPE   (0x03)
	#endif

	/*  Uncomment the system Operating mode */
	#define USE_LOW_POWER_MODE

	#if defined (USE_LOW_POWER_MODE)
	#define LPM_ENABLE
	#define MCU_STOP_MODE
	//#define MCU_SLEEP_MODE
	#define RF_STANDBY
	#endif


  /*------------------------------------------------------------------------------*/
    
  - Connect the board to PC through USB - mini connector of nucleo board.

  - Flash the board for desired configuration as explained above.
  
@par How to use it ? 

In order to make the program work, you must do the following:
 - WARNING: before opening the project with any toolchain be sure your folder
   installation path is not too in-depth since the toolchain may report errors
   after building.
 - The tested tool chain and environment is explained in the Release notes
 - Open the suitable toolchain (IAR, Keil, System Workbench for STM32) and open the project 
    for the required STM32 Nucleo board
 - Rebuild all files and load your image into target memory.
 - Run the example.
 - Alternatively, you can download the pre-built binaries in "Binary" 
   folder included in the distributed package.

 - IMPORTANT NOTE: To avoid issues with USB connection (mandatory if you have USB 3.0), it is   
   suggested to update the ST-Link/V2 firmware for STM32 Nucleo boards to the latest version.
   Please refer to the readme.txt file in the Applications directory for details.



 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
