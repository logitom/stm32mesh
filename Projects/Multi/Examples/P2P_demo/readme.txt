/**
  @page SPIRIT1 Expansion Board for STM32 Nucleo Boards Point-to-Point example

  @verbatim
  ******************** (C) COPYRIGHT 2014 STMicroelectronics *******************
  * @file    Point-to-Point Example/readme.txt
  * @author  Central Lab
  * @version V1.2.1
  * @date    15-May-2017
  * @brief   This application is an example to show a Point-to-Point 
             communication between two nodes using STM32 Nucleo boards and 
             SPIRIT1 expansion boards 
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

  This example is to demonstrate Point-to-Point communication using Spirit1 
  driver and STM32Cube firmware. 
 
@par Hardware and Software environment

  - This example runs on STM32 Nucleo devices with SPIRIT1 expansion board
    (X-NUCLEO-IDS01A4 for 868 MHz or X-NUCLEO-IDS01A5 for 915 MHz)
  - This example has been tested with STMicroelectronics:
    - NUCLEO-L053R8 RevC board
    - NUCLEO-L152RE RevC board
    - NUCLEO-F401RE RevC board
    and can be easily tailored to any other supported device and development 
    board.
     
  - User can select desired P2P features in spirit1_appli.h file located 
     @P2P_Demo/Inc

  /*******Platform definition : Uncomment the expansion board to be used*********/
#define X_NUCLEO_IDS01A4		
// #define X_NUCLEO_IDS01A5	
/******************************************************************************/



/************************Set the Node address**********************************/ 
/*Source address of one node should be destination for other node & vice-versa*/   
#define MY_ADDRESS                  0x44//0x34//
#define DESTINATION_ADDRESS         0x44//0x44// 
/******************************************************************************/
    


/**************************Uncomment the system Operating mode*****************/
//#define USE_LOW_POWER_MODE

#if defined (USE_LOW_POWER_MODE)
//#define MCU_SLEEP_MODE
#define MCU_STOP_MODE
#endif
/******************************************************************************/


#ifdef USE_BASIC_PROTOCOL
  //#define CSMA_ENABLE  /* Uncomment this line to enable the CSMA */
#endif


@par How to use it ? 

In order to make the program work, you must do the following:
 - WARNING: before opening the project with any toolchain be sure your folder
   installation path is not too in-depth since the toolchain may report errors
   after building.
 - The tested tool chain and environment is explained in the Release notes
 - Open the suitable toolchain (IAR, Keil, System Workbench for STM32) and open 
    the project for the required STM32 Nucleo board
 - Rebuild all files and load your image into target memory.
 - Run the example.
 - Alternatively, you can download the pre-built binaries in "Binary" 
   folder included in the distributed package.

 How to verify the firmware
 - Power-up the two nodes, each comprising of X-NUCLEO-IDS01A4/5 mounted on 
   STM32 Nucleo board
 - The LED LD2 on STM32 Nucleo board starts to blink
 - User can press user button B1 (Blue Colour) on any of the STM32 Nucleo board
    This causes a command packet to be sent from the node (Node1) to Node2
 - The successful communication with the other node will cause the LED D1 on 
    X-NUCLEO-IDS01Ax to blink on the receiving node (Node2)
 - The receiving node (Node2) will acknowledge the command by sending an RF 
    packet and this can be seen by blink of LED D1 on X-NUCLEO-IDS01Ax of 
    the node-1 
 
    To summarise:
    
    *1*  User presses B1 on Node1
    *2*  Node1  ----------- RF Command packet ------>   Node2
    *3*                                    Success = LED D1 on Node2 Blinks
    *4*  Node1  <---------- Ack packet --------------   Node2                                                     
    *5*  Success = LED D1 Blinks on Node1
 
 
 - IMPORTANT NOTE: To avoid issues with USB connection (mandatory if you have USB 3.0), it is   
   suggested to update the ST-Link/V2 firmware for STM32 Nucleo boards to the latest version.
   Please refer to the readme.txt file in the Applications directory for details.


 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
