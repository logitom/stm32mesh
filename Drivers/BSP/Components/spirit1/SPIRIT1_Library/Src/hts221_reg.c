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

#include "hts221_reg.h"
#include "stm32l1xx_hal.h"

/* Private variables ---------------------------------------------------------*/
//#define TX_BUF_DIM          1000

I2C_HandleTypeDef hi2c1;


typedef struct {
  float x0;
  float y0;
  float x1;
  float y1;  
} lin_t;

static axis1bit16_t data_raw_humidity;
static axis1bit16_t data_raw_temperature;
static float humidity_perc;
static float temperature_degC;
static uint8_t whoamI;
//static uint8_t tx_buffer[TX_BUF_DIM];
static hts221_ctx_t dev_ctx;
static axis1bit16_t coeff;
static lin_t lin_hum;
static lin_t lin_temp;
static uint8_t tx_buffer[40];
/* I2C1 init function */
void MX_I2C1_Init(void)
{
   GPIO_InitTypeDef  GPIO_InitStruct;

  /* Setup the I2C clock source */
 // BOARD_I2C_EXPBD_SET_CLK_SOURCE();

  /* Enable I2C GPIO clocks */
  //BOARD_I2C_EXPBD_SCL_SDA_GPIO_CLK_ENABLE();

  /* I2C_EXPBD SCL and SDA pins configuration -------------------------------------*/
  GPIO_InitStruct.Pin        = GPIO_PIN_8 | GPIO_PIN_9;
  GPIO_InitStruct.Mode       = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Speed      = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Pull       = GPIO_NOPULL;
  GPIO_InitStruct.Alternate  = ((uint8_t)0x04);

  HAL_GPIO_Init( GPIOB, &GPIO_InitStruct );

  /* Enable the I2C_EXPBD peripheral clock */
   __I2C1_CLK_ENABLE();

  /* Force the I2C peripheral clock reset */
  __I2C1_FORCE_RESET();

  /* Release the I2C peripheral clock reset */
  __I2C1_RELEASE_RESET();

  /* Enable and set I2C_EXPBD Interrupt priority */
  HAL_NVIC_SetPriority(I2C1_EV_IRQn, 0x00, 0x01);
  HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
  
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
       printf(" I2C Init Failed ...\n");     
  }else
  {
       printf(" I2C Init successful ...\n");   
  }
}

/* Private functions ---------------------------------------------------------*/

/*
 *   Replace the functions "platform_write" and "platform_read" with your
 *   platform specific read and write function.
 *   This example use an STM32 evaluation board and CubeMX tool.
 *   In this case the "*handle" variable is usefull in order to select the
 *   correct interface but the usage uf "*handle" is not mandatory.
 */

int32_t platform_write(void *handle, uint8_t Reg, uint8_t *Bufp,
                              uint16_t len)
{
  if (handle == &hi2c1)
  {
    /* enable auto incremented in multiple read/write commands */
    Reg |= 0x80; 
    HAL_I2C_Mem_Write(handle, HTS221_I2C_ADDRESS, Reg,
                      I2C_MEMADD_SIZE_8BIT, Bufp, len, 1000);
  }
  return 0;
}

int32_t platform_read(void *handle, uint8_t Reg, uint8_t *Bufp,
                             uint16_t len)
{
  uint8_t mem_read=99;
  if (handle == &hi2c1)
  {
    /* enable auto incremented in multiple read/write commands */
    Reg |= 0x80;
    mem_read=HAL_I2C_Mem_Read(handle, HTS221_I2C_ADDRESS, Reg,
                     I2C_MEMADD_SIZE_8BIT, Bufp, len, 1000);
  }
  
  //printf("i2c read status:%d\n",mem_read);
  return 0;
}


/*
 *  Function used to apply coefficient
 */
float linear_interpolation(lin_t *lin, int16_t x) 
{
 return ((lin->y1 - lin->y0) * x +  ((lin->x1 * lin->y0) - (lin->x0 * lin->y1)))
        / (lin->x1 - lin->x0);
}



int HTS221_Init(void)
{
    /*
   *  Initialize mems driver interface
   */
   
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.handle = &hi2c1;  
  
  /*
   *  Check device ID
   */
  whoamI = 0;
  hts221_device_id_get(&dev_ctx, &whoamI);
  if ( whoamI != HTS221_ID )
    return -1; /*manage here device not found */

  /*
   *  Read humidity calibration coefficient
   */  
  //

  hts221_hum_adc_point_0_get(&dev_ctx, coeff.u8bit);
  lin_hum.x0 = (float)coeff.i16bit; 
  hts221_hum_rh_point_0_get(&dev_ctx, coeff.u8bit);
  lin_hum.y0 = (float)coeff.u8bit[0];
  hts221_hum_adc_point_1_get(&dev_ctx, coeff.u8bit);
  lin_hum.x1 = (float)coeff.i16bit; 
  hts221_hum_rh_point_1_get(&dev_ctx, coeff.u8bit);  
  lin_hum.y1 = (float)coeff.u8bit[0];
    
  /*
   *  Read temperature calibration coefficient
   */  
  
  hts221_temp_adc_point_0_get(&dev_ctx, coeff.u8bit);
  lin_temp.x0 = (float)coeff.i16bit; 
  hts221_temp_deg_point_0_get(&dev_ctx, coeff.u8bit);
  lin_temp.y0 = (float)coeff.u8bit[0];
  hts221_temp_adc_point_1_get(&dev_ctx, coeff.u8bit);
  lin_temp.x1 = (float)coeff.i16bit; 
  hts221_temp_deg_point_1_get(&dev_ctx, coeff.u8bit);  
  lin_temp.y1 = (float)coeff.u8bit[0];  
  
  /*
   *  Enable Block Data Update
   */
  hts221_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
  /*
   * Set Output Data Rate
   */
  hts221_data_rate_set(&dev_ctx, HTS221_ODR_1Hz);
  /*
   * Device power on
   */
  hts221_power_on_set(&dev_ctx, PROPERTY_ENABLE);
  
}

void HTS221_Get_Humidity_Temper(char *tx_buffer)
{
    /*
     * Read output only if new value is available
     */
    hts221_reg_t reg;
    hts221_status_get(&dev_ctx, &reg.status_reg);
    
    if (reg.status_reg.h_da)
    {
      /* Read humidity data */
      memset(data_raw_humidity.u8bit, 0x00, sizeof(int16_t));
      hts221_humidity_raw_get(&dev_ctx, data_raw_humidity.u8bit);
      //HAL_Delay(200);
      humidity_perc = linear_interpolation(&lin_hum, data_raw_humidity.i16bit);
      
      if (humidity_perc < 0) humidity_perc = 0;
      else if (humidity_perc > 100) humidity_perc = 100;
       
      //  sprintf((char*)tx_buffer, "node1:Humidity [%%]:%3.2f", humidity_perc);
    //  printf("read humidity:%3.2f\n",humidity_perc);     
    }
    if (reg.status_reg.t_da)
    {
      /* Read temperature data */
      memset(data_raw_temperature.u8bit, 0x00, sizeof(int16_t));
      hts221_temperature_raw_get(&dev_ctx, data_raw_temperature.u8bit);
      //HAL_Delay(200);
      temperature_degC = linear_interpolation(&lin_temp, data_raw_temperature.i16bit);
      if (temperature_degC < 0) temperature_degC = 0;
      else if (temperature_degC > 100) temperature_degC = 100;
    
      //  sprintf((char*)tx_buffer, " Temp[degC]:%6.2f\r\n", temperature_degC );
     // printf("%s\n",tx_buffer); 
     // printf("read temp:%6.2f\n",temperature_degC);  
    }
    sprintf((char*)tx_buffer, "t:%3.2f h:%3.2f\r\n", temperature_degC,humidity_perc );
    printf("%s\n",tx_buffer); 
}




/**
  * @addtogroup  hts221
  * @brief  This file provides a set of functions needed to drive the
  *         hts221 enanced inertial module.
  * @{
  */

/**
  * @addtogroup  interfaces_functions
  * @brief  This section provide a set of functions used to read and write
  *         a generic register of the device.
  * @{
  */

/**
  * @brief  Read generic device register
  *
  * @param  hts221_ctx_t* ctx: read / write interface definitions
  * @param  uint8_t reg: register to read
  * @param  uint8_t* data: pointer to buffer that store the data read
  * @param  uint16_t len: number of consecutive register to read
  *
  */
int32_t hts221_read_reg(hts221_ctx_t* ctx, uint8_t reg, uint8_t* data,
                        uint16_t len)
{
  return ctx->read_reg(ctx->handle, reg, data, len);
}

/**
  * @brief  Write generic device register
  *
  * @param  hts221_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t reg: register to write
  * @param  uint8_t* data: pointer to data to write in register reg
  * @param  uint16_t len: number of consecutive register to write
  *
*/
int32_t hts221_write_reg(hts221_ctx_t* ctx, uint8_t reg, uint8_t* data,
                         uint16_t len)
{
  return ctx->write_reg(ctx->handle, reg, data, len);
}

/**
  * @}
  */

/**
  * @addtogroup  data_generation_c
  * @brief   This section group all the functions concerning data generation
  * @{
  */

/**
  * @brief  humidity_avg: [set]  The numbers of averaged humidity samples.
  *
  * @param  hts221_ctx_t *ctx: read / write interface definitions
  * @param  hts221_avgh_t: change the values of avgh in reg AV_CONF
  *
  */
int32_t hts221_humidity_avg_set(hts221_ctx_t *ctx, hts221_avgh_t val)
{
  hts221_reg_t reg;
  int32_t mm_error;

  mm_error = hts221_read_reg(ctx, HTS221_AV_CONF, &reg.byte, 1);
  reg.av_conf.avgh = val;
  mm_error = hts221_write_reg(ctx, HTS221_AV_CONF, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  humidity_avg: [get]  The numbers of averaged humidity samples.
  *
  * @param  hts221_ctx_t *ctx: read / write interface definitions
  * @param  hts221_avgh_t: Get the values of avgh in reg AV_CONF
  *
  */
int32_t hts221_humidity_avg_get(hts221_ctx_t *ctx, hts221_avgh_t *val)
{
  hts221_reg_t reg;
  int32_t mm_error;

  mm_error = hts221_read_reg(ctx, HTS221_AV_CONF, &reg.byte, 1);
  *val = (hts221_avgh_t) reg.av_conf.avgh;

  return mm_error;
}

/**
  * @brief  temperature_avg: [set]  The numbers of averaged temperature samples.
  *
  * @param  hts221_ctx_t *ctx: read / write interface definitions
  * @param  hts221_avgt_t: change the values of avgt in reg AV_CONF
  *
  */
int32_t hts221_temperature_avg_set(hts221_ctx_t *ctx, hts221_avgt_t val)
{
  hts221_reg_t reg;
  int32_t mm_error;

  mm_error = hts221_read_reg(ctx, HTS221_AV_CONF, &reg.byte, 1);
  reg.av_conf.avgt = val;
  mm_error = hts221_write_reg(ctx, HTS221_AV_CONF, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  temperature_avg: [get]  The numbers of averaged temperature
  *                                 samples.
  *
  * @param  hts221_ctx_t *ctx: read / write interface definitions
  * @param  hts221_avgt_t: Get the values of avgt in reg AV_CONF
  *
  */
int32_t hts221_temperature_avg_get(hts221_ctx_t *ctx, hts221_avgt_t *val)
{
  hts221_reg_t reg;
  int32_t mm_error;

  mm_error = hts221_read_reg(ctx, HTS221_AV_CONF, &reg.byte, 1);
  *val = (hts221_avgt_t) reg.av_conf.avgt;

  return mm_error;
}

/**
  * @brief  data_rate: [set]  Output data rate selection.
  *
  * @param  hts221_ctx_t *ctx: read / write interface definitions
  * @param  hts221_odr_t: change the values of odr in reg CTRL_REG1
  *
  */
int32_t hts221_data_rate_set(hts221_ctx_t *ctx, hts221_odr_t val)
{
  hts221_reg_t reg;
  int32_t mm_error;

  mm_error = hts221_read_reg(ctx, HTS221_CTRL_REG1, &reg.byte, 1);
  reg.ctrl_reg1.odr = val;
  mm_error = hts221_write_reg(ctx, HTS221_CTRL_REG1, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  data_rate: [get]  Output data rate selection.
  *
  * @param  hts221_ctx_t *ctx: read / write interface definitions
  * @param  hts221_odr_t: Get the values of odr in reg CTRL_REG1
  *
  */
int32_t hts221_data_rate_get(hts221_ctx_t *ctx, hts221_odr_t *val)
{
  hts221_reg_t reg;
  int32_t mm_error;

  mm_error = hts221_read_reg(ctx, HTS221_CTRL_REG1, &reg.byte, 1);
  *val = (hts221_odr_t) reg.ctrl_reg1.odr;

  return mm_error;
}

/**
  * @brief  block_data_update: [set] Blockdataupdate.
  *
  * @param  hts221_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of bdu in reg CTRL_REG1
  *
  */
int32_t hts221_block_data_update_set(hts221_ctx_t *ctx, uint8_t val)
{
  hts221_reg_t reg;
  int32_t mm_error;

  mm_error = hts221_read_reg(ctx, HTS221_CTRL_REG1, &reg.byte, 1);
  reg.ctrl_reg1.bdu = val;
  mm_error = hts221_write_reg(ctx, HTS221_CTRL_REG1, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  block_data_update: [get] Blockdataupdate.
  *
  * @param  hts221_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of bdu in reg CTRL_REG1
  *
  */
int32_t hts221_block_data_update_get(hts221_ctx_t *ctx, uint8_t *val)
{
  hts221_reg_t reg;
  int32_t mm_error;

  mm_error = hts221_read_reg(ctx, HTS221_CTRL_REG1, &reg.byte, 1);
  *val = reg.ctrl_reg1.bdu;

  return mm_error;
}

/**
  * @brief  one_shoot_trigger: [set]  One-shot mode. Device perform a
  *                                   single measure.
  *
  * @param  hts221_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of one_shot in reg CTRL_REG2
  *
  */
int32_t hts221_one_shoot_trigger_set(hts221_ctx_t *ctx, uint8_t val)
{
  hts221_reg_t reg;
  int32_t mm_error;

  mm_error = hts221_read_reg(ctx, HTS221_CTRL_REG2, &reg.byte, 1);
  reg.ctrl_reg2.one_shot = val;
  mm_error = hts221_write_reg(ctx, HTS221_CTRL_REG2, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  one_shoot_trigger: [get]  One-shot mode. Device perform a
  *                                   single measure.
  *
  * @param  hts221_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of one_shot in reg CTRL_REG2
  *
  */
int32_t hts221_one_shoot_trigger_get(hts221_ctx_t *ctx, uint8_t *val)
{
  hts221_reg_t reg;
  int32_t mm_error;

  mm_error = hts221_read_reg(ctx, HTS221_CTRL_REG2, &reg.byte, 1);
  *val = reg.ctrl_reg2.one_shot;

  return mm_error;
}

/**
  * @brief  temp_data_ready: [get]  Temperature data available.
  *
  * @param  hts221_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of t_da in reg STATUS_REG
  *
  */
int32_t hts221_temp_data_ready_get(hts221_ctx_t *ctx, uint8_t *val)
{
  hts221_reg_t reg;
  int32_t mm_error;

  mm_error = hts221_read_reg(ctx, HTS221_STATUS_REG, &reg.byte, 1);
  *val = reg.status_reg.t_da;

  return mm_error;
}

/**
  * @brief  hum_data_ready: [get]  Humidity data available.
  *
  * @param  hts221_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of h_da in reg STATUS_REG
  *
  */
int32_t hts221_hum_data_ready_get(hts221_ctx_t *ctx, uint8_t *val)
{
  hts221_reg_t reg;
  int32_t mm_error;

  mm_error = hts221_read_reg(ctx, HTS221_STATUS_REG, &reg.byte, 1);
  *val = reg.status_reg.h_da;

  return mm_error;
}

/**
  * @brief  humidity_raw: [get]  Humidity output value
  *
  * @param  hts221_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t * : buffer that stores data read
  *
  */
int32_t hts221_humidity_raw_get(hts221_ctx_t *ctx, uint8_t *buff)
{
  return hts221_read_reg(ctx, HTS221_HUMIDITY_OUT_L, buff, 2);
}

/**
  * @brief  temperature_raw: [get]  Temperature output value
  *
  * @param  hts221_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t * : buffer that stores data read
  *
  */
int32_t hts221_temperature_raw_get(hts221_ctx_t *ctx, uint8_t *buff)
{
  return hts221_read_reg(ctx, HTS221_TEMP_OUT_L, buff, 2);
}

/**
  * @}
  */

/**
  * @addtogroup  common
  * @brief   This section group common usefull functions
  * @{
  */

/**
  * @brief  device_id: [get] DeviceWhoamI.
  *
  * @param  hts221_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t * : buffer that stores data read
  *
  */
int32_t hts221_device_id_get(hts221_ctx_t *ctx, uint8_t *buff)
{
  return hts221_read_reg(ctx, HTS221_WHO_AM_I, buff, 1);
}

/**
  * @brief  power_on: [set]  Switch device on/off
  *
  * @param  hts221_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of pd in reg CTRL_REG1
  *
  */
int32_t hts221_power_on_set(hts221_ctx_t *ctx, uint8_t val)
{
  hts221_reg_t reg;
  int32_t mm_error;

  mm_error = hts221_read_reg(ctx, HTS221_CTRL_REG1, &reg.byte, 1);
  reg.ctrl_reg1.pd = val;
  mm_error = hts221_write_reg(ctx, HTS221_CTRL_REG1, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  power_on: [get]  Switch device on/off
  *
  * @param  hts221_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of pd in reg CTRL_REG1
  *
  */
int32_t hts221_power_on_get(hts221_ctx_t *ctx, uint8_t *val)
{
  hts221_reg_t reg;
  int32_t mm_error;

  mm_error = hts221_read_reg(ctx, HTS221_CTRL_REG1, &reg.byte, 1);
  *val = reg.ctrl_reg1.pd;

  return mm_error;
}

/**
  * @brief  heater: [set]  Heater enable / disable.
  *
  * @param  hts221_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of heater in reg CTRL_REG2
  *
  */
int32_t hts221_heater_set(hts221_ctx_t *ctx, uint8_t val)
{
  hts221_reg_t reg;
  int32_t mm_error;

  mm_error = hts221_read_reg(ctx, HTS221_CTRL_REG2, &reg.byte, 1);
  reg.ctrl_reg2.heater = val;
  mm_error = hts221_write_reg(ctx, HTS221_CTRL_REG2, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  heater: [get]  Heater enable / disable.
  *
  * @param  hts221_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of heater in reg CTRL_REG2
  *
  */
int32_t hts221_heater_get(hts221_ctx_t *ctx, uint8_t *val)
{
  hts221_reg_t reg;
  int32_t mm_error;

  mm_error = hts221_read_reg(ctx, HTS221_CTRL_REG2, &reg.byte, 1);
  *val = reg.ctrl_reg2.heater;

  return mm_error;
}

/**
  * @brief  boot: [set]  Reboot memory content. Reload the calibration
  *                      parameters.
  *
  * @param  hts221_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of boot in reg CTRL_REG2
  *
  */
int32_t hts221_boot_set(hts221_ctx_t *ctx, uint8_t val)
{
  hts221_reg_t reg;
  int32_t mm_error;

  mm_error = hts221_read_reg(ctx, HTS221_CTRL_REG2, &reg.byte, 1);
  reg.ctrl_reg2.boot = val;
  mm_error = hts221_write_reg(ctx, HTS221_CTRL_REG2, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  boot: [get]  Reboot memory content. Reload the calibration
  *                      parameters.
  *
  * @param  hts221_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of boot in reg CTRL_REG2
  *
  */
int32_t hts221_boot_get(hts221_ctx_t *ctx, uint8_t *val)
{
  hts221_reg_t reg;
  int32_t mm_error;

  mm_error = hts221_read_reg(ctx, HTS221_CTRL_REG2, &reg.byte, 1);
  *val = reg.ctrl_reg2.boot;

  return mm_error;
}

/**
  * @brief  status: [get]  Info about device status
  *
  * @param  hts221_ctx_t *ctx: read / write interface definitions
  * @param  hts221_status_t: Registers STATUS_REG
  *
  */
int32_t hts221_status_get(hts221_ctx_t *ctx, hts221_status_reg_t *val)
{
  return hts221_read_reg(ctx, HTS221_STATUS_REG, (uint8_t*) val, 1);
}

/**
  * @}
  */

/**
  * @addtogroup  interrupts
  * @brief   This section group all the functions that manage interrupts
  * @{
  */

/**
  * @brief  drdy_on_int: [set]  Data-ready signal on INT_DRDY pin.
  *
  * @param  hts221_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t val: change the values of drdy in reg CTRL_REG3
  *
  */
int32_t hts221_drdy_on_int_set(hts221_ctx_t *ctx, uint8_t val)
{
  hts221_reg_t reg;
  int32_t mm_error;

  mm_error = hts221_read_reg(ctx, HTS221_CTRL_REG3, &reg.byte, 1);
  reg.ctrl_reg3.drdy = val;
  mm_error = hts221_write_reg(ctx, HTS221_CTRL_REG3, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  drdy_on_int: [get]  Data-ready signal on INT_DRDY pin.
  *
  * @param  hts221_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t: change the values of drdy in reg CTRL_REG3
  *
  */
int32_t hts221_drdy_on_int_get(hts221_ctx_t *ctx, uint8_t *val)
{
  hts221_reg_t reg;
  int32_t mm_error;

  mm_error = hts221_read_reg(ctx, HTS221_CTRL_REG3, &reg.byte, 1);
  *val = reg.ctrl_reg3.drdy;

  return mm_error;
}

/**
  * @brief  pin_mode: [set]  Push-pull/open drain selection on interrupt pads.
  *
  * @param  hts221_ctx_t *ctx: read / write interface definitions
  * @param  hts221_pp_od_t: change the values of pp_od in reg CTRL_REG3
  *
  */
int32_t hts221_pin_mode_set(hts221_ctx_t *ctx, hts221_pp_od_t val)
{
  hts221_reg_t reg;
  int32_t mm_error;

  mm_error = hts221_read_reg(ctx, HTS221_CTRL_REG3, &reg.byte, 1);
  reg.ctrl_reg3.pp_od = val;
  mm_error = hts221_write_reg(ctx, HTS221_CTRL_REG3, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  pin_mode: [get]  Push-pull/open drain selection on interrupt pads.
  *
  * @param  hts221_ctx_t *ctx: read / write interface definitions
  * @param  hts221_pp_od_t: Get the values of pp_od in reg CTRL_REG3
  *
  */
int32_t hts221_pin_mode_get(hts221_ctx_t *ctx, hts221_pp_od_t *val)
{
  hts221_reg_t reg;
  int32_t mm_error;

  mm_error = hts221_read_reg(ctx, HTS221_CTRL_REG3, &reg.byte, 1);
  *val = (hts221_pp_od_t) reg.ctrl_reg3.pp_od;

  return mm_error;
}

/**
  * @brief  int_polarity: [set]  Interrupt active-high/low.
  *
  * @param  hts221_ctx_t *ctx: read / write interface definitions
  * @param  hts221_drdy_h_l_t: change the values of drdy_h_l in reg CTRL_REG3
  *
  */
int32_t hts221_int_polarity_set(hts221_ctx_t *ctx, hts221_drdy_h_l_t val)
{
  hts221_reg_t reg;
  int32_t mm_error;

  mm_error = hts221_read_reg(ctx, HTS221_CTRL_REG3, &reg.byte, 1);
  reg.ctrl_reg3.drdy_h_l = val;
  mm_error = hts221_write_reg(ctx, HTS221_CTRL_REG3, &reg.byte, 1);

  return mm_error;
}

/**
  * @brief  int_polarity: [get]  Interrupt active-high/low.
  *
  * @param  hts221_ctx_t *ctx: read / write interface definitions
  * @param  hts221_drdy_h_l_t: Get the values of drdy_h_l in reg CTRL_REG3
  *
  */
int32_t hts221_int_polarity_get(hts221_ctx_t *ctx, hts221_drdy_h_l_t *val)
{
  hts221_reg_t reg;
  int32_t mm_error;

  mm_error = hts221_read_reg(ctx, HTS221_CTRL_REG3, &reg.byte, 1);
  *val = (hts221_drdy_h_l_t) reg.ctrl_reg3.drdy_h_l;

  return mm_error;
}

/**
  * @}
  */

/**
  * @addtogroup  calibration
  * @brief   This section group all the calibration coefficients need
  *          for reading data
  * @{
  */

/**
  * @brief  hum_rh_point_0: [get]  First calibration point for Rh Humidity.
  *
  * @param  hts221_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t * : buffer that stores data read
  *
  */
int32_t hts221_hum_rh_point_0_get(hts221_ctx_t *ctx, uint8_t *buff)
{
  int32_t mm_error;

  mm_error = hts221_read_reg(ctx, HTS221_H0_RH_X2, buff, 1);
  *buff = (((*buff) >> 1) & 0x7FFF);
  
  return mm_error;
}

/**
  * @brief  hum_rh_point_1: [get]  Second calibration point for Rh Humidity.
  *
  * @param  hts221_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t * : buffer that stores data read
  *
  */
int32_t hts221_hum_rh_point_1_get(hts221_ctx_t *ctx, uint8_t *buff)
{
  int32_t mm_error;

  mm_error = hts221_read_reg(ctx, HTS221_H1_RH_X2, buff, 1);
  *buff = (((*buff) >> 1) & 0x7FFF);
  
  return mm_error;
}

/**
  * @brief  temp_deg_point_0: [get]  First calibration point for
  *                                  degC temperature.
  *
  * @param  hts221_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t * : buffer that stores data read
  *
  */
int32_t hts221_temp_deg_point_0_get(hts221_ctx_t *ctx, uint8_t *buff)
{
  hts221_reg_t reg;
  int32_t mm_error;
  axis1bit16_t coeff;
    
  mm_error = hts221_read_reg(ctx, HTS221_T0_DEGC_X8, coeff.u8bit, 1);
  mm_error = hts221_read_reg(ctx, HTS221_T1_T0_MSB, &reg.byte, 1);
  coeff.u8bit[1] = reg.t1_t0_msb.t0_msb;
  coeff.i16bit = coeff.i16bit >> 3;
  *(buff)   = coeff.u8bit[0];
  
  return mm_error;
}

/**
  * @brief  temp_deg_point_1: [get]  Second calibration point for
  *                                  degC temperature.
  *
  * @param  hts221_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t * : buffer that stores data read
  *
  */
int32_t hts221_temp_deg_point_1_get(hts221_ctx_t *ctx, uint8_t *buff)
{
  hts221_reg_t reg;
  int32_t mm_error;
  axis1bit16_t coeff;
  
  mm_error = hts221_read_reg(ctx, HTS221_T1_DEGC_X8, coeff.u8bit, 1);
  mm_error = hts221_read_reg(ctx, HTS221_T1_T0_MSB, &reg.byte, 1);
  coeff.u8bit[1] = reg.t1_t0_msb.t1_msb;
  coeff.i16bit = coeff.i16bit >> 3;
  *(buff)   = coeff.u8bit[0];
  return mm_error;
}

/**
  * @brief  hum_adc_point_0: [get]  First calibration point for
  *                                 humidity in LSB.
  *
  * @param  hts221_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t * : buffer that stores data read
  *
  */
int32_t hts221_hum_adc_point_0_get(hts221_ctx_t *ctx, uint8_t *buff)
{
  return hts221_read_reg(ctx, HTS221_H0_T0_OUT_L, buff, 2);
}

/**
  * @brief  hum_adc_point_1: [get]  Second calibration point for
  *                                 humidity in LSB.
  *
  * @param  hts221_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t * : buffer that stores data read
  *
  */
int32_t hts221_hum_adc_point_1_get(hts221_ctx_t *ctx, uint8_t *buff)
{
  return hts221_read_reg(ctx, HTS221_H1_T0_OUT_L, buff, 2);
}

/**
  * @brief  temp_adc_point_0: [get]  First calibration point for
  *                                  temperature in LSB.
  *
  * @param  hts221_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t * : buffer that stores data read
  *
  */
int32_t hts221_temp_adc_point_0_get(hts221_ctx_t *ctx, uint8_t *buff)
{
  return hts221_read_reg(ctx, HTS221_T0_OUT_L, buff, 2);
}

/**
  * @brief  temp_adc_point_1: [get]  Second calibration point for
  *                                  temperature in LSB.
  *
  * @param  hts221_ctx_t *ctx: read / write interface definitions
  * @param  uint8_t * : buffer that stores data read
  *
  */
int32_t hts221_temp_adc_point_1_get(hts221_ctx_t *ctx, uint8_t *buff)
{
  return hts221_read_reg(ctx, HTS221_T1_OUT_L, buff, 2);
}

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/