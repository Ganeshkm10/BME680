/*
 * BME680.c
 *
 *  Created on: 03-Mar-2023
 *      Author: Windows
 */

/*
 * bme680.c
 *
 *  Created on: 13-Feb-2023
 *      Author: Windows
 */


#include <stdio.h>
#include <stdint.h>
#include "em_device.h"
#include "em_i2c.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "bme680.h"



I2C_TypeDef *i2c = I2C0;

void i2c_init(void)
{
  CMU_ClockEnable(cmuClock_I2C0, true);
  I2C_Init_TypeDef i2cInit = I2C_INIT_DEFAULT;
  i2cInit.freq = I2C_FREQ_STANDARD_MAX;
  I2C_Init(i2c, &i2cInit);
}

void i2c_write_reg(uint8_t reg_addr, uint8_t value)
{
  I2C_TransferSeq_TypeDef seq;
  seq.addr = BME680_I2C_ADDRESS;
  seq.flags = I2C_FLAG_WRITE;
  seq.buf[0].data = &reg_addr;
  seq.buf[0].len = 1;
  seq.buf[1].data = &value;
  seq.buf[1].len = 1;

  I2C_TransferReturn_TypeDef status = I2C_TransferInit(i2c, &seq);
  while (status == i2cTransferInProgress) {

      status = I2C_Transfer(I2C0);
//    printf("Error writing to BME680\n = status %d \n\r",status);
    if(status == i2cTransferDone)
      {
//        printf(" I2C transfer done \n\r");
      }

  }
}

uint8_t i2c_read_reg(uint8_t reg_addr)
{
  I2C_TransferReturn_TypeDef status;
  I2C_TransferSeq_TypeDef seq;
  seq.addr = BME680_I2C_ADDRESS;
  seq.flags = I2C_FLAG_WRITE_READ;
  seq.buf[0].data = &reg_addr;
  seq.buf[0].len = 1;
  seq.buf[1].data = &reg_addr;
  seq.buf[1].len = 1;
  uint8_t value;
  seq.buf[1].data = &value;
  seq.buf[1].len = 1;

  status = I2C_TransferInit(i2c, &seq);
  while (status == i2cTransferInProgress) {

       status = I2C_Transfer(I2C0);
//     printf("Error reading from BME680\n = status %d \n\r",status);
     if(status == i2cTransferDone)
       {
//         printf(" I2C read done \n\r");
       }
  }
  return value;
}

int16_t read_temp(void)
{
  uint8_t msb = i2c_read_reg(BME680_TEMP_MSB_ADDR);
  uint8_t lsb = i2c_read_reg(BME680_TEMP_LSB_ADDR);
  uint8_t xlsb = i2c_read_reg(BME680_TEMP_XLSB_ADDR);
  int16_t temp = (msb << 12) | (lsb << 4) | (xlsb >> 4);
//  uint16_t par_t1 =
//  int16_t par_t2 =
//  int8_t par_t3 =
  return temp;
}

int32_t read_pressure(void)
{
  uint8_t msb = i2c_read_reg(BME680_PRESS_MSB_ADDR);
  uint8_t lsb = i2c_read_reg(BME680_PRESS_LSB_ADDR);
  uint8_t xlsb = i2c_read_reg(BME680_PRESS_XLSB_ADDR);
  int32_t pressure = (msb << 12) | (lsb << 4) | (xlsb >> 4);

  return pressure;
}

int16_t read_humidity(void)
{
  uint8_t msb = i2c_read_reg(BME680_HUM_MSB_ADDR);
  uint8_t lsb = i2c_read_reg(BME680_HUM_LSB_ADDR);
  int16_t humidity = (msb << 8) | lsb;

  return humidity;
}

