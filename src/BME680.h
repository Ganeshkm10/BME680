/*
 * BME680.h
 *
 *  Created on: 03-Mar-2023
 *      Author: Windows
 */

#ifndef SRC_BME680_H_
#define SRC_BME680_H_

#include <stdint.h>

#define BME680_I2C_ADDRESS (0x76 << 1)
#define BME680_TEMP_MSB_ADDR 0x22
#define BME680_TEMP_LSB_ADDR 0x23
#define BME680_TEMP_XLSB_ADDR 0x24
#define BME680_PRESS_MSB_ADDR 0x1F
#define BME680_PRESS_LSB_ADDR 0x20
#define BME680_PRESS_XLSB_ADDR 0x21
#define BME680_HUM_MSB_ADDR 0x25
#define BME680_HUM_LSB_ADDR 0x26

#define BME680_CTRL_HUM_ADDR 0x72
#define BME680_CTRL_MEAS_ADDR 0x74
#define BME680_CONFIG_ADDR 0x75
#define BME680_RESET_ADDR 0xE0

void i2c_init(void);
void i2c_write_reg(uint8_t reg_addr, uint8_t value);
uint8_t i2c_read_reg(uint8_t reg_addr);
int16_t read_temp(void);
int32_t read_pressure(void);
int16_t read_humidity(void);

#endif /* SRC_BME680_H_ */
