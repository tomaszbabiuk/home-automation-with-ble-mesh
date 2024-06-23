#ifndef _SENSORS_H_
#define _SENSORS_H_

#include "esp_err.h"

#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define BH1750_SENSOR_ADDR  0x23
#define BH1750_CMD_START    0x23
#define SHT30_SENSOR_ADDR   0x45
#define SHT30_CMD_MEASURE_HIGH_REPETABILITY {0x2C, 0x06}

#define I2C_MASTER_SCL_IO 10
#define I2C_MASTER_SDA_IO 8
#define I2C_MASTER_NUM 0
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_TX_BUF_DISABLE 0                  
#define I2C_MASTER_RX_BUF_DISABLE 0   

esp_err_t i2c_sensors_init(void);
esp_err_t i2c_sensors_read_sht30(float *tempC, float *humidity);
esp_err_t i2c_sensors_read_bh1750(float *luminocity);
esp_err_t i2c_sensors_read_sgp30(uint16_t *tvoc, uint16_t *eco2);

#endif