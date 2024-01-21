#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "sensors.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_err.h"

esp_err_t i2c_master_sensor_bh1750(uint32_t *luminocity)
{
    uint8_t command[] = { BH1750_CMD_START };
    uint8_t data[2];
	esp_err_t ret = i2c_master_write_read_device(I2C_MASTER_NUM, BH1750_SENSOR_ADDR, command, 1, data, 2, pdMS_TO_TICKS(1000));

    *luminocity = (data[0] << 8 | data[1]) / 1.2 * 100;

    return ret;
}

uint8_t calculate_crc8(uint8_t *data, int len)
{
	const uint8_t	POLYNOMIAL = 0x31;
	uint8_t			crc = 0xFF;
	int				i, j;
	
	for (i = 0; i < len; ++i)
	{
		crc ^= *data++;
		for (j = 0; j < 8; ++j)
			crc = (crc & 0x80) ? (crc << 1) ^ POLYNOMIAL : (crc << 1);
	}
	return crc;
}

esp_err_t i2c_master_sensor_sht30(float *tempC, float *humidity)
{
    uint16_t val;
    uint8_t command[] = SHT30_CMD_MEASURE_HIGH_REPETABILITY;
    uint8_t data[6];
	esp_err_t ret = i2c_master_write_read_device(I2C_MASTER_NUM, SHT30_SENSOR_ADDR, command, 2, data, 6, pdMS_TO_TICKS(1000));
	if (ret != ESP_OK) {
        return ret;
    }

    if (data[0] == 0xff && data[1] == 0xff && data[2] == 0xff && data[3] == 0xff && data[4] == 0xff && data[5] == 0xff)
		{
			return ESP_ERR_INVALID_RESPONSE;		
		}

		if (calculate_crc8(data, 2) != data[2])
		{
            return ESP_ERR_INVALID_CRC;
		} else {
			val = data[0] << 8;
			val += data[1];
			*tempC = -45.0f + 175.0f * ((float)val / 65535.0f);
		}

		if (calculate_crc8(data+3, 2) != data[5])
		{
            return ESP_ERR_INVALID_CRC;
		} else {
			val = data[0] << 8;
			val += data[1];
			*humidity = 100.0f * ((float)val / 65535.0f);
		}

    return ret;
    }

esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    esp_err_t err = i2c_param_config(i2c_master_port, &conf);
    if (err != ESP_OK) {
        return err;
    }
    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}