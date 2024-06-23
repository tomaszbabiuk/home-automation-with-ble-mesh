#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "sensors.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_err.h"
#include "sgp30.h"

#define TAG "sensors"

#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */

static sgp30_dev_t main_sgp30_sensor;

esp_err_t i2c_sensors_read_bh1750(float *luminocity)
{
    uint8_t command[] = { BH1750_CMD_START };
    uint8_t data[2];
	esp_err_t ret = i2c_master_write_read_device(I2C_MASTER_NUM, BH1750_SENSOR_ADDR, command, 1, data, 2, pdMS_TO_TICKS(1000));

    *luminocity = (data[0] << 8 | data[1]) / 1.2;

    return ret;
}

static uint8_t calculate_crc8(uint8_t *data, int len)
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

esp_err_t i2c_sensors_read_sht30(float *tempC, float *humidity)
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
			*tempC = (-45.0f + 175.0f * ((float)val / 65535.0f));
		}

		if (calculate_crc8(data+3, 2) != data[5])
		{
            return ESP_ERR_INVALID_CRC;
		} else {
			val = data[3] << 8;
			val += data[4];
			*humidity = 100.0f * ((float)val / 65535.0f);
		}

    return ret;
}

int8_t main_i2c_write(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    int8_t ret = 0; /* Return 0 for Success, non-zero for failure */

    uint8_t chip_addr = *(uint8_t*)intf_ptr;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (chip_addr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    
    if (reg_addr != 0xFF) {
        i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
    }

    i2c_master_write(cmd, reg_data, len, ACK_CHECK_EN);
    i2c_master_stop(cmd);

    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    
    i2c_cmd_link_delete(cmd);
    
    return ret;
}

int8_t main_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) { // *intf_ptr = dev->intf_ptr
    int8_t ret = 0; /* Return 0 for Success, non-zero for failure */

    if (len == 0) {
        return ESP_OK;
    }

    uint8_t chip_addr = *(uint8_t*)intf_ptr;
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    
    if (reg_addr != 0xff) {
        i2c_master_write_byte(cmd, (chip_addr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
        i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
        i2c_master_start(cmd);
    }
    
    i2c_master_write_byte(cmd, (chip_addr << 1) | I2C_MASTER_READ, ACK_CHECK_EN);

    if (len > 1) {
        i2c_master_read(cmd, reg_data, len - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, reg_data + len - 1, NACK_VAL);
    i2c_master_stop(cmd);

    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    
    i2c_cmd_link_delete(cmd);
    
    return ret;
}

static esp_err_t sensors_sgp30_init() {
    sgp30_init(&main_sgp30_sensor, (sgp30_read_fptr_t)main_i2c_read, (sgp30_write_fptr_t)main_i2c_write);

    // SGP30 needs to be read every 1s and sends TVOC = 400 14 times when initializing
    for (int i = 0; i < 15; i++) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        sgp30_IAQ_measure(&main_sgp30_sensor);
        ESP_LOGI(TAG, "SGP30 Calibrating... TVOC: %d,  eCO2: %d",  main_sgp30_sensor.TVOC, main_sgp30_sensor.eCO2);
    }

    uint16_t eco2_baseline, tvoc_baseline;
    sgp30_get_IAQ_baseline(&main_sgp30_sensor, &eco2_baseline, &tvoc_baseline);
    ESP_LOGI(TAG, "SGP30 BASELINES - TVOC: %d,  eCO2: %d",  tvoc_baseline, eco2_baseline);

    return 0;
}

esp_err_t i2c_sensors_read_sgp30(uint16_t *tvoc, uint16_t *eco2) {
	//https://github.com/co-env/esp32_SGP30
    sgp30_IAQ_measure(&main_sgp30_sensor);

    *tvoc = main_sgp30_sensor.TVOC;
    *eco2 = main_sgp30_sensor.eCO2;

    return 0;
}

esp_err_t i2c_sensors_init(void)
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
    err = i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    if (err != ESP_OK) {
        return err;
    }

    return sensors_sgp30_init();
}