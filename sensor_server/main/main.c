/* main.c - Application main entry point */

/*
 * SPDX-FileCopyrightText: 2017 Intel Corporation
 * SPDX-FileContributor: 2018-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "esp_log.h"
#include "nvs_flash.h"
#include "mesh_app.h"

#include "ux.h"
#include "sensors.h"
#include "ble_mesh_example_init.h"
#include "board.h"

#define TAG "MAIN"

SemaphoreHandle_t print_mux = NULL;

/* I2C refresh interval */
#define DELAY_TIME_BETWEEN_ITEMS_MS 5000

static void i2c_process(void *arg)
{
    ESP_LOGE(TAG, "I2C process");

    int ret;
    float tempCF, humidityF, luminocityF;
    uint16_t tvoc, eco2;
    
    while (1) {
        ret = i2c_sensors_read_bh1750(&luminocityF);
        xSemaphoreTake(print_mux, portMAX_DELAY);
        if (ret == ESP_ERR_TIMEOUT) {
            ESP_LOGE(TAG, "I2C Timeout");
        } else if (ret == ESP_OK) {
            mesh_app_update_luminocity(luminocityF);
            ESP_LOGI(TAG, "LIGHT: %.02f [Lux]\n", luminocityF);
        } else {
            ESP_LOGW(TAG, "%s: No ack, sensor not connected...skip...", esp_err_to_name(ret));
        }

        ret = i2c_sensors_read_sht30(&tempCF, &humidityF);
        if (ret == ESP_ERR_TIMEOUT) {
            ESP_LOGE(TAG, "I2C Timeout");
        } else if (ret == ESP_OK) {
            mesh_app_update_temperature(tempCF);
            mesh_app_update_humidity(humidityF);
            ESP_LOGI(TAG, "SHT30 temp=%.2fC, hum=%.2f%%", tempCF, humidityF);
        } else {
            ESP_LOGW(TAG, "%s: No ack, sensor not connected...skip...", esp_err_to_name(ret));
        }

        ret = i2c_sensors_read_sgp30(&tvoc, &eco2);
        if (ret == ESP_ERR_TIMEOUT) {
            ESP_LOGE(TAG, "I2C Timeout");
        } else if (ret == ESP_OK) {
            mesh_app_update_tvoc(tvoc);
            mesh_app_update_eco2(eco2);
            ESP_LOGI(TAG, "SGP30 tvoc=%d, eCO2=%d",  tvoc, eco2);
        } else {
            ESP_LOGW(TAG, "%s: No ack, sensor not connected...skip...", esp_err_to_name(ret));
        }

        xSemaphoreGive(print_mux);
        vTaskDelay(DELAY_TIME_BETWEEN_ITEMS_MS / portTICK_PERIOD_MS);
    }
    vSemaphoreDelete(print_mux);
    vTaskDelete(NULL);
}

void press_callback(int how_long_ns) {
    int how_long_s = how_long_ns / 1000000;
    if (how_long_s > 5) {
        ESP_LOGI(TAG, "Resetting mesh initiative requested");
        esp_ble_mesh_node_local_reset();
        ux_signal_provisioning_state(esp_ble_mesh_node_is_provisioned());
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        esp_restart();
    } else if (how_long_ns == 0) {
        ESP_LOGI(TAG, "Resetting mesh initiative started");
        ux_signal_reset_initiative_started();

        mesh_app_publish_sensors_data();
    } else {
        ESP_LOGI(TAG, "Resetting mesh initiative given up");
        ux_signal_provisioning_state(esp_ble_mesh_node_is_provisioned());
    }
}

void provisioning_complete() {
    ux_signal_provisioned(esp_ble_mesh_node_is_provisioned());
}

void attention(bool on) {
    if (on) {
        ux_attention();
    } else {
        ux_signal_provisioning_state(esp_ble_mesh_node_is_provisioned());
    }
}

void app_main(void)
{
    ux_init(press_callback);

    print_mux = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK(i2c_sensors_init());
    xTaskCreate(i2c_process, "i2c_process", 1024 * 5, (void *)0, 10, NULL);
    vTaskDelay(5000 / portTICK_PERIOD_MS);

    esp_err_t err;

    ESP_LOGI(TAG, "Initializing mesh ...");

    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    err = bluetooth_init();
    if (err) {
        ESP_LOGE(TAG, "esp32_bluetooth_init failed (err %d)", err);
        return;
    }

    /* Initialize the Bluetooth Mesh Subsystem */
    err = mesh_app_init(provisioning_complete, attention);
    if (err) {
        ESP_LOGE(TAG, "Bluetooth mesh init failed (err %d)", err);
    }

    bt_mesh_set_device_name("SENSOR-SERVER");

    ux_signal_provisioning_state(esp_ble_mesh_node_is_provisioned());
}