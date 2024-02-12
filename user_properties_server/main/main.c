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
#include "ble_mesh_example_init.h"

#define TAG "MAIN"

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
    esp_err_t err;

    ESP_LOGI(TAG, "Initializing mesh ...");

    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ux_init(press_callback);

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

    bt_mesh_set_device_name("PROP-SERVER");

    ux_signal_provisioning_state(esp_ble_mesh_node_is_provisioned());

    while (1) {
        vTaskDelay(10000 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, ".");
    }
}