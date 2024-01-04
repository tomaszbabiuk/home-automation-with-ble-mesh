/* board.c - Board-specific hooks */

/*
 * SPDX-FileCopyrightText: 2017 Intel Corporation
 * SPDX-FileContributor: 2018-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "board.h"

#define TAG "BOARD"

struct _relay_state relay_state[8] = {
    { RELAY_OFF, RELAY_OFF, R0, "Relay 0" },
    { RELAY_OFF, RELAY_OFF, R1, "Relay 1" },
    { RELAY_OFF, RELAY_OFF, R2, "Relay 2" },
    { RELAY_OFF, RELAY_OFF, R3, "Relay 3" },
    { RELAY_OFF, RELAY_OFF, R4, "Relay 4" },
    { RELAY_OFF, RELAY_OFF, R5, "Relay 5" },
    { RELAY_OFF, RELAY_OFF, R6, "Relay 6" },
    { RELAY_OFF, RELAY_OFF, R7, "Relay 7" },
};

void board_led_operation(uint8_t pin, uint8_t onoff)
{
    for (int i = 0; i < 8; i++) {
        if (relay_state[i].pin != pin) {
            continue;
        }
        if (onoff == relay_state[i].previous) {
            ESP_LOGW(TAG, "relay %s is already %s",
                     relay_state[i].name, (onoff ? "on" : "off"));
            return;
        }
        gpio_set_level(pin, onoff);
        relay_state[i].previous = onoff;
        return;
    }

    ESP_LOGE(TAG, "RELAY is not found!");
}

static void board_relay_init(void)
{
    for (int i = 0; i < 8; i++) { 
        gpio_reset_pin(relay_state[i].pin);
        gpio_set_direction(relay_state[i].pin, GPIO_MODE_OUTPUT);
        gpio_set_level(relay_state[i].pin, RELAY_OFF);
        relay_state[i].previous = RELAY_OFF;
    }
}

void board_init(void)
{
    board_relay_init();
}
