/* board.h - Board-specific hooks */

/*
 * SPDX-FileCopyrightText: 2017 Intel Corporation
 * SPDX-FileContributor: 2018-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _BOARD_H_
#define _BOARD_H_

#include "rgb_led.h"
#include "digital_input.h"

#define GPIO_BUTTON_IO      9
#define GPIO_INPUT_PIN_SEL  (1ULL<<GPIO_BUTTON_IO)
#define ESP_INTR_FLAG_DEFAULT 0

void board_rgb_led_control(rgb_led_color_t color);

void board_init(input_signal_f input_callback);

#endif
