#pragma once

#include <stdint.h>
#include "driver/rmt_tx.h"
#include "led_strip_encoder.h"

#define RMT_LED_STRIP_RESOLUTION_HZ 10000000
#define RMT_LED_STRIP_GPIO_NUM      7

#define EXAMPLE_LED_NUMBERS         1
#define EXAMPLE_CHASE_SPEED_MS      10

#if defined(CONFIG_BLE_MESH_WEMOS_C3_MINI)
typedef enum rgb_led_color {
  RED = 0x001100,
  GREEN = 0x000011,
  BLUE = 0x110000,
  ORANGE = 0x004411,
  INTENSIVE_WHITE = 0xFFFFFF
} rgb_led_color_t;
#endif

#if defined(CONFIG_BLE_MESH_WEMOS_C3_PICO)
typedef enum rgb_led_color {
  RED = 0x000011,
  GREEN = 0x001100,
  BLUE = 0x110000,
  ORANGE = 0x001144,
  INTENSIVE_WHITE = 0xFFFFFF
} rgb_led_color_t;
#endif

void rgb_led_enable();
void rgb_led_control(rgb_led_color_t color);