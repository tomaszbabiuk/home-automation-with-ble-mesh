#include <stdio.h>
#include "esp_log.h"
#include "board.h"
#include "rgb_led.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"


void board_rgb_led_control(rgb_led_color_t color) {
    rgb_led_control(color);
}

void board_init(input_signal_f input_callback)
{
    digital_input_init(input_callback);
    rgb_led_enable();
}
