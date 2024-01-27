#include <stdio.h>
#include "esp_log.h"
#include "board.h"
#include "rgb_led.h"

void board_rgb_led_control(rgb_led_color_t color) {
    rgb_led_control(color);
}

void board_init(void)
{
    rgb_led_enable();
}
