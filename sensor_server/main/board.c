#include <stdio.h>
#include "esp_log.h"
#include "board.h"
#include "rgb_led.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

gpio_config_t io_conf = {
    .intr_type = GPIO_INTR_NEGEDGE,
    .pin_bit_mask = GPIO_INPUT_PIN_SEL,
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = 1
};

QueueHandle_t gpio_evt_queue;


void board_rgb_led_control(rgb_led_color_t color) {
    rgb_led_control(color);
}


static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void gpio_task_example(void* arg)
{
    uint32_t io_num;
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            printf("Button pressed\n");
        }
    }
}

void button_init() {
    gpio_config(&io_conf);
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);

    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(GPIO_INPUT_PIN_SEL, gpio_isr_handler, (void*) GPIO_INPUT_PIN_SEL);
}

void board_init(void)
{
    rgb_led_enable();
    button_init();
}
