// GPIO demo for the esp32sim board UI. Pin numbers are chip GPIOs; on a
// Seeed XIAO ESP32C3 they are D0, D1, D2, D3 and D10; on the Espressif
// DevKits they are printed on the board.
//
//   GPIO2   output  blinks once per second
//   GPIO10  output  blinks twice per second
//   GPIO3   output  follows GPIO4 (inverted: pressing a button lights it)
//   GPIO4   input   pull-up, "button" (drive it low in the UI)
//   GPIO5   input   pull-down, interrupt on any edge, prints changes
//   GPIO9   input   BOOT button, interrupt on falling edge
#include <stdio.h>

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#define LED_SLOW   GPIO_NUM_2
#define LED_FAST   GPIO_NUM_10
#define LED_FOLLOW GPIO_NUM_3
#define BUTTON     GPIO_NUM_4
#define SENSOR     GPIO_NUM_5
#define BOOT_BTN   GPIO_NUM_9

static QueueHandle_t events;

static void IRAM_ATTR on_edge(void* arg)
{
    uint32_t pin = (uint32_t)arg;
    xQueueSendFromISR(events, &pin, NULL);
}

static void event_task(void* arg)
{
    uint32_t pin;
    for (;;) {
        if (xQueueReceive(events, &pin, portMAX_DELAY)) {
            if (pin == BOOT_BTN) printf("BOOT button pressed\n");
            else printf("GPIO%lu changed, now %d\n", (unsigned long)pin, gpio_get_level(pin));
        }
    }
}

void app_main(void)
{
    gpio_config_t out = {
        .pin_bit_mask = (1ULL << LED_SLOW) | (1ULL << LED_FAST) | (1ULL << LED_FOLLOW),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&out);

    gpio_config_t button = {
        .pin_bit_mask = 1ULL << BUTTON,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&button);

    gpio_config_t sensor = {
        .pin_bit_mask = 1ULL << SENSOR,
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    gpio_config(&sensor);

    gpio_config_t boot = {
        .pin_bit_mask = 1ULL << BOOT_BTN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&boot);

    events = xQueueCreate(16, sizeof(uint32_t));
    xTaskCreate(event_task, "events", 3072, NULL, 10, NULL);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(SENSOR, on_edge, (void*)SENSOR);
    gpio_isr_handler_add(BOOT_BTN, on_edge, (void*)BOOT_BTN);

    printf("gpio_demo: GPIO2/GPIO10 blink, GPIO3 follows button GPIO4, GPIO5 and BOOT report changes\n");
    int tick = 0, last_button = -1;
    for (;;) {
        gpio_set_level(LED_FAST, tick & 1);
        gpio_set_level(LED_SLOW, (tick >> 1) & 1);
        for (int i = 0; i < 25; ++i) {  // 250 ms, polling the button every 10 ms
            int b = gpio_get_level(BUTTON);
            gpio_set_level(LED_FOLLOW, !b);
            if (b != last_button) {
                printf("button GPIO4 %s\n", b ? "released" : "pressed");
                last_button = b;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        ++tick;
    }
}
