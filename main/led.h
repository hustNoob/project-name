#include "touwenjian.h"

#define LED_PIN GPIO_NUM_2 // 设置LED连接的GPIO口

void led_light() {
    gpio_reset_pin(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT); // 将LED引脚设置为输出模式
    gpio_set_level(LED_PIN, 1); // 点亮LED
}
