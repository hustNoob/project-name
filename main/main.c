#include "touwenjian.h"
#include "i2c_mpu6050.h"
#include "uart_send.h"
#include "wifi.h"

void app_main(void)
{
    //xTaskCreate(mpu6050_task, "mpu6050_task", 2048, NULL, 5, NULL);
    //xTaskCreate(uart_task, "uart_task", 2048, NULL, 5, NULL);
    SAT_SET();
}
