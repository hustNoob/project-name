#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "esp_err.h"

/*#define TX_PIN (GPIO_NUM_4)
#define RX_PIN (GPIO_NUM_5)*/

void init()
{
    const uart_port_t uart_num = UART_NUM_2;
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_APB,
    };
    // 配置UART参数
    uart_param_config(uart_num, &uart_config);
    // 设置UART引脚
    uart_set_pin(uart_num, 4, 5, 18, 19);
    // 安装UART驱动程序，设置数据缓冲区，并获取事件队列
    const int uart_buffer_size = (1024 * 2);
    QueueHandle_t uart_queue;
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_2, uart_buffer_size, uart_buffer_size, 10, &uart_queue, 0));
}

void app_main()
{
    init();
    // 发送数据
    const char *data = "hello world\n";
    uart_write_bytes(UART_NUM_2, (const char *)data, strlen(data));
    // 检查是否有错误码返回
    int len = uart_write_bytes(UART_NUM_2, "hello world", 11);
    if (len < 0)
    {
        printf("UART发送失败，错误码：%d\n", len);
    }
}
