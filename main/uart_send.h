#include "touwenjian.h"

#define TX_PIN GPIO_NUM_4
#define RX_PIN GPIO_NUM_5
const uart_port_t uart_num = UART_NUM_0;
const char *test_str = "hello world\n";

void uart_send()
{
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
    };
    // 配置UART参数
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    // 设置UART引脚
    ESP_ERROR_CHECK(uart_set_pin(uart_num, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    // 安装UART驱动程序，设置数据缓冲区，并获取事件队列
    const int uart_buffer_size = (1024 * 2);
    QueueHandle_t uart_queue;
    ESP_ERROR_CHECK(uart_driver_install(uart_num, uart_buffer_size, uart_buffer_size, 10, &uart_queue, 0));
    // send message
}

void uart_task(void *arg){
    uart_send();
    while (1)
    {
        uart_write_bytes(uart_num, test_str, strlen(test_str));
    }
    
}