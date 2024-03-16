#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include <stdio.h>

#define TX_PIN GPIO_NUM_4
#define RX_PIN GPIO_NUM_5
const uart_port_t uart_num = UART_NUM_0;
const char *test_str = "hello world\n";

#define SDA_PIN GPIO_NUM_5
#define SCL_PIN GPIO_NUM_4
#define I2C_HZ 1000000
#define I2C_MASTER_NUM I2C_NUM_0
#define READ_ADD 0x43 //陀螺仪的地址
#define I2C_MPU_ADDR 0x68
#define DATA_LENGTH 100
#define DATA_WR I2C_MASTER_WRITE
#define DARA_RD I2C_MASTER_READ
const uint8_t mpu6050_address; // address
const bool ACK_EN = true;      // Enable ACK signal
const TickType_t time_out = 1000 / portTICK_PERIOD_MS;
static const char *TAG = "mpu6050";


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
    uart_write_bytes(uart_num, test_str, strlen(test_str));
}



void app_main()
{
    // I2C总线的配置
    i2c_master_bus_config_t i2c_mst_conf = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_MASTER_NUM,
        .scl_io_num = SCL_PIN,
        .sda_io_num = SDA_PIN,
        .glitch_ignore_cnt = 0, // if the glitch period on the line is less than 7, it can be filtered out
        .flags.enable_internal_pullup = true,
    };
    // 创建I2C总线句柄，以此管控i2c总线上通信
    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_conf, &bus_handle)); // 创建一个新的i2c主总线,将其句柄存储在bus_handle中
    // 配置MPU6050
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = I2C_MPU_ADDR, // mpu6050的i2c地址
        .scl_speed_hz = 100000,
    };
    // 添加MPU6050设备到I2C总线
    i2c_master_dev_handle_t dev_handle;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));
    const uint8_t READ_ADDR[1] = {0x3B};
    const uint8_t add_r[14] = {0};
    while (1){
    //先发送需要用的寄存器地址
    i2c_master_transmit(dev_handle, (uint8_t *)READ_ADDR, 1, 10000);
    //读数据
    i2c_master_receive(dev_handle, (uint8_t *)add_r, 14, 10000);
    
        ESP_LOGI(TAG, "Acceleration: X:%d, Y:%d, Z:%d", add_r[0] << 8 | add_r[1], add_r[2] << 8 | add_r[3], add_r[4] << 8 | add_r[5]);
        ESP_LOGI(TAG, "Gyroscope: X:%d, Y:%d, Z:%d", add_r[8] << 8 | add_r[9], add_r[10] << 8 | add_r[11], add_r[12] << 8 | add_r[13]);
        //i2c_master_receive(dev_handle, (uint8_t *)add_r, 14, 10000);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
        // uart_send();idf.py -p com5 flash monitor
    }
