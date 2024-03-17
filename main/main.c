#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/i2c.h"

#define I2C_MASTER_SCL_IO    4    /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO    5    /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM        I2C_NUM_0    /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ    100000     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0   /*!< I2C master do not need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0   /*!< I2C master do not need buffer */
#define MPU6050_SENSOR_ADDR                 0x68 /*!< slave address for MPU6050 sensor */
#define MPU6050_ACCEL_XOUT_H                0x3B /*!< Register for ACCEL_XOUT_H */
#define MPU6050_PWR_MGMT_1                  0x6B /*!< Register for PWR_MGMT_1 */
#define WRITE_BIT                           I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT                            I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN                        0x1     /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS                       0x0     /*!< I2C master will not check ack from slave */
#define ACK_VAL                             0x0         /*!< I2C ack value */
#define NACK_VAL                            0x1         /*!< I2C nack value */

static const char *TAG = "MPU6050";

// i2c master initialization
void i2c_master_init()
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

//Generic function to write a byte to MPU6050
esp_err_t mpu6050_write_byte(uint8_t regAddr, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MPU6050_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, regAddr, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Generic function to read multiple bytes from MPU6050
esp_err_t mpu6050_read_bytes(uint8_t regAddr, uint8_t *data, uint16_t length)
{
    if (length == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MPU6050_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, regAddr, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MPU6050_SENSOR_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
    if (length > 1) {
        i2c_master_read(cmd, data, length - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data + length - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Task to read data from MPU6050

void mpu6050_task(void *arg)
{
    uint8_t sensor_data[14];
    i2c_master_init();
    mpu6050_write_byte(MPU6050_PWR_MGMT_1, 0); // Wake up MPU6050

    while (1)
    {
        mpu6050_read_bytes(MPU6050_ACCEL_XOUT_H, sensor_data, 14);
        ESP_LOGI(TAG, "Acceleration: X:%d, Y:%d, Z:%d", sensor_data[0] << 8 | sensor_data[1], sensor_data[2] << 8 | sensor_data[3], sensor_data[4] << 8 | sensor_data[5]);
        ESP_LOGI(TAG, "Gyroscope: X:%d, Y:%d, Z:%d", sensor_data[8] << 8 | sensor_data[9], sensor_data[10] << 8 | sensor_data[11], sensor_data[12] << 8 | sensor_data[13]);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void)
{
    xTaskCreate(mpu6050_task, "mpu6050_task", 2048, NULL, 5, NULL);
}
