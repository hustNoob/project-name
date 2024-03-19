#include "touwenjian.h"

#define I2C_MASTER_SCL_IO    4    //scl gpoi
#define I2C_MASTER_SDA_IO    5    //sda gpoi
#define I2C_MASTER_NUM        I2C_NUM_0    // I2C port number for master dev
#define I2C_MASTER_FREQ_HZ    100000     // I2C master clock frequency 
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0
#define MPU6050_I2C_ADDR                 0x68
#define MPU6050_READ_ADDR                0x3B
#define MPU6050_PWR_MGMT_1                  0x6B // wake up
#define ACK_CHECK_EN                        0x1     //master will check ack from slave
#define ACK_CHECK_DIS                       0x0     //master will not check ack from slave 
#define ACK_VAL                             0x0         // I2C ack value 
#define NACK_VAL                            0x1         // I2C nack value 

static const char *TAG = "MPU6050";

// i2c master initialization
void i2c_master_init()
{
    i2c_config_t conf={
        .mode = I2C_MODE_MASTER, //master mode
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0); //缓冲区大小都为0，slave mode才会用到,也不用分配中断的标志
}

//wake up mpu6050
esp_err_t mpu6050_write_byte(uint8_t regAddr, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    //写入的信号
    i2c_master_write_byte(cmd, MPU6050_I2C_ADDR << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
    //写入目标寄存器地址
    i2c_master_write_byte(cmd, regAddr, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}


//function to read multiple bytes from MPU6050
esp_err_t mpu6050_read_bytes(uint8_t regAddr, uint8_t *data, uint16_t length)
{
    if (length == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    //写入的信号
    i2c_master_write_byte(cmd, MPU6050_I2C_ADDR << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
    //写入目标寄存器地址
    i2c_master_write_byte(cmd, regAddr, ACK_CHECK_EN);
    i2c_master_start(cmd);
    //读的信号
    i2c_master_write_byte(cmd, MPU6050_I2C_ADDR << 1 | I2C_MASTER_READ, ACK_CHECK_EN);
    if (length > 1) {
        i2c_master_read(cmd, data, length - 1, ACK_VAL);
    }
    //停止读
    i2c_master_read_byte(cmd, data + length - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

//封装一个读数据的函数
void mpu6050_read()
{
    uint8_t read_data[14];
    i2c_master_init();
    // Wake up MPU6050
    mpu6050_write_byte(MPU6050_PWR_MGMT_1, 0);
    while (1)
    {
        mpu6050_read_bytes(MPU6050_READ_ADDR, read_data, 14);
        ESP_LOGI(TAG, "Acceleration: X:%d, Y:%d, Z:%d", read_data[0] << 8 | read_data[1], read_data[2] << 8 | read_data[3], read_data[4] << 8 | read_data[5]);
        ESP_LOGI(TAG, "Gyroscope: X:%d, Y:%d, Z:%d", read_data[8] << 8 | read_data[9], read_data[10] << 8 | read_data[11], read_data[12] << 8 | read_data[13]);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
