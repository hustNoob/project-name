#include "touwenjian.h"
#include "i2c_mpu6050.h"
#include "uart_send.h"
#include "wifi.h"

void mpu6050_send();

void app_main(void)
{
    /*以下三个都是带循环的，选一个*/
    //mpu6050_read();
    //uart_send_data();
    //mpu6050_send();
}

 //封装一个将mpu6050数据发送到计算机的函数
void mpu6050_send(){
    uint8_t mpu6050_data[14];
    int data_send[6] = {0};
    int i, j;

    i2c_master_init();
    // Wake up MPU6050
    mpu6050_write_byte(MPU6050_PWR_MGMT_1, 0);

    SAT_SET();
    vTaskDelay(pdMS_TO_TICKS(2000));//用于sat初始化和socket设置之间必要的时间

    int server_socket = socket_tcp();//返回一个连接好的套接字
    while(1){
    mpu6050_read_bytes(MPU6050_READ_ADDR, mpu6050_data, 14);
    for (i = 0, j = 0; i < 3; i++){
        data_send[i] = mpu6050_data[j] << 8 | mpu6050_data[j + 1];
        data_send[i + 3] = mpu6050_data[j + 8] << 8 | mpu6050_data[j + 9];
        j = j + 2;
    }
    ESP_LOGI(TAG, "Acceleration: X:%d, Y:%d, Z:%d", data_send[0], data_send[1], data_send[2] );
    ESP_LOGI(TAG, "Gyroscope: X:%d, Y:%d, Z:%d", data_send[3], data_send[4], data_send[5]);
    //以上两行用于检验data_send是否正确
    send(server_socket, data_send, sizeof(data_send), 0);//发送给电脑
    vTaskDelay(pdMS_TO_TICKS(1000));
    }
}