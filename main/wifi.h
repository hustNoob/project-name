#include "touwenjian.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"

void wifi_callback(void* event_handler_arg, esp_event_base_t event_base, int32_t event_id, void* event_data){
    if(event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED){
        ip_event_got_ip_t* got_ip = (ip_event_got_ip_t*) event_data;
        ESP_LOGI("IP_ADDR", "Your ip address is" IPSTR, IP2STR(&got_ip->ip_info.ip));
    }//获取ip地址
}


 void SAT_SET(){
    nvs_flash_init();//键值对

    esp_event_loop_create_default();//创建事件循环（是一个线程）
    esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_CONNECTED, wifi_callback, NULL);

    esp_netif_init();//初始化网络接口
    esp_netif_create_default_wifi_sta();//创建sta类网卡
    wifi_init_config_t wifi_conf = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&wifi_conf);//初始化wifi底层配置
    esp_wifi_set_mode(WIFI_MODE_STA);//设置wifi模式
    wifi_config_t sta_conf={
        .sta={
            .ssid = "esp32s3_test",
            .password = "12345678"
        }
    };
    esp_wifi_set_config(WIFI_IF_STA, &sta_conf);//配置sta的相关参数

    esp_wifi_start();
    esp_wifi_connect();//启动与连接

 }