#include "touwenjian.h"

const char* message = "socket send successfully!";
char rev_buffer[10000];

void wifi_callback(void* event_handler_arg, esp_event_base_t event_base, int32_t event_id, void* event_data){
    if(event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP){
        ip_event_got_ip_t* got_ip = (ip_event_got_ip_t*) event_data;
        ESP_LOGI("IP_ADDR", "Your ip address is " IPSTR, IP2STR(&got_ip->ip_info.ip));
    }//获取ip地址
}

//设置为SAT模式并顺便获取ip地址
 void SAT_SET(){
    nvs_flash_init();//键值对

    esp_event_loop_create_default();//创建默认事件循环（是一个线程）
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_callback, NULL);

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

//配置socket，作为tcp客户端收发数据
 int socket_tcp(){
    //创建服务器端的套接字:说明服务端接口的协议类型，自定义一个该服务端的代号
    int server_socket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (server_socket == -1){
        printf("套接字创建失败");
        return 0;
    }
    //说明服务端的位置（ip+port）
    struct sockaddr_in server_MSG = {
        .sin_family = AF_INET,
        .sin_port = htons(4000), //平时是小端存储，而这里是sin_port是大端存储，htons可以将高低8位颠倒
        .sin_addr.s_addr = inet_addr("192.168.137.1")
        //inet_addr:将一个点分十进制的IPv4地址字符串转换为网络字节序的二进制值。
    };
    //将socket与服务端连接
    connect(server_socket, (const struct sockaddr *)&server_MSG, sizeof(server_MSG));
    /*if (flag == 0)
        //发送数据
        send(server_socket, send_message, sizeof(send_message), 0);
    else if (flag == 1)
        //接收数据
        recv(server_socket, rev_buffer, 9999, 0);*/
    return server_socket;
 }
 