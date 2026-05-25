#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sockets.h"

// Network config
#define WIFI_SSID      "ssid"
#define WIFI_PASS      "password"
#define UDP_PORT       4000

static const char *TAG = "UDP_APP";


static EventGroupHandle_t s_wifi_event_group;

#define WIFI_CONNECTED_BIT BIT0

float Kp = 0.05;
float Ki = 0.00;
float Kd = 0.01;
float previous_error = 0;
float integral = 0;
float dt = 0.05;

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "Trying to reconnect to Wi-Fi...");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "ESP32 IP: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}


void wifi_init_sta(void) {
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
}

void TaskComunicacaoUDP(void *pvParameters) {
    char rx_buffer[255];

    while (1) {
        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(UDP_PORT);

        int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
        if (sock < 0) {
            ESP_LOGE(TAG, "Error socket creation: errno %d", errno);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err < 0) {
            ESP_LOGE(TAG, "Error socket bind: errno %d", errno);
            close(sock);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        ESP_LOGI(TAG, "Listen in UDP port %d", UDP_PORT);

        struct sockaddr_storage source_addr;

        while (1) {
            socklen_t socklen = sizeof(source_addr);

            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);
            
            if (len < 0) {
                ESP_LOGE(TAG, "Error in recvfrom: errno %d", errno);
                break; 
            } else {
                rx_buffer[len] = 0; 
                
                float current_omega = atof(rx_buffer);

                float omega_ref = 0.0;
                float error = omega_ref - current_omega;
                
                integral += error * dt;
                float derivative = (error - previous_error) / dt;
                
                float delta_comand = (Kp * error) + (Ki * integral) + (Kd * derivative);
                previous_error = error;
                
                if (delta_comand > 0.5) delta_comand = 0.5;
                if (delta_comand < -0.5) delta_comand = -0.5;

                char tx_buffer[32];
                snprintf(tx_buffer, sizeof(tx_buffer), "%.4f", delta_comand);

                
                int send_err = sendto(sock, tx_buffer, strlen(tx_buffer), 0, (struct sockaddr *)&source_addr, socklen);
                
                if (send_err < 0) {
                    ESP_LOGE(TAG, "Error to send callback: errno %d", errno);
                } else {
                    ESP_LOGI(TAG, "Sensor w: %.4f | Comand Delta: %.4f", current_omega, delta_comand);
                }
            }
            
            
            vTaskDelay(pdMS_TO_TICKS(5));
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Ending socket and restarting...");
            close(sock);
        }
    }
    
    vTaskDelete(NULL);
}

void app_main(void) {

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "Conecting to WiFi...");
    
    
    wifi_init_sta();
    
    ESP_LOGI(TAG, "WiFi connected and network initialization complete!");

    
    xTaskCreatePinnedToCore(
        TaskComunicacaoUDP, 
        "TaskUDP",          
        4096,               
        NULL,               
        1,                  
        NULL,               
        0                   
    );
}