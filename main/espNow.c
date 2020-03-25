#include <stdio.h>
#include "espNow.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_event_loop.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_sleep.h"
#include "freertos/event_groups.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "tcpip_adapter.h"
#include "nvs_flash.h"
#include "driver/rtc_io.h"
#include "driver/i2c.h"

extern void sensorBeginRun();
uint8_t broadcastAddress[]    = {0xB4,0xE6,0x2D,0xEB,0x18,0x81};      
char *TAGEspNow               = "ESPNOW MODE: ";
char *recievedData;
static const char *TAG = "IEQ_SENSOR";
int  state = 1;
esp_now_peer_info_t peerInfo;
const int CONNECTED_BIT = BIT0;
// static EventGroupHandle_t s_wifi_event_group;
//Time to sleep 20 minutes
int timeToSleep = 20000000;
const int requestSend = BIT1;
static EventGroupHandle_t requestDone;

void deactivate(){
    esp_wifi_stop();
    nvs_flash_deinit();
    rtc_gpio_isolate(GPIO_NUM_0);
    rtc_gpio_isolate(GPIO_NUM_2);
    rtc_gpio_isolate(GPIO_NUM_5);
    rtc_gpio_isolate(GPIO_NUM_12);
    rtc_gpio_isolate(GPIO_NUM_15);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
}


void startEspNow(){
    requestDone = xEventGroupCreate();

    // esp_reset_reason_t reason = esp_reset_reason();
    // ESP_LOGI(TAGEspNow, "Reset reason %d",reason);
    // if (reason == 8) {
    //     esp_restart();
    // }
    // if (reason == 3) {
    //     ESP_LOGI(TAGEspNow, "Go to state 3");
    //     espNowReInit();
    //     state = 3;
    // }

    while(1){
        if(state == 1){espNowReInit();}
        if(state == 2){sensorBeginRun(); vTaskDelay(5000 / portTICK_PERIOD_MS);}
        if(state == 3){
            deactivate();
            ESP_LOGI(TAG, "Entering Deep Sleep...");
            esp_sleep_enable_timer_wakeup(timeToSleep);
            esp_deep_sleep_start();
        }
    }

}

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
        ESP_LOGI(TAG, "WiFi started");
        break;
    default:
        break;
    }
    return ESP_OK;
}

void espNowReInit(){
    tcpip_adapter_init();    
    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_APSTA) );
    ESP_ERROR_CHECK( esp_wifi_start());
    ESP_ERROR_CHECK( esp_wifi_set_channel(6, 0) );
    if (esp_now_init() != ESP_OK) {
        ESP_LOGI(TAGEspNow, "Error at Espnow init");
        return;
    }
    else{
        ESP_LOGI(TAGEspNow, "Espnow init OK");
    }

    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK){
    ESP_LOGI(TAGEspNow, "Error adding peer");
    return;
    }
    else{
        ESP_LOGI(TAGEspNow, "Peer info added");
    }

    //Call back function to send data
    esp_now_register_send_cb(OnDataSent);
    state = 2;
}

//send the data
void sendDataOverEspNow(char *message){
    
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)message, 250);

    if (result == ESP_OK) {
        xEventGroupSetBits(requestDone, requestSend);

        ESP_LOGI(TAGEspNow, "Data sent successfully");
    }
    else {
        xEventGroupSetBits(requestDone, requestSend);

        ESP_LOGI(TAGEspNow, "Error sending data");
    }
    xEventGroupWaitBits(requestDone, requestSend,
                        false, true, portMAX_DELAY);
    xEventGroupClearBits(requestDone, requestSend);
}
//callback function after data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  ESP_LOGI(TAGEspNow,"Last Packet Sent to: "); ESP_LOGI(TAGEspNow,"MAC: %s",macStr);
  state = 3;
}