#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_event_loop.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "tcpip_adapter.h"
#include "nvs_flash.h"
#include "driver/rtc_io.h"
#include "driver/i2c.h"
#include "espNow.h"

static const char *TAG = "IEQ_SENSOR";
extern int taskBIT;
extern EventGroupHandle_t taskGroup;

void app_main()
{
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    ESP_LOGI(TAG, "ESPNOW MODE START: ");
    xTaskCreatePinnedToCore(startEspNow, "startEspNow",  20000, NULL, 5, NULL, 0);
    // startEspNow();
}