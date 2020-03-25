//Standard Libraries
#include <string.h>
#include <stdlib.h>
#include <sys/time.h>
#include <time.h>

//Other ESP-IDF Components
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_adc_cal.h"
#include "esp_http_client.h"
#include "cJSON.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "bsec_integration.h"
#include "TCS34725.h"
#include "esp_now.h"

extern void sendDataOverEspNow(char *cJson);

static const char *TAG = "Spacr";
// const int requestSend = BIT1;
int taskBIT = BIT2;
// static EventGroupHandle_t requestDone;
EventGroupHandle_t taskGroup;
RTC_DATA_ATTR static uintmax_t timestampDS = 0;
//RTC_DATA_ATTR static bool initialised = false, TCS_init = false;
int timeToSleepAdd = 298*1000000; //5mins

//I2C Configurations
#define I2C_BUS       0
#define I2C_SCL_PIN   16
#define I2C_SDA_PIN   13
#define I2C_FREQ      50000

//ADC reference voltage in mV
#define VREF 1100

//char accessKey[] = "NLKcT8FyNO852RIUrza2";

//bsec config stuff
const uint8_t bsec_config_iaq[454] = 
     {4,7,4,1,61,0,0,0,0,0,0,0,174,1,0,0,48,0,1,0,0,168,19,73,64,49,119,76,0,0,225,68,137,65,0,63,205,204,204,62,0,0,64,63,205,204,204,62,0,0,0,0,216,85,0,100,0,0,0,0,0,0,0,0,28,0,2,0,0,244,1,225,0,25,0,0,128,64,0,0,32,65,144,1,0,0,112,65,0,0,0,63,16,0,3,0,10,215,163,60,10,215,35,59,10,215,35,59,9,0,5,0,0,0,0,0,1,88,0,9,0,7,240,150,61,0,0,0,0,0,0,0,0,28,124,225,61,52,128,215,63,0,0,160,64,0,0,0,0,0,0,0,0,205,204,12,62,103,213,39,62,230,63,76,192,0,0,0,0,0,0,0,0,145,237,60,191,251,58,64,63,177,80,131,64,0,0,0,0,0,0,0,0,93,254,227,62,54,60,133,191,0,0,64,64,12,0,10,0,0,0,0,0,0,0,0,0,229,0,254,0,2,1,5,48,117,100,0,44,1,112,23,151,7,132,3,197,0,92,4,144,1,64,1,64,1,144,1,48,117,48,117,48,117,48,117,100,0,100,0,100,0,48,117,48,117,48,117,100,0,100,0,48,117,48,117,100,0,100,0,100,0,100,0,48,117,48,117,48,117,100,0,100,0,100,0,48,117,48,117,100,0,100,0,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,8,7,8,7,8,7,8,7,8,7,8,7,8,7,8,7,8,7,8,7,8,7,8,7,8,7,8,7,112,23,112,23,112,23,112,23,112,23,112,23,112,23,112,23,112,23,112,23,112,23,112,23,112,23,112,23,255,255,255,255,255,255,255,255,220,5,220,5,220,5,255,255,255,255,255,255,220,5,220,5,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,44,1,0,0,0,0,23,142,0,0};

extern const uint8_t name_pem_start[] asm("_binary_name_pem_start");
extern const uint8_t name_pem_end[] asm("_binary_name_pem_end");
//const char *deviceName = (const char*) name_pem_start;

//Color Temp VARS
uint16_t colorTemp;
uint16_t lux;
float r,g,b;

//*******************************************************************************************************
//I2C functions
//*******************************************************************************************************
void i2c_init (int bus, gpio_num_t scl, gpio_num_t sda, uint32_t freq){
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = sda;
    conf.scl_io_num = scl;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = freq;
    i2c_param_config(bus, &conf);
    i2c_driver_install(bus, I2C_MODE_MASTER, 0, 0, 0);
}

int i2c_master_write_slave (uint8_t bus, uint8_t addr, const uint8_t *reg, 
                     uint8_t *data, uint32_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, addr << 1 | I2C_MASTER_WRITE, true);
    if (reg)
        i2c_master_write_byte(cmd, *reg, true);
    if (data)
        i2c_master_write(cmd, data, len, true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(bus, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    
    return err;
}

int i2c_master_read_slave (uint8_t bus, uint8_t addr, const uint8_t *reg, 
                    uint8_t *data, uint32_t len)
{
    if (len == 0) return true;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (reg)
    {
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, ( addr << 1 ) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, *reg, true);
        if (!data)
            i2c_master_stop(cmd);
    }
    if (data)
    {
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, ( addr << 1 ) | I2C_MASTER_READ, true);
        if (len > 1) i2c_master_read(cmd, data, len-1, I2C_MASTER_ACK);
        i2c_master_read_byte(cmd, data + len-1, I2C_MASTER_NACK);
        i2c_master_stop(cmd);
    }
    esp_err_t err = i2c_master_cmd_begin(bus, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return err;
}
//*******************************************************************************************************

// esp_err_t _http_event_handler(esp_http_client_event_t *evt)
// {
//     switch(evt->event_id) {
//         case HTTP_EVENT_ERROR:
//             ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
//             break;
//         case HTTP_EVENT_ON_CONNECTED:
//             ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
//             break;
//         case HTTP_EVENT_HEADER_SENT:
//             ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
//             break;
//         case HTTP_EVENT_ON_HEADER:
//             ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
//             break;
//         case HTTP_EVENT_ON_DATA:
//             ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
//             break;
//         case HTTP_EVENT_ON_FINISH:
//             ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
//             break;
//         case HTTP_EVENT_DISCONNECTED:
//             ESP_LOGD(TAG, "HTTP_EVENT_DISCONNECTED");
//             break;
//     }
//     return ESP_OK;
// }

// int sendRequest(char url[], char pData[]){
//     int ret;
//     esp_http_client_config_t config = {
//         .url = url,
//         .event_handler = _http_event_handler,
//     };
//     esp_http_client_handle_t client = esp_http_client_init(&config);
//     const char *post_data = pData;
//     esp_http_client_set_url(client, url);
//     esp_http_client_set_method(client, HTTP_METHOD_POST);
//     esp_http_client_set_header(client, "Content-Type", "application/json");
//     esp_http_client_set_post_field(client, post_data, strlen(post_data));
//     esp_err_t err = esp_http_client_perform(client);
//     if (err == ESP_OK){
//         xEventGroupSetBits(requestDone, requestSend);
//         ret = esp_http_client_get_status_code(client);
//     }else{
//         xEventGroupSetBits(requestDone, requestSend);
//         ret = 0;
//     }
//     xEventGroupWaitBits(requestDone, requestSend,
//                         false, true, portMAX_DELAY);
//     xEventGroupClearBits(requestDone, requestSend);
//     esp_http_client_cleanup(client);
//     return ret;
// }

//*******************************************************************************************************
//BSEC BME680 functions
//*******************************************************************************************************
int8_t bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data_ptr, uint16_t data_len)
{
    int8_t err = i2c_master_write_slave(I2C_BUS,dev_addr,&reg_addr,reg_data_ptr,data_len);
    return err;
}

int8_t bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data_ptr, uint16_t data_len)
{
    int8_t err = i2c_master_read_slave(I2C_BUS,dev_addr,&reg_addr,reg_data_ptr,data_len);
    return err;
}

void bsleep(uint32_t t_ms)
{
    vTaskDelay(t_ms / portTICK_PERIOD_MS);
}

uintmax_t get_timestamp_us()
{
    uintmax_t timeret;
    if (timestampDS == 0){
        struct timeval time;
        gettimeofday(&time, NULL);
        timeret = (time.tv_sec*1000000) +  time.tv_usec;
        timestampDS = timeret;
        return timeret;
    }else{
        timestampDS = timestampDS + timeToSleepAdd;
        timeret = timestampDS;
        return timeret;
    }
}

uint32_t state_load(uint8_t *state_buffer, uint32_t n_buffer)
{
    esp_err_t err;
    nvs_handle my_handle;
    err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
        return 0;
    } else {
        uint32_t lengthS = 0;
        err = nvs_get_u32(my_handle, "state_length", &lengthS);
        printf((err != ESP_OK) ? "Failed!\n" : "");
        err = nvs_get_blob(my_handle, "state_buffer", state_buffer, &lengthS);
        printf((err != ESP_OK) ? "Failed!\n" : "");
        nvs_close(my_handle);
        if (err == ESP_OK ){
            return lengthS; 
        }else{
            return 0;
        }
    }
}

void state_save(const uint8_t *state_buffer, uint32_t length)
{
    esp_err_t err;
    nvs_handle my_handle;
    err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else {
        err = nvs_set_blob(my_handle, "state_buffer", state_buffer, length);
        printf((err != ESP_OK) ? "Failed!\n" : "");
        err = nvs_set_u32(my_handle, "state_length", length);
        printf((err != ESP_OK) ? "Failed!\n" : "");
        err = nvs_commit(my_handle);
        printf((err != ESP_OK) ? "Failed!\n" : "");
    }
    nvs_close(my_handle);
}

uint32_t config_load(uint8_t *config_buffer, uint32_t n_buffer)
{
    memcpy(config_buffer, bsec_config_iaq, sizeof(bsec_config_iaq));
    return sizeof(bsec_config_iaq);
}

void output_ready(int64_t timestamp, float iaq, uint8_t iaq_accuracy, float temperature, float humidity,
     float pressure, float raw_temperature, float raw_humidity, float gas, bsec_library_return_t bsec_status,
     float static_iaq, float co2_equivalent, float breath_voc_equivalent)
{
    cJSON *root = cJSON_CreateObject();
    cJSON_AddItemToObject(root, "name", cJSON_CreateString((const char*) name_pem_start));
    cJSON_AddNumberToObject(root, "temperature", temperature);
    cJSON_AddNumberToObject(root, "humidity", humidity + 3);
    cJSON_AddNumberToObject(root, "pressure", pressure/1000);
    //cJSON_AddNumberToObject(root, "gasresistance", gas);
    cJSON_AddNumberToObject(root, "co2", co2_equivalent);
    //cJSON_AddNumberToObject(root, "breathVOC", breath_voc_equivalent);
    cJSON_AddNumberToObject(root, "airquality", static_iaq);
    cJSON_AddNumberToObject(root, "illuminance", lux);
    cJSON_AddNumberToObject(root, "colortemperature", colorTemp);
    cJSON_AddNumberToObject(root, "accuracy", iaq_accuracy);

    //Voltage Reading
    double day = timestamp/(8.64*(pow(10,13))); //Convert nanoseconds to days
    if((day - trunc(day)) > 0.99655092592593){ //Value is 1day minus 5mins in days
        uint32_t V_raw; 
        float V_out;
        esp_adc_cal_characteristics_t *adc_chars;

        //Initialise ADC
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(ADC1_GPIO32_CHANNEL, ADC_ATTEN_DB_11);
        adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
        esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, VREF, adc_chars);
        esp_adc_cal_get_voltage(ADC1_GPIO32_CHANNEL, adc_chars, &V_raw);
        V_out = (0.00000000294115617841*(pow(V_raw,4))) - (0.0000362770485278603*(pow(V_raw,3))) + (0.167778744994736*(pow(V_raw,2))) - (344.838980459769*V_raw) + 265757.594882814;
        cJSON_AddNumberToObject(root, "voltage", V_out);
        adc_power_off();
    }

    char *cJson = cJSON_PrintUnformatted(root);
    sendDataOverEspNow(cJson);

    //sendRequest("https://spacr.ai/submitData", cJson);
    //sendRequest("http://172.16.1.143:8081/submitData", cJson);
    //sendRequest("http://spacr-devices.68p3mggwmz.ca-central-1.elasticbeanstalk.com/submitData",cJson);
    cJSON_Delete(root);
    free(cJson);

    printf("[%ld] T: %f degC | rH: %f%% | P: %f kPa | G: %f Ohms | eCO2: %f ppm | VOC:%f ppm | sIAQ: %f (%d)\r\n",
    (long)(timestamp/1000000),temperature,humidity + 3,pressure/1000,gas,co2_equivalent,breath_voc_equivalent,static_iaq,iaq_accuracy);
    printf("Color Temperature: %d K | Illuminance: %d lux\r\n\n",colorTemp, lux);
}

void invokeBME(){
    /* Timestamp variables */
    uintmax_t time_stamp = 0;
    
    /* Allocate enough memory for up to BSEC_MAX_PHYSICAL_SENSOR physical inputs*/
    bsec_input_t bsec_inputs[BSEC_MAX_PHYSICAL_SENSOR];
    
    /* Number of inputs to BSEC */
    uint8_t num_bsec_inputs = 0;
    
    /* BSEC sensor settings struct */
    bsec_bme_settings_t sensor_settings;
    
    /* Save state variables */
    uint8_t bsec_state[BSEC_MAX_STATE_BLOB_SIZE];
    uint8_t work_buffer[BSEC_MAX_STATE_BLOB_SIZE];
    uint32_t bsec_state_len = 0;
    uint32_t n_samples = 0;
    
    bsec_library_return_t bsec_status = BSEC_OK;

    /* get the timestamp in nanoseconds before calling bsec_sensor_control() */
    time_stamp = get_timestamp_us() * 1000;
    
    /* Retrieve sensor settings to be used in this time instant by calling bsec_sensor_control */
    bsec_sensor_control(time_stamp, &sensor_settings);
    
    /* Trigger a measurement if necessary */
    bme680_bsec_trigger_measurement(&sensor_settings, bsleep);
    
    /* Read data from last measurement */
    num_bsec_inputs = 0;
    bme680_bsec_read_data(time_stamp, bsec_inputs, &num_bsec_inputs, sensor_settings.process_data);
    
    /* Time to invoke BSEC to perform the actual processing */
    bme680_bsec_process_data(bsec_inputs, num_bsec_inputs, output_ready);
    
    /* Increment sample counter */
    n_samples++;
    
    /* Retrieve and store state if the passed save_intvl */
    if (n_samples >= 1)
    {
        bsec_status = bsec_get_state(0, bsec_state, sizeof(bsec_state), work_buffer, sizeof(work_buffer), &bsec_state_len);
        if (bsec_status == BSEC_OK)
        {
            state_save(bsec_state, bsec_state_len);
        }
        n_samples = 0;
    } 
}
//*******************************************************************************************************

void calculateColor(uint16_t *temp, uint16_t *lx, float *r, float *g, float *b){
    uint16_t rr, gg, bb, cc = 0;
    //getRGB(r,g,b);
    getRawData(&rr,&gg,&bb,&cc);
    *temp = calculateColorTemperature(rr, gg, bb);
    *lx = calculateLux(rr, gg, bb);
}

void sensorBeginRun() {
    
    //xEventGroupClearBits(taskGroup, taskBIT);
    //Check if device is registered, if not, we go back to sleep.
    cJSON *root = cJSON_CreateObject();
    cJSON_AddItemToObject(root, "deviceID", cJSON_CreateString((const char*) name_pem_start));
    char *cJson = cJSON_PrintUnformatted(root);
    //int auth_code = sendRequest("https://spacr.ai/deviceRegistered", cJson);
    //int auth_code = sendRequest("http://spacr-devices.68p3mggwmz.ca-central-1.elasticbeanstalk.com/deviceRegistered", cJson);
    int auth_code = 409;
    cJSON_Delete(root);
    free(cJson);
 
    //checks if the device is in system, if not, it will sleep for 5 minutes.
    if (auth_code == 409){
        //Initialise I2C
        i2c_init(I2C_BUS, I2C_SCL_PIN, I2C_SDA_PIN, I2C_FREQ);

        /* if(!initialised){
            //Initialise Colour Sensor
            if(sensor_init()){
                TCS_init = true;
                initialised = true;
            }else{
                ESP_LOGE(TAG, "TCS34725 Color Sensor failed to initialise\n");
                TCS_init = false;
            }
        } */

        //Initialise, run and disable colour sensor
        if(sensor_init()){
            //10-20uA in deep sleep
            enableTCS();
            calculateColor(&colorTemp, &lux, &r, &g, &b); //Wrong reading
            calculateColor(&colorTemp, &lux, &r, &g, &b); //Correct reading
            disableTCS();
        }else {
            ESP_LOGE(TAG, "TCS34725 Color Sensor failed to initialise\n");
        }
        /* if(TCS_init){
            //30-40uA
            enableTCS();
            calculateColor(&colorTemp, &lux, &r, &g, &b);
            calculateColor(&colorTemp, &lux, &r, &g, &b);
            disableTCS();
        } */

        //Get bme680 data
        return_values_init ret;
        ret = bsec_iot_init(BSEC_SAMPLE_RATE_ULP, 0.0f, bus_write, bus_read, bsleep, state_load, config_load);
        if (ret.bme680_status || ret.bsec_status)
        {
            //xEventGroupSetBits(taskGroup, taskBIT);
            return;
        }
        invokeBME();
    }
    //xEventGroupSetBits(taskGroup, taskBIT);
    return;  
}


//wifi disabled
//bluetooth disabled
//wifi transmit speed lower (lower dbm)
//only single core
//sensor put to sleep
//sleeping on ulp mode for bme680
//http://iot-bits.com/ultra-low-power-design-esp32-10-tips/
//https://www.macnica.eu/Ultra-Low-Power-Management-Espressif-ESP32
//also added power management option (dont know what it will do)

//since we are not using aws anymore, we can move the power operation down to 80MHZ

//NEED TO IMPLEMENT RESET OF BME680