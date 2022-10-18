/* Simple HTTP + SSL Server Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include "esp_netif.h"
#include "esp_eth.h"
#include "protocol_examples_common.h"
#include "freertos/event_groups.h"

#include <esp_https_server.h>
#include "esp_tls.h"

#include "cJSON.h"

#include "nvs.h"

#include <driver/gpio.h>

#include "driver/timer.h"

#include "st7789.h"

#include "esp_sntp.h"

// #define CONFIG_ESP_WIFI_AUTH_WPA2_PSK 1

#if CONFIG_ESP_WIFI_AUTH_OPEN
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN
#elif CONFIG_ESP_WIFI_AUTH_WEP
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WEP
#elif CONFIG_ESP_WIFI_AUTH_WPA_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WAPI_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WAPI_PSK
#endif



/* A simple example that demonstrates how to create GET and POST
 * handlers and start an HTTPS server.
*/

#define ESP_WIFI_SSID      "ESP_EXAM"
#define ESP_WIFI_PASS      "12345678"
#define EXAMPLE_ESP_WIFI_CHANNEL   1
#define EXAMPLE_MAX_STA_CONN       4

#define ESP_MAXIMUM_RETRY  10

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

#define MAGICNUMBER 0xAA5413

#define WIFI_STATE_AP 0x22
#define WIFI_STATE_STA 0x44

#define MOSFET1 32
#define MOSFET2 33
#define MOSFET3 25

#define TRIAC1 26
#define TRIAC2 27
#define TRIAC3 14

#define CROSSIN 34

#define TIMERG_0     0
#define TIMERG_1     1
#define hw_timer_0   0
#define hw_timer_1   1

#define TIMER_DIVIDER         8  //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds


static const char *TAG = "ESP_HOME";

time_t now;
struct tm timeinfo;

static int s_retry_num = 0;

static EventGroupHandle_t s_wifi_event_group;

nvs_handle_t app_nvs_handle;

spi_device_handle_t SPIhandle;
TaskHandle_t xSun_task_handl;

typedef enum
{
    mos_1,
    mos_2,
    mos_3
}MOS_num_t;

typedef enum
{
    tri_1,
    tri_2,
    tri_3
}TRIAC_num_t;

static void LCD_info_setwifi();
static bool write_state_wifi(uint8_t WIFI_state);
static void enable_interr_cross();
static void check_need_interr_cross();
static bool check_en_sun_mode(TRIAC_num_t num);
static void disable_sun_mode();

static uint8_t counter_one=0;
static uint8_t counter_two=0;
static uint8_t prob=1;
static bool cross=0;




typedef struct 
{
    uint8_t TRIAC1_Del;
    uint8_t TRIAC2_Del;
    uint8_t TRIAC3_Del;

    bool    TRIAC1_state;
    bool    TRIAC2_state;
    bool    TRIAC3_state;

    uint8_t TRIAC1_counter;
    uint8_t TRIAC2_counter;
    uint8_t TRIAC3_counter;
}TRIAC_state_strc;

typedef struct 
{
    uint8_t hour_start;
    uint8_t min_start;
    uint8_t  dur;
    uint8_t chanal;
    bool state;
    bool sun_on;
}sun_mode_strc;

sun_mode_strc sun_mode={
    .state=false,
    .sun_on=false,
};


TRIAC_state_strc TRIAC_state=
{
    .TRIAC1_state=false,
    .TRIAC2_state=false,
    .TRIAC3_state=false,
    
    .TRIAC1_Del=10,
    .TRIAC2_Del=10,
    .TRIAC3_Del=10,

    .TRIAC1_counter=0,
    .TRIAC2_counter=0,
    .TRIAC3_counter=0,
};



/* An HTTP GET handler */
static esp_err_t root_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, "<h1>Hello Secure World!</h1>", HTTPD_RESP_USE_STRLEN);

    return ESP_OK;
}

#if CONFIG_EXAMPLE_ENABLE_HTTPS_USER_CALLBACK
/**
 * Example callback function to get the certificate of connected clients,
 * whenever a new SSL connection is created
 *
 * Can also be used to other information like Socket FD, Connection state, etc.
 */
void https_server_user_callback(esp_https_server_user_cb_arg_t *user_cb)
{
    ESP_LOGI(TAG, "Session Created!");
    const mbedtls_x509_crt *cert;

    const size_t buf_size = 1024;
    char *buf = calloc(buf_size, sizeof(char));
    if (buf == NULL) {
        ESP_LOGE(TAG, "Out of memory - Callback execution failed!");
        return;
    }

    cert = mbedtls_ssl_get_peer_cert(&user_cb->tls->ssl);
    if (cert != NULL) {
        mbedtls_x509_crt_info((char *) buf, buf_size - 1, "      ", cert);
        ESP_LOGI(TAG, "Peer certificate info:\n%s", buf);
    } else {
        ESP_LOGW(TAG, "Could not obtain the peer certificate!");
    }

    free(buf);
}
#endif

void IRAM_ATTR gpio_CROSS_isr_handler(void* arg)
{   
    timer_start(TIMERG_0, hw_timer_1);
    gpio_intr_disable(CROSSIN);
    cross=1;   
}

void IRAM_ATTR timer_CROSS_callback()
{   
     
    if(TRIAC_state.TRIAC1_state)
    {
        if(TRIAC_state.TRIAC1_counter==TRIAC_state.TRIAC1_Del && cross)
        {
            gpio_set_level(TRIAC1,1); 
        }
    }

    if(TRIAC_state.TRIAC2_state)
    {
        if(TRIAC_state.TRIAC2_counter==TRIAC_state.TRIAC2_Del && cross)
        {
            gpio_set_level(TRIAC2,1); 
        }
    }
    if((TRIAC_state.TRIAC1_counter>9||TRIAC_state.TRIAC2_counter>9)  && cross)
    {
        gpio_intr_enable(CROSSIN);
        timer_pause(TIMERG_0, hw_timer_1);
        cross=0;
        TRIAC_state.TRIAC1_counter=0;
        TRIAC_state.TRIAC2_counter=0;
    }


    if(TRIAC_state.TRIAC2_counter==TRIAC_state.TRIAC2_Del || TRIAC_state.TRIAC1_counter==TRIAC_state.TRIAC1_Del)
    {
        for(int i=0;i<0xAAA;i++);
        
        gpio_set_level(TRIAC1,0); 
        gpio_set_level(TRIAC2,0); 
    }
    TIMERG0.hw_timer[hw_timer_1].update = 1;
    TIMERG0.int_clr_timers.t0 = 1;
    TIMERG0.hw_timer[hw_timer_1].config.alarm_en = 1;
    TRIAC_state.TRIAC1_counter++;
    TRIAC_state.TRIAC2_counter++;
}

void init_timer()
{
    timer_config_t config = {
        .divider = TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = 1,
        .intr_type = TIMER_INTR_LEVEL,
    }; 
    
    timer_init(TIMERG_0, hw_timer_1, &config);
    timer_pause(TIMERG_0, hw_timer_1);
    timer_set_counter_value(TIMERG_0, hw_timer_1, 0);
    timer_set_alarm_value(TIMERG_0, hw_timer_1, 0.0008 * TIMER_SCALE); // 0.008 min 
    timer_enable_intr(TIMERG_0, hw_timer_1);
   
    timer_isr_callback_add(TIMERG_0, hw_timer_1, timer_CROSS_callback, NULL , 0);
}

void init_gpio_intr()
{

    gpio_set_direction(CROSSIN,  GPIO_MODE_INPUT);
    gpio_set_intr_type(CROSSIN, GPIO_INTR_POSEDGE);
    // gpio_intr_enable(CROSSIN);
    check_need_interr_cross();

    //  gpio_isr_register(gpio_pp_isr_handler, NULL, ESP_INTR_FLAG_IRAM, NULL); // 17
    gpio_install_isr_service(0);
    gpio_isr_handler_add(CROSSIN ,gpio_CROSS_isr_handler, NULL );

}

static void initHW()
{
    gpio_set_direction (TRIAC1,GPIO_MODE_OUTPUT);
    gpio_set_direction (TRIAC2,GPIO_MODE_OUTPUT);
    gpio_set_direction (TRIAC3,GPIO_MODE_OUTPUT);

    gpio_set_direction (MOSFET1,GPIO_MODE_OUTPUT);
    gpio_set_direction (MOSFET2,GPIO_MODE_OUTPUT);
    gpio_set_direction (MOSFET3,GPIO_MODE_OUTPUT);

    gpio_set_level(TRIAC1,1);
    gpio_set_level(TRIAC2,1);
    gpio_set_level(TRIAC3,1);

    gpio_set_level(MOSFET1,1);
    gpio_set_level(MOSFET2,1);
    gpio_set_level(MOSFET3,1);

    init_gpio_intr();

    init_timer();
}

static void restart_system()
{
         // Restart module
    for (int i = 5; i >= 0; i--) {
        printf("Restarting in %d seconds...\n", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    printf("Restarting now.\n");
    fflush(stdout);
    esp_restart();
}

static bool save_data_wifi(char* _SSID, char* _PASS, bool _RES)
{   
    esp_err_t err;

    ESP_LOGI("wifi_data", "SSID:%s", _SSID);
    ESP_LOGI("wifi_data", "PASS:%s", _PASS);
    ESP_LOGI("wifi_data", "RES:%d",  _RES);

    err = nvs_open("storage", NVS_READWRITE, &app_nvs_handle);
    if (err != ESP_OK) {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
        return 0;
    }
    err =  nvs_set_str(app_nvs_handle, "SSID", _SSID );
    printf((err != ESP_OK) ? "Failed write SSID\n" : "Done write SSID\n");

    err =  nvs_set_str(app_nvs_handle, "PASS", _PASS );
    printf((err != ESP_OK) ? "Failed write PASS\n" : "Done write PASS\n");

    err =  nvs_set_u32(app_nvs_handle, "MagNum", MAGICNUMBER );
    printf((err != ESP_OK) ? "Failed write MAGICNUMBER\n" : "Done write MAGICNUMBER\n");

    // Commit written value.
    // After setting any values, nvs_commit() must be called to ensure changes are written
    // to flash storage. Implementations may write to storage at other times,
    // but this is not guaranteed.
    printf("Committing updates in NVS ... ");
    err = nvs_commit(app_nvs_handle);

    write_state_wifi(WIFI_STATE_STA);

    if(_RES == 1) 
    {
        restart_system();
    }
    return 1;
}

static bool read_data_wifi(uint8_t* WIFI_SSID_, uint8_t* WIFI_PASS_)
{
    esp_err_t err;
    size_t required_size;
    uint8_t WIFI_SSID[40];
    uint8_t WIFI_PASS[40];

    err = nvs_open("storage", NVS_READWRITE, &app_nvs_handle);
    if (err != ESP_OK) {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
        return 0;
    }
    ///////////////////////////////////////////////////////////////////////////// start read ssid
     err=nvs_get_str(app_nvs_handle, "SSID", NULL, &required_size);
        
        
        switch (err) {
            case ESP_OK:
                printf("Done read size wifi\n");
                break;
            case ESP_ERR_NVS_NOT_FOUND:
                printf("The value is not initialized yet! size\n");
                break;
            default :
                printf("Error (%s) reading!\n", esp_err_to_name(err));
        }

    
    
    err = nvs_get_str(app_nvs_handle, "SSID", &WIFI_SSID, &required_size);
        
        
        switch (err) {
            case ESP_OK:
                printf("Done read SSID\n");
                printf("SSID = %s\n", WIFI_SSID);
                break;
            case ESP_ERR_NVS_NOT_FOUND:
                printf("The value is not initialized yet SSID!\n");
                break;
            default :
                printf("Error (%s) reading!\n", esp_err_to_name(err));
        }

    memcpy(WIFI_SSID_, &WIFI_SSID, required_size);
    ////////////////////////////////////////////////////////////////////////////start read pass
     err=nvs_get_str(app_nvs_handle, "PASS", NULL, &required_size);
        
        
        switch (err) {
            case ESP_OK:
                printf("Done read size pass\n");
                break;
            case ESP_ERR_NVS_NOT_FOUND:
                printf("The value is not initialized yet! size pass\n");
                break;
            default :
                printf("Error (%s) reading!\n", esp_err_to_name(err));
        }

    err = nvs_get_str(app_nvs_handle, "PASS", &WIFI_PASS, &required_size);
        
        
        switch (err) {
            case ESP_OK:
                printf("Done read PASS\n");
                printf("PASS = %s\n", WIFI_PASS);
                break;
            case ESP_ERR_NVS_NOT_FOUND:
                printf("The value is not initialized yet PASS!\n");
                break;
            default :
                printf("Error (%s) reading!\n", esp_err_to_name(err));
        }

     memcpy(WIFI_PASS_, &WIFI_PASS, required_size);
    ////////////////////////////////////////////////////////////////////////////// start read magic number
    
    uint32_t WIFI_MagNum=0;
    err = nvs_get_u32(app_nvs_handle, "MagNum", &WIFI_MagNum);

    switch (err) {
            case ESP_OK:
                printf("Done read MagNum\n");
                // printf("MagNum = %d\n", WIFI_MagNum);
                break;
            case ESP_ERR_NVS_NOT_FOUND:
                printf("The value is not initialized yet MagNum!\n");
                break;
            default :
                printf("Error (%s) reading!\n", esp_err_to_name(err));
        }
    ////////////////////////////////////////////////////////////////////////////close nvs

    nvs_close(app_nvs_handle);

    if (WIFI_MagNum == MAGICNUMBER)
    {
        return 1;
    }
    
    return 0;
}

static void setMosfet(MOS_num_t num, uint8_t state)
{   

    if(state>0) state=1;

    switch (num)
    {
    case mos_1:
        gpio_set_level(MOSFET1,state); //HW nie dzila
        break;
    case mos_2:
        gpio_set_level(MOSFET2,state);  //HW nie dzila
        break;
    case mos_3:
        gpio_set_level(MOSFET3,state);
        break;

    }
}

static void setTRIAC(TRIAC_num_t num, uint8_t state)
{   

    if(state>10) state=10;

    switch (num)
    {
    case tri_1:
        if(state<10)
        {
            TRIAC_state.TRIAC1_state=true;
            TRIAC_state.TRIAC1_Del=state;
            enable_interr_cross();
        }
        else 
        {
            TRIAC_state.TRIAC1_state=false;
            TRIAC_state.TRIAC1_Del=10;
        }
        break;
    case tri_2:
        if(state<10)
        {
            TRIAC_state.TRIAC2_state=true;
            TRIAC_state.TRIAC2_Del=state;
            enable_interr_cross();
        }
        else 
        {
            TRIAC_state.TRIAC2_state=false;
            TRIAC_state.TRIAC2_Del=10;
        } 
        break;
    case tri_3:
        if(state<10)
        {
            TRIAC_state.TRIAC3_state=true;
            TRIAC_state.TRIAC3_Del=state;
            enable_interr_cross();
        }
        else 
        {
            TRIAC_state.TRIAC3_state=false;
            TRIAC_state.TRIAC3_Del=10;
        }
        break;

    }
    check_need_interr_cross();
}

static bool cJSON_parser_setstate(char *output_buffer)
{   

    cJSON *root = cJSON_Parse(output_buffer);
    
    if (root == NULL) 
    {
        ESP_LOGW("cJSON", "errorPars");
        return 0;
    }

    cJSON *TRIAC = cJSON_GetObjectItem(root, "TRIAC");

        cJSON *T_1 = cJSON_GetObjectItem(TRIAC, "T_1");
        ESP_LOGW("cJSON", "T_1:%d", T_1->valueint);
        
        if(check_en_sun_mode(tri_1))
        {
            disable_sun_mode();
        }
        
        setTRIAC(tri_1, T_1->valueint);

        
        cJSON *T_2 = cJSON_GetObjectItem(TRIAC, "T_2");
        ESP_LOGW("cJSON", "T_2:%d", T_2->valueint);

        if(check_en_sun_mode(tri_2))
        {
            disable_sun_mode();
        }

        setTRIAC(tri_2, T_2->valueint);

        
        cJSON *T_3 = cJSON_GetObjectItem(TRIAC, "T_3");
        ESP_LOGW("cJSON", "T_3:%d", T_3->valueint);

         if(check_en_sun_mode(tri_3))
        {
            disable_sun_mode();
        }

        setTRIAC(tri_3, T_3->valueint);

    cJSON *MOS = cJSON_GetObjectItem(root, "MOS");

        cJSON *M_1 = cJSON_GetObjectItem(MOS, "M_1");
        ESP_LOGW("cJSON", "M_1:%d", M_1->valueint);
        setMosfet(mos_1, M_1->valueint);

        cJSON *M_2 = cJSON_GetObjectItem(MOS, "M_2");
        ESP_LOGW("cJSON", "M_2:%d", M_2->valueint);
        setMosfet(mos_2, M_2->valueint);

        cJSON *M_3 = cJSON_GetObjectItem(MOS, "M_3");
        ESP_LOGW("cJSON", "M_3:%d", M_3->valueint);
        setMosfet(mos_3, M_3->valueint);    

    cJSON *Secur = cJSON_GetObjectItem(root, "Secur");
    ESP_LOGW("cJSON", "Secur:%d", Secur->valueint);

    cJSON *ADC_VCC = cJSON_GetObjectItem(root, "ADC_VCC");
    ESP_LOGW("cJSON", "ADC_VCC:%d", ADC_VCC->valueint);

    cJSON *OneWire = cJSON_GetObjectItem(root, "OneWire");
    ESP_LOGW("cJSON", "OneWire:%d", OneWire->valueint);

    cJSON *BMP280 = cJSON_GetObjectItem(root, "BMP280");
    ESP_LOGW("cJSON", "BMP280:%d", BMP280->valueint);

    cJSON *LCD = cJSON_GetObjectItem(root, "LCD");
    ESP_LOGW("cJSON", "LCD:%d", LCD->valueint); 
    

    // memcpy(mess_str.data_text, chat->valuestring, strlen(chat->valuestring));
    cJSON_Delete(root);
    return 1;
}

static void cJSON_parser_setwifi(char *output_buffer)
{
    cJSON *root = cJSON_Parse(output_buffer);
    
    if (root == NULL) 
    {
        ESP_LOGW("cJSON", "errorPars");
        // return 0;
    }

    
    cJSON *SSID = cJSON_GetObjectItem(root, "SSID");
    ESP_LOGW("cJSON", "SSID:%s", SSID->valuestring);
    // memcpy(mess_str.data_text, chat->valuestring, strlen(chat->valuestring));

    cJSON *PASS = cJSON_GetObjectItem(root, "PASS");
    ESP_LOGW("cJSON", "PASS:%s", PASS->valuestring);

    cJSON *RES = cJSON_GetObjectItem(root, "RES");
    ESP_LOGW("cJSON", "RES:%d", RES->valueint);

    save_data_wifi(SSID->valuestring, PASS->valuestring, RES->valueint);

    cJSON_Delete(root);
}

static bool cJSON_parser_setsunmode(char *output_buffer)
{
    cJSON *root = cJSON_Parse(output_buffer);
    
    if (root == NULL) 
    {
        ESP_LOGW("cJSON", "errorPars");
        // return 0;
    }

    cJSON *Chanal = cJSON_GetObjectItem(root, "Chanal");
    ESP_LOGW("cJSON", "Chanal:%d", Chanal->valueint);

    cJSON *Hour = cJSON_GetObjectItem(root, "Hour");
    ESP_LOGW("cJSON", "Hour:%d", Hour->valueint);

    cJSON *Min = cJSON_GetObjectItem(root, "Min");
    ESP_LOGW("cJSON", "Min:%d", Min->valueint);

    cJSON *Dur = cJSON_GetObjectItem(root, "Dur");
    ESP_LOGW("cJSON", "Dur:%d", Dur->valueint);

    sun_mode.chanal=Chanal->valueint;
    sun_mode.hour_start=Hour->valueint;
    sun_mode.min_start=Min->valueint;
    sun_mode.dur=Dur->valueint;
    sun_mode.state=true;

    cJSON_Delete(root);

    return 1;
}

static esp_err_t setwifi_post_handler(httpd_req_t *req)
{
    char buf[100];
    int ret, remaining = req->content_len;

    while (remaining > 0) {
        /* Read the data for the request */
        if ((ret = httpd_req_recv(req, buf,
                        MIN(remaining, sizeof(buf)))) <= 0) {
            if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
                /* Retry receiving if timeout occurred */
                continue;
            }
            return ESP_FAIL;
        }

        /* Send back the same data */
        httpd_resp_send_chunk(req, buf, ret);
        remaining -= ret;

        /* Log data received */
        ESP_LOGI(TAG, "=========== RECEIVED DATA ==========");
        ESP_LOGI(TAG, "%.*s", ret, buf);
        ESP_LOGI(TAG, "====================================");

       
    }

    // End response
    httpd_resp_send_chunk(req, NULL, 0);
    cJSON_parser_setwifi(&buf);
    return ESP_OK;
}

static esp_err_t setstate_post_handler(httpd_req_t *req)
{
    char buf[400];
    int ret, remaining = req->content_len;

    while (remaining > 0) {
        /* Read the data for the request */
        if ((ret = httpd_req_recv(req, buf,
                        MIN(remaining, sizeof(buf)))) <= 0) {
            if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
                /* Retry receiving if timeout occurred */
                continue;
            }
            return ESP_FAIL;
        }

        /* Send back the same data */
        httpd_resp_send_chunk(req, buf, ret);
        remaining -= ret;

        /* Log data received */
        ESP_LOGI(TAG, "=========== RECEIVED DATA ==========");
        ESP_LOGI(TAG, "%.*s", ret, buf);
        ESP_LOGI(TAG, "====================================");

       
    }

    // End response
    httpd_resp_send_chunk(req, NULL, 0);
    cJSON_parser_setstate(&buf);
    return ESP_OK;
}

static esp_err_t setsunmode_post_handler(httpd_req_t *req)
{
    char buf[400];
    int ret, remaining = req->content_len;

    while (remaining > 0) {
        /* Read the data for the request */
        if ((ret = httpd_req_recv(req, buf,
                        MIN(remaining, sizeof(buf)))) <= 0) {
            if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
                /* Retry receiving if timeout occurred */
                continue;
            }
            return ESP_FAIL;
        }

        /* Send back the same data */
        httpd_resp_send_chunk(req, buf, ret);
        remaining -= ret;

        /* Log data received */
        ESP_LOGI(TAG, "=========== RECEIVED DATA ==========");
        ESP_LOGI(TAG, "%.*s", ret, buf);
        ESP_LOGI(TAG, "====================================");

       
    }

    // End response
    httpd_resp_send_chunk(req, NULL, 0);
    cJSON_parser_setsunmode(&buf);
    return ESP_OK;
}


static const httpd_uri_t root = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = root_get_handler
};

static const httpd_uri_t setwifi = {
    .uri       = "/setwifi",
    .method    = HTTP_POST,
    .handler   = setwifi_post_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t setStateDevice = {
    .uri       = "/setstate",
    .method    = HTTP_POST,
    .handler   = setstate_post_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t setSunMode = {
    .uri       = "/setsunmode",
    .method    = HTTP_POST,
    .handler   = setsunmode_post_handler,
    .user_ctx  = NULL
};


static httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;

    // Start the httpd server
    ESP_LOGI(TAG, "Starting server");

    httpd_ssl_config_t conf = HTTPD_SSL_CONFIG_DEFAULT();

    extern const unsigned char cacert_pem_start[] asm("_binary_cacert_pem_start");
    extern const unsigned char cacert_pem_end[]   asm("_binary_cacert_pem_end");
    conf.cacert_pem = cacert_pem_start;
    conf.cacert_len = cacert_pem_end - cacert_pem_start;

    extern const unsigned char prvtkey_pem_start[] asm("_binary_prvtkey_pem_start");
    extern const unsigned char prvtkey_pem_end[]   asm("_binary_prvtkey_pem_end");
    conf.prvtkey_pem = prvtkey_pem_start;
    conf.prvtkey_len = prvtkey_pem_end - prvtkey_pem_start;

    #if CONFIG_EXAMPLE_ENABLE_HTTPS_USER_CALLBACK
    conf.user_cb = https_server_user_callback;
    #endif
    esp_err_t ret = httpd_ssl_start(&server, &conf);
    if (ESP_OK != ret) {
        ESP_LOGI(TAG, "Error starting server!");
        return NULL;
    }

    // Set URI handlers
    ESP_LOGI(TAG, "Registering URI handlers");
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &root));
    
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &setwifi));

    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &setStateDevice));

    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &setSunMode));

    // LCD_info_setwifi();

    return server;


}

static void stop_webserver(httpd_handle_t server)
{
    // Stop the httpd server
    httpd_ssl_stop(server);
}



static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                    int32_t event_id, void* event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        // ESP_LOGI(TAG, "station "MACSTR" join, AID=%d", MAC2STR(event->mac), event->aid);
                 
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        // ESP_LOGI(TAG, "station "MACSTR" leave, AID=%d",  MAC2STR(event->mac), event->aid);
    }
}


static void event_handler_sta(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}


EventBits_t wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler_sta,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler_sta,
                                                        NULL,
                                                        &instance_got_ip));



    wifi_config_t wifi_config = 
    {
    };
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.sta.pmf_cfg.capable = true;
    wifi_config.sta.pmf_cfg.required = false;
    // wifi_config.sta.ssid=ESP_WIFI_SSID;
    // wifi_config.sta.password="hAslo1313@@";
    if(!read_data_wifi(&wifi_config.sta.ssid, &wifi_config.sta.password))
         return 0;
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s. password:%s.",
                 wifi_config.sta.ssid, wifi_config.sta.password);
                 return bits;
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s. password:%s.",
                 wifi_config.sta.ssid, wifi_config.sta.password);
                return bits;
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
    return 1;
}

static void LCD_info_sofrap()
{
    ST7789_WriteCommand (SPIhandle,ST7789_DISPOFF);

	ST7789_Fill_Color(SPIhandle, 0xBDF8);
    ST7789_WriteString(SPIhandle, 5, 10, "TO CONECT WIFI", Font_16x26, MAGENTA, 0xBDF8);
    ST7789_WriteString(SPIhandle, 30, 50, "SSID: ESP_EXAM ", Font_11x18, MAGENTA, 0xBDF8);
    
    ST7789_WriteString(SPIhandle, 30, 70, "PASS: 12345678 ", Font_11x18, MAGENTA, 0xBDF8);
    ST7789_WriteCommand (SPIhandle,ST7789_DISPON);
}


static void LCD_info_setwifi()
{
    ST7789_WriteCommand (SPIhandle,ST7789_DISPOFF);

	ST7789_Fill_Color(SPIhandle, 0xBDF8);
    ST7789_WriteString(SPIhandle, 5, 10, "Сonfigure WIFI", Font_16x26, MAGENTA, 0xBDF8);
    ST7789_WriteString(SPIhandle, 30, 50, "IP: 192.168.4.1 ", Font_11x18, MAGENTA, 0xBDF8);
    
    ST7789_WriteString(SPIhandle, 30, 70, "ADD POST REQ: /setwifi ", Font_11x18, MAGENTA, 0xBDF8);
    
    ST7789_WriteString(SPIhandle, 50, 70, "Body exam: { ", Font_11x18, MAGENTA, 0xBDF8);
    
    ST7789_WriteString(SPIhandle, 70, 70, "*SSID*:**,", Font_11x18, MAGENTA, 0xBDF8);
    ST7789_WriteCommand (SPIhandle,ST7789_DISPON);
}


void wifi_init_softap(void)
{
   
     ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = ESP_WIFI_SSID,
            .ssid_len = strlen(ESP_WIFI_SSID),
            .channel = EXAMPLE_ESP_WIFI_CHANNEL,
            .password = ESP_WIFI_PASS,
            .max_connection = EXAMPLE_MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK,
            // .pmf_cfg = {
            //         .required = false,
            // },
        },
    };
    if (strlen(ESP_WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_softap finished. SSID:%s password:%s channel:%d",
             ESP_WIFI_SSID, ESP_WIFI_PASS, EXAMPLE_ESP_WIFI_CHANNEL);

    LCD_info_sofrap();
}

static bool write_state_wifi(uint8_t WIFI_state)
{
    esp_err_t err;

    err = nvs_open("storage", NVS_READWRITE, &app_nvs_handle);
    if (err != ESP_OK) {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
        return 0;
    }   

    err = nvs_set_u8(app_nvs_handle, "WIFI_state", WIFI_state);
    printf((err != ESP_OK) ? "Failed write WIFI_state\n" : "Done write WIFI_state\n");

    printf("Committing updates in NVS ... ");
    err = nvs_commit(app_nvs_handle);
     printf((err != ESP_OK) ? "Failed commit WIFI_state\n" : "Done commit WIFI_state\n");

    nvs_close(app_nvs_handle);
    return 1;
}

static uint8_t read_state_wifi()
{
    esp_err_t err;

    err = nvs_open("storage", NVS_READWRITE, &app_nvs_handle);
    if (err != ESP_OK) {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
        return 0;
    }    
    
    uint32_t WIFI_state=WIFI_STATE_AP;
    err = nvs_get_u8(app_nvs_handle, "WIFI_state", &WIFI_state);

     switch (err) {
            case ESP_OK:
                printf("Done read WIFI_state\n");
                printf("WIFI_state = %d\n", WIFI_state);
                break;
            case ESP_ERR_NVS_NOT_FOUND:
                printf("The value is not initialized yet WIFI_state!\n");
                break;
            default :
                printf("Error (%s) reading!\n", esp_err_to_name(err));
        }

    

    nvs_close(app_nvs_handle);

   return WIFI_state;

}

static void check_need_interr_cross()
{
    if((!TRIAC_state.TRIAC1_state) && (!TRIAC_state.TRIAC2_state) && (!TRIAC_state.TRIAC3_state))
    {
    gpio_intr_disable(CROSSIN);
    ESP_LOGI(TAG, "INTR CROSS DISABLE");
    }
}

static void enable_interr_cross()
{
    gpio_intr_enable(CROSSIN);
    ESP_LOGI(TAG, "INTR CROSS ENABLE");
}

static bool check_en_sun_mode(TRIAC_num_t num)
{
    if(sun_mode.state==true)
    {
        if(sun_mode.chanal==num)
        ESP_LOGE(TAG, "sun_mode is on");
        return 1;
    }
    else return 0;
}

static void disable_sun_mode()
{
    ESP_LOGE(TAG, "sun_mode is off");
    sun_mode.sun_on=false;
}

static void sun_task()
{   
    bool i=1;
    time_t now_sun;
    struct tm timeinfo_sun;
    uint8_t counter=9;
    while(1)
    {

        while (i)
        {   
            time(&now);
            localtime_r(&now, &timeinfo);
            ESP_LOGE(TAG, "hours=%d",timeinfo.tm_hour);
            
            ESP_LOGE(TAG, "hours=%d",timeinfo.tm_min);

            ESP_LOGE(TAG, "hours_set=%d",sun_mode.hour_start);

            if(sun_mode.hour_start==timeinfo.tm_hour)
            {
                ESP_LOGE(TAG, "HOURS OK");
                if(sun_mode.min_start==timeinfo.tm_min)
                {
                    ESP_LOGE(TAG, "MIN OK");
                    sun_mode.sun_on=true;
                    i=0;
                    break;
                    }
                else ESP_LOGE(TAG, "MIN bad");
            }       
            else ESP_LOGE(TAG, "hours bad");

            vTaskDelay(59000/ portTICK_PERIOD_MS);
            }
            
            
            
            
            while(sun_mode.sun_on)
            {   
                uint32_t delay_time=(sun_mode.dur/9)*60000;
                    ESP_LOGE(TAG, "start sunmode=%d, %d",counter, delay_time);  
                
                if(counter==9)
                {
                    switch (sun_mode.chanal)
                    {
                    case 1:
                        TRIAC_state.TRIAC1_state=true;
                        break;
                    case 2:
                        TRIAC_state.TRIAC2_state=true;
                        break;
                    case 3:
                        TRIAC_state.TRIAC3_state=true;
                        break;
                    
                    
                    }

                    enable_interr_cross();
                }

                
                 switch (sun_mode.chanal)
                    {
                    case 1:
                        TRIAC_state.TRIAC1_Del=counter;
                        break;
                    case 2:
                        TRIAC_state.TRIAC2_Del=counter;
                        break;
                    case 3:
                        TRIAC_state.TRIAC3_Del=counter;
                        break;
                    
                    
                    }
                
                if (counter==1) 
                {
                    sun_mode.sun_on=false;
                    i=1;
                    
                    break;
                }
                counter--;
                vTaskDelay(delay_time/ portTICK_PERIOD_MS);

                i=1;
            }

            counter=9;
        
    
    }
    // vTaskDelete(NULL);
}

void sun_mode_start(uint8_t dur)
{
     xTaskCreate(sun_task, "sun_task", 2024, NULL, 1, &xSun_task_handl);
}

void time_sync_notification_cb(struct timeval *tv)
{
    ESP_LOGI(TAG, "Notification of a time synchronization event");
}

static void initialize_sntp(void)
{
    ESP_LOGI(TAG, "Initializing SNTP");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_set_time_sync_notification_cb(time_sync_notification_cb);

    sntp_init();
}

static void obtain_time(void)
{
    initialize_sntp();

    // wait for time to be set
   
    int retry = 0;
    const int retry_count = 10;
    while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < retry_count) {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
    // time(&now);
    // localtime_r(&now, &timeinfo);
}

void get_time()
{
    
    obtain_time();
    time(&now);

    char strftime_buf[64];

    // Set timezone to Eastern Standard Time and print local time
     setenv("TZ", "MST7MDT,M3.2.0/2,M11.1.0", 1);
     tzset();
    localtime_r(&now, &timeinfo);
    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
    ESP_LOGI(TAG, "The current date/time in Calgary is: %s", strftime_buf);
}


void app_main(void)
{   
    char strftime_buf[64];
    initHW();
  spi_master_init(&SPIhandle);
	ST7789_Init(&SPIhandle);
   
    
	
	
    static httpd_handle_t server = NULL;
 esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);  

 

    uint8_t WIFI_state = read_state_wifi();
   
    if  (WIFI_state == WIFI_STATE_STA) 
    {
        if(wifi_init_sta()==WIFI_FAIL_BIT)
        {
            ESP_LOGE(TAG, "CON ERROR");
            write_state_wifi(WIFI_STATE_AP);
            restart_system();
        }  
    }
    else 
        if  (WIFI_state == WIFI_STATE_AP) 
        {
            wifi_init_softap();
        }               
        
         start_webserver();


get_time();
sun_mode_start(5);
   while(1)
   {
    
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    // time(&now);
    // localtime_r(&now, &timeinfo);
    // strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
    // ESP_LOGI(TAG, "The current date/time in Calgary is: %s", strftime_buf);
    // ESP_LOGE(TAG, "hours=%d",timeinfo.tm_hour);
    
    }
 



    
}
