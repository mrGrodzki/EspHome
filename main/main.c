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

static const char *TAG = "example";



static int s_retry_num = 0;

static EventGroupHandle_t s_wifi_event_group;

 nvs_handle_t app_nvs_handle;
typedef struct 
{
    char* SSID;
    char* PASS;
    bool  RES;
    int   MagicNumber;
}wifi_data_strc;


static bool write_state_wifi(uint8_t WIFI_state);


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
                printf("MagNum = %d\n", WIFI_MagNum);
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

static void cJSON_parser_setwifi(char *output_buffer)
{
    cJSON *root = cJSON_Parse(output_buffer);
    
    if (root == NULL) 
    {
        ESP_LOGW("cJSON", "errorPars");
        return 0;
    }

    
    cJSON *SSID = cJSON_GetObjectItem(root, "SSID");
    ESP_LOGW("cJSON", "SSID:%s", SSID->valuestring);
    // memcpy(mess_str.data_text, chat->valuestring, strlen(chat->valuestring));

    cJSON *PASS = cJSON_GetObjectItem(root, "PASS");
    ESP_LOGW("cJSON", "PASS:%s", PASS->valuestring);

    cJSON *RES = cJSON_GetObjectItem(root, "RES");
    ESP_LOGW("cJSON", "RES:%d", RES->valueint);

    save_data_wifi(SSID->valuestring, PASS->valuestring, RES->valueint);
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

        cJSON_parser_setwifi(&buf);
    }

    // End response
    httpd_resp_send_chunk(req, NULL, 0);
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
        ESP_LOGI(TAG, "station "MACSTR" join, AID=%d",
                 MAC2STR(event->mac), event->aid);
                 
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" leave, AID=%d",
                 MAC2STR(event->mac), event->aid);
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

void app_main(void)
{
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
 
  

    
}
