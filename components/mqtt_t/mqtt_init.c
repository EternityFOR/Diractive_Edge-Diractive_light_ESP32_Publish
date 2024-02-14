
//***Header files***
#include <stdio.h>
#include <string.h>
#include "mqtt_client.h"
#include "esp_log.h"
#include "mqtt_init.h"
#include "sign_api.h"
#include "radar_data_handle.h"
#include "cJSON.h"

#include "light_control.h"
#include "main.h"

//***Parameters***

//Tags
#define MQTT_TAG "MQTT"

//Mqtt connection params,topics and client
#define PRODUCT_KEY       "..."
#define PRODUCT_SECRET    "..."
#define DEVICE_SECRET     "..."

const char LIGHT_CONTROL_TOPIC[] =       "...";//Light control topic
const char RADAR_DATA_RECEIVE_TOPIC[] =  "...";//LD2410 data receive
esp_mqtt_client_handle_t client_mqtt;

//Light control params
struct __Light_ctr_data_mqtt{
    int dataLen;
    char allData[1024];
} light_ctr_data_mqtt;

//***Functions***

//Functions
static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(MQTT_TAG, "Last error %s: 0x%x", message, error_code);
    }
}

/*
 * @brief Event handler registered to receive MQTT events
 *
 *  This function is called by the MQTT client event loop.
 *
 * @param handler_args user data registered to the event.
 * @param base Event base for the handler(always MQTT Base in this example).
 * @param event_id The id for the received event.
 * @param event_data The data for the event, esp_mqtt_event_handle_t.
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(MQTT_TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    client_mqtt = event->client;
    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(MQTT_TAG, "MQTT_EVENT_CONNECTED");
            uint8_t msg_id0 = esp_mqtt_client_subscribe(client_mqtt, LIGHT_CONTROL_TOPIC, 1);
            ESP_LOGI(MQTT_TAG, "sent subscribe successful, msg_id=%d", msg_id0);
            radar_init(client_mqtt);
            uint8_t msg_id1 = esp_mqtt_client_subscribe(client_mqtt, RADAR_DATA_RECEIVE_TOPIC, 1);
            ESP_LOGI(MQTT_TAG, "sent subscribe successful, msg_id=%d", msg_id1);
            // msg_id = esp_mqtt_client_unsubscribe(client, "/topic/qos1");
            // ESP_LOGI(MQTT_TAG, "sent unsubscribe successful, msg_id=%d", msg_id);
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(MQTT_TAG, "MQTT_EVENT_DISCONNECTED");
            break;
        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(MQTT_TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(MQTT_TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(MQTT_TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGI(MQTT_TAG,"TOPIC=%.*s", event->topic_len, event->topic);
            ESP_LOGI(MQTT_TAG,"DATA=%.*s  \r\n", event->data_len, event->data);
            struct __Light_ctr_data_mqtt *data;
            sprintf(light_ctr_data_mqtt.allData, "%s", event->data);
            data = &light_ctr_data_mqtt;
            light_ctr_data_mqtt.dataLen = event->data_len;
            ////首先整体判断是否为一个json格式的数据
            cJSON *root = cJSON_Parse(data->allData);
            //如果是否json格式数据
            if (root == NULL)
            {
                ESP_LOGI(MQTT_TAG,"[SY] Task_ParseJSON_Message xQueueReceive not json ... \n");
                goto __cJSON_Delete;
            }
            if (strncmp(event->topic, LIGHT_CONTROL_TOPIC, event->topic_len) == 0) {
                cJSON *JSON_Item_Red = cJSON_GetObjectItem(root, "red");
                cJSON *JSON_Item_Green = cJSON_GetObjectItem(root, "green");
                cJSON *JSON_Item_Blue = cJSON_GetObjectItem(root, "blue");
                if((JSON_Item_Red->valueint == 1) & (JSON_Item_Green->valueint == 0) & (JSON_Item_Blue->valueint == 0)){
                }else{
                    set_rgb(JSON_Item_Red->valueint, JSON_Item_Green->valueint, JSON_Item_Blue->valueint);
                    ESP_LOGI(MQTT_TAG,"Set rgb,controlled by app");
                }
            }
            else if (strncmp(event->topic, RADAR_DATA_RECEIVE_TOPIC, event->topic_len) == 0){
                cJSON *JSON_Item_State = cJSON_GetObjectItem(root,"目标状态");
                if(JSON_Item_State->valueint == 0)
                {
                    set_rgb(0,0,0);  
                }
                else if(JSON_Item_State->valueint == 1)
                {
                    set_rgb(255,25,10);  
                }
                else if(JSON_Item_State->valueint == 2)
                {
                    set_rgb(67,255,10); 
                }
                else if(JSON_Item_State->valueint == 3)
                {
                    set_rgb(255,150,0); 
                }
                ESP_LOGI(MQTT_TAG,"Set rgb,another radar state update");
            }  
            __cJSON_Delete:
                    cJSON_Delete(root);
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGI(MQTT_TAG, "MQTT_EVENT_ERROR");
            if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
                log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
                log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
                log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
                ESP_LOGI(MQTT_TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));

            }
            break;
        default:
            ESP_LOGI(MQTT_TAG, "Other event id:%d", event->event_id);
            break;
    }
}

//Initialize mqtt
void mqtt_app_start(void)
{
    // esp_mqtt_client_config_t mqtt_cfg = {
    //     .broker.address.uri = CONFIG_BROKER_URL,
    // };
#if CONFIG_BROKER_URL_FROM_STDIN
    char line[128];

    if (strcmp(mqtt_cfg.broker.address.uri, "FROM_STDIN") == 0) {
        int count = 0;
        ESP_LOGI(MQTT_TAG,"Please enter url of mqtt broker\n");
        while (count < 128) {
            int c = fgetc(stdin);
            if (c == '\n') {
                line[count] = '\0';
                break;
            } else if (c > 0 && c < 127) {
                line[count] = c;
                ++count;
            }
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        mqtt_cfg.broker.address.uri = line;
        ESP_LOGI(MQTT_TAG"Broker url: %s\n", line);
    } else {
        ESP_LOGE(MQTT_TAG, "Configuration mismatch: wrong broker url");
        abort();
    }
#endif /* CONFIG_BROKER_URL_FROM_STDIN */
    iotx_dev_meta_info_t meta_info;
    iotx_sign_mqtt_t sign_mqtt;
    memset(&meta_info, 0, sizeof(iotx_dev_meta_info_t));
    memcpy(meta_info.product_key, PRODUCT_KEY, strlen(PRODUCT_KEY));
    memcpy(meta_info.product_secret, PRODUCT_SECRET, strlen(PRODUCT_SECRET));
    memcpy(meta_info.device_name, DEVICE_NAME, strlen(DEVICE_NAME));
    memcpy(meta_info.device_secret, DEVICE_SECRET, strlen(DEVICE_SECRET));
    IOT_Sign_MQTT(IOTX_CLOUD_REGION_SHANGHAI, &meta_info, &sign_mqtt);
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = "mqtt://iot-06z00cttbpxocf9.mqtt.iothub.aliyuncs.com",
        .broker.address.hostname = sign_mqtt.hostname,
        .broker.address.port = 1883,
        .credentials.authentication.password = sign_mqtt.password,
        .credentials.client_id = sign_mqtt.clientid,
        .credentials.username = sign_mqtt.username,
    };
    // ESP_LOGI("MQTT_connect","%s",mqtt_cfg.broker.address.hostname);
    // ESP_LOGI("MQTT_connect","%d",mqtt_cfg.broker.address.port);
    // ESP_LOGI("MQTT_connect","%s",mqtt_cfg.credentials.authentication.password);
    // ESP_LOGI("MQTT_connect","%s",mqtt_cfg.credentials.client_id);
    // ESP_LOGI("MQTT_connect","%s",mqtt_cfg.credentials.username);
    client_mqtt = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client_mqtt, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client_mqtt);
}
