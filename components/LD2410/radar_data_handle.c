//***Header files***
#include <string.h>
#include <time.h>
#include "esp_system.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "mqtt_client.h"
#include "cJSON.h"

#include "main.h"

//***Parameters***

//Engineering_mode_commands
char config_LD2410_cmd[14] = {0xFD, 0xFC, 0xFB, 0xFA, 0x04, 0x00, 0xFF, 0x00, 0x01, 0x00, 0x04, 0x03, 0x02, 0x01};
char open_engineering_pattern_cmd[12] = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0x62, 0x00, 0x04, 0x03, 0x02, 0x01};
char unconfig_LD2410_cmd[14] = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0xFE, 0x00, 0x04, 0x03, 0x02, 0x01};

//Mqtt topic and client
const char RADAR_DATA_UPDATE_TOPIC[] = "...";//LD2410 data update

struct radar_to_esp32_task_param{
    esp_mqtt_client_handle_t client_radar;
};

//Uart port,pin,task and queue
#define UART_PORT_NUM1 UART_NUM_1
#define TXD_PIN (GPIO_NUM_18) //yellow, data sent from esp32 to radar
#define RXD_PIN (GPIO_NUM_19) //green,  data sent from radar to esp32
static TaskHandle_t xUARTEventTaskHandle1 = NULL;
static QueueHandle_t uart1_queue;

//Tags
#define RADAR_ENG_MODE_TAG "RADAR_ENG_MODE"
#define MEG_GENERATE_TAG   "MEG_GENERATE"
#define ESP32_TO_MQTT_TAG  "ESP32_TO_MQTT"
#define ESP32_TO_RADAR_TAG "ESP32_TO_RADAR"
#define RADAR_TO_ESP32_TAG "RADAR_TO_ESP32"

//Buff size
#define MAX_COUNT 50
static const uint16_t MAX_BUFF_NUM = 45* 50;
static const uint16_t BUF_SIZE = 2048;

//Json parameters
cJSON *root;
typedef struct {
cJSON *frame_length_array;
cJSON *radar_mode_array;
cJSON *state_array;
cJSON *active_dis_array;
cJSON *active_eng_array;
cJSON *still_dis_array;
cJSON *still_eng_array;
cJSON *detect_dis_array;
cJSON *max_activedoor_array;
cJSON *max_stilldoor_array;
cJSON *active_door0_array;
cJSON *active_door1_array;
cJSON *active_door2_array;
cJSON *active_door3_array;
cJSON *active_door4_array;
cJSON *active_door5_array;
cJSON *active_door6_array;
cJSON *active_door7_array;
cJSON *still_door0_array;
cJSON *still_door1_array;
cJSON *still_door2_array;
cJSON *still_door3_array;
cJSON *still_door4_array;
cJSON *still_door5_array;
cJSON *still_door6_array;
cJSON *still_door7_array;
}cJSON_struct;
cJSON_struct JSON_array;


//***Functions***

//Functions declaration
static void esp32_to_radar(char* data, uint16_t length);
static void radar_to_esp32(void *pvParameters);
static void esp32_to_mqtt(char* data, esp_mqtt_client_handle_t client);

//Uart_init
static void uart_init(void){
    uart_config_t uart_config = {
        .baud_rate = 256000,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    }; 
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_1, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart1_queue, 0);
}

//JSON_message_generate
void init_JSON(){
    JSON_array.frame_length_array = cJSON_CreateArray();
    JSON_array.radar_mode_array = cJSON_CreateArray();
    JSON_array.state_array = cJSON_CreateArray();
    JSON_array.active_dis_array = cJSON_CreateArray();
    JSON_array.active_eng_array = cJSON_CreateArray();
    JSON_array.still_dis_array = cJSON_CreateArray();
    JSON_array.still_eng_array = cJSON_CreateArray();
    JSON_array.detect_dis_array = cJSON_CreateArray();
    JSON_array.max_activedoor_array = cJSON_CreateArray();
    JSON_array.max_stilldoor_array = cJSON_CreateArray();
    JSON_array.active_door0_array = cJSON_CreateArray();
    JSON_array.active_door1_array = cJSON_CreateArray();
    JSON_array.active_door2_array = cJSON_CreateArray();
    JSON_array.active_door3_array = cJSON_CreateArray();
    JSON_array.active_door4_array = cJSON_CreateArray();
    JSON_array.active_door5_array = cJSON_CreateArray();
    JSON_array.active_door6_array = cJSON_CreateArray();
    JSON_array.active_door7_array = cJSON_CreateArray();
    JSON_array.still_door0_array = cJSON_CreateArray();
    JSON_array.still_door1_array = cJSON_CreateArray();
    JSON_array.still_door2_array = cJSON_CreateArray();
    JSON_array.still_door3_array = cJSON_CreateArray();
    JSON_array.still_door4_array = cJSON_CreateArray();
    JSON_array.still_door5_array = cJSON_CreateArray();
    JSON_array.still_door6_array = cJSON_CreateArray();
    JSON_array.still_door7_array = cJSON_CreateArray();
    root = cJSON_CreateObject();
    cJSON_AddItemToObject(root, "帧内数据长度", JSON_array.frame_length_array);
    cJSON_AddItemToObject(root, "雷达模式", JSON_array.radar_mode_array);
    cJSON_AddItemToObject(root, "目标状态", JSON_array.state_array);
    cJSON_AddItemToObject(root, "运动目标距离", JSON_array.active_dis_array);
    cJSON_AddItemToObject(root, "运动目标能量值", JSON_array.active_eng_array);
    cJSON_AddItemToObject(root, "静止目标距离", JSON_array.still_dis_array);
    cJSON_AddItemToObject(root, "静止目标能量值", JSON_array.still_eng_array);
    cJSON_AddItemToObject(root, "探测距离", JSON_array.detect_dis_array);
    cJSON_AddItemToObject(root, "最大运动距离门", JSON_array.max_activedoor_array);
    cJSON_AddItemToObject(root, "最大静止距离门", JSON_array.max_stilldoor_array);
    cJSON_AddItemToObject(root, "运动距离门0能量值", JSON_array.active_door0_array);
    cJSON_AddItemToObject(root, "运动距离门1能量值", JSON_array.active_door1_array);
    cJSON_AddItemToObject(root, "运动距离门2能量值", JSON_array.active_door2_array);
    cJSON_AddItemToObject(root, "运动距离门3能量值", JSON_array.active_door3_array);
    cJSON_AddItemToObject(root, "运动距离门4能量值", JSON_array.active_door4_array);
    cJSON_AddItemToObject(root, "运动距离门5能量值", JSON_array.active_door5_array);
    cJSON_AddItemToObject(root, "运动距离门6能量值", JSON_array.active_door6_array);
    cJSON_AddItemToObject(root, "运动距离门7能量值", JSON_array.active_door7_array);
    cJSON_AddItemToObject(root, "静止距离门0能量值", JSON_array.still_door0_array);
    cJSON_AddItemToObject(root, "静止距离门1能量值", JSON_array.still_door1_array);
    cJSON_AddItemToObject(root, "静止距离门2能量值", JSON_array.still_door2_array);
    cJSON_AddItemToObject(root, "静止距离门3能量值", JSON_array.still_door3_array);
    cJSON_AddItemToObject(root, "静止距离门4能量值", JSON_array.still_door4_array);
    cJSON_AddItemToObject(root, "静止距离门5能量值", JSON_array.still_door5_array);
    cJSON_AddItemToObject(root, "静止距离门6能量值", JSON_array.still_door6_array);
    cJSON_AddItemToObject(root, "静止距离门7能量值", JSON_array.still_door7_array);
}
void CreateJson(uint8_t* data,uint16_t length) {
    // 创建JSON对象
    uint16_t j = 0;
    while (j < length) {
        if((data[0 + j]== 244) && (data[1 + j] == 243) && (data[2 + j] == 242) && (data[3 + j] == 241)){
            // Add array data
            cJSON_AddItemToArray(cJSON_GetObjectItem(root,"帧内数据长度"), cJSON_CreateNumber(data[4 + j]));
            cJSON_AddItemToArray(cJSON_GetObjectItem(root,"雷达模式"), cJSON_CreateNumber(data[6 + j]));
            cJSON_AddItemToArray(cJSON_GetObjectItem(root,"目标状态"), cJSON_CreateNumber(data[8 + j]));
            cJSON_AddItemToArray(cJSON_GetObjectItem(root,"运动目标距离"), cJSON_CreateNumber(data[9 + j]));
            cJSON_AddItemToArray(cJSON_GetObjectItem(root,"运动目标能量值"), cJSON_CreateNumber(data[11 + j]));
            cJSON_AddItemToArray(cJSON_GetObjectItem(root,"静止目标距离" ), cJSON_CreateNumber(data[12 + j]));
            cJSON_AddItemToArray(cJSON_GetObjectItem(root,"静止目标能量值"), cJSON_CreateNumber(data[14 + j]));
            cJSON_AddItemToArray(cJSON_GetObjectItem(root,"探测距离"), cJSON_CreateNumber(data[15 + j]));
            cJSON_AddItemToArray(cJSON_GetObjectItem(root,"最大运动距离门"), cJSON_CreateNumber(data[17 + j]));
            cJSON_AddItemToArray(cJSON_GetObjectItem(root,"最大静止距离门"), cJSON_CreateNumber(data[18 + j]));
            cJSON_AddItemToArray(cJSON_GetObjectItem(root,"运动距离门0能量值"), cJSON_CreateNumber(data[19 + j]));
            cJSON_AddItemToArray(cJSON_GetObjectItem(root,"运动距离门1能量值"), cJSON_CreateNumber(data[20 + j]));
            cJSON_AddItemToArray(cJSON_GetObjectItem(root,"运动距离门2能量值"), cJSON_CreateNumber(data[21 + j]));
            cJSON_AddItemToArray(cJSON_GetObjectItem(root,"运动距离门3能量值"), cJSON_CreateNumber(data[22 + j]));
            cJSON_AddItemToArray(cJSON_GetObjectItem(root,"运动距离门4能量值"), cJSON_CreateNumber(data[23 + j]));
            cJSON_AddItemToArray(cJSON_GetObjectItem(root,"运动距离门5能量值"), cJSON_CreateNumber(data[24 + j]));
            cJSON_AddItemToArray(cJSON_GetObjectItem(root,"运动距离门6能量值"), cJSON_CreateNumber(data[25 + j]));
            cJSON_AddItemToArray(cJSON_GetObjectItem(root,"运动距离门7能量值"), cJSON_CreateNumber(data[26 + j]));
            cJSON_AddItemToArray(cJSON_GetObjectItem(root,"静止距离门0能量值"), cJSON_CreateNumber(data[27+ j]));
            cJSON_AddItemToArray(cJSON_GetObjectItem(root,"静止距离门1能量值"), cJSON_CreateNumber(data[28+ j]));
            cJSON_AddItemToArray(cJSON_GetObjectItem(root,"静止距离门2能量值"), cJSON_CreateNumber(data[29+ j]));
            cJSON_AddItemToArray(cJSON_GetObjectItem(root,"静止距离门3能量值"), cJSON_CreateNumber(data[30+ j]));
            cJSON_AddItemToArray(cJSON_GetObjectItem(root,"静止距离门4能量值"), cJSON_CreateNumber(data[31+ j]));
            cJSON_AddItemToArray(cJSON_GetObjectItem(root,"静止距离门5能量值"), cJSON_CreateNumber(data[32+ j]));
            cJSON_AddItemToArray(cJSON_GetObjectItem(root,"静止距离门6能量值"), cJSON_CreateNumber(data[33+ j]));
            cJSON_AddItemToArray(cJSON_GetObjectItem(root,"静止距离门7能量值"), cJSON_CreateNumber(data[34+ j]));
            // Move to the next expected frame start
            j += 45;
        } else {
            // Frame start not found where expected, move forward by one byte
            j++;
        }
    }
    char *json_str = cJSON_Print(root);
    ESP_LOGI(MEG_GENERATE_TAG,"%s", json_str);
    free(json_str);
}
cJSON* create_json_object() {
    cJSON *json = cJSON_CreateObject();
    cJSON_AddStringToObject(json, "deviceName", DEVICE_NAME);

    srand(time(NULL));
    int messageId = rand();
    cJSON_AddNumberToObject(json, "messageId", messageId);

    cJSON *data = cJSON_CreateObject();
    cJSON_AddItemToObject(json, "data", data);

    return json;
}
void add_data_to_json(cJSON *json, const char *key, int value) {
    cJSON *data = cJSON_GetObjectItem(json, "data");
    if (data != NULL) {
        cJSON_AddNumberToObject(data, key, value);
    }
}
char* json_to_string(cJSON *json) {
    char *json_string = cJSON_PrintUnformatted(json);
    return json_string;
}
void free_json_object(cJSON *json) {
    cJSON_Delete(json);
}

//Turn on LD2410 engineering mode
void open_radar_engineering_mode()
{
    uint8_t* read_buf = (uint8_t*) malloc(BUF_SIZE);
    ESP_LOGI(RADAR_ENG_MODE_TAG,"try to open LD2410 engineering mode");
    esp32_to_radar(config_LD2410_cmd, sizeof(config_LD2410_cmd));
    vTaskDelay(100 / portTICK_PERIOD_MS);
    esp32_to_radar(open_engineering_pattern_cmd, sizeof(open_engineering_pattern_cmd));
    vTaskDelay(100 / portTICK_PERIOD_MS);
    esp32_to_radar(unconfig_LD2410_cmd, sizeof(unconfig_LD2410_cmd));
    vTaskDelay(100 / portTICK_PERIOD_MS);
    free(read_buf);
}

//ESP_RADAR_Communications
static void esp32_to_radar(char* data, uint16_t length)
{
    const uint16_t tx_Bytes = uart_write_bytes(UART_NUM_1, data, length);
    ESP_LOGI(RADAR_ENG_MODE_TAG, "Wrote %d bytes", tx_Bytes);
}
static void radar_to_esp32(void *pvParameters){
    struct radar_to_esp32_task_param *param = (struct radar_to_esp32_task_param *)pvParameters;
    esp_mqtt_client_handle_t client_radar = param->client_radar;
    uart_event_t event;
	uint8_t* data = (uint8_t*) malloc(MAX_BUFF_NUM);
    while(1) {
        //Waiting for UART event.
        if(xQueueReceive(uart1_queue, (void * )&event, (TickType_t)portMAX_DELAY)) {
            ESP_LOGI(RADAR_TO_ESP32_TAG, "uart[%d](receive) event:", UART_PORT_NUM1);
            switch(event.type) {
                case UART_DATA:
                    //read data from buffer
                    const uint16_t rxBytes = uart_read_bytes(UART_NUM_1, data, MAX_BUFF_NUM, portMAX_DELAY);
                    if (rxBytes > 0) {
                        ESP_LOGI(RADAR_TO_ESP32_TAG, "Read %d bytes", rxBytes);
                        //ESP_LOG_BUFFER_HEXDUMP(RADAR_TO_ESP32_TAG, data, rxBytes, ESP_LOG_INFO);
                        init_JSON();
                        CreateJson(data, rxBytes);
                        cJSON* targetState_array = cJSON_GetObjectItem(root, "目标状态");
                        if (targetState_array == NULL || targetState_array->type != cJSON_Array) {
                            ESP_LOGI(RADAR_TO_ESP32_TAG,"Invalid JSON format.\n");
                            goto _cJSON_Delete;
                        }
                        // Count the occurrences of each number
                        uint16_t count[MAX_COUNT] = {0};
                        uint16_t maxCount = 0;
                        uint16_t mostFrequentNumber = 0;
                        uint16_t size = cJSON_GetArraySize(targetState_array);
                        for (uint16_t i = 0; i < size; i++) {
                            uint16_t number = cJSON_GetArrayItem(targetState_array, i)->valueint;
                            count[number]++;
                            if (count[number] > maxCount) {
                                maxCount = count[number];
                                mostFrequentNumber = number;
                            }
                        }
                        if (mostFrequentNumber == 3) {
                            mostFrequentNumber = 1;
                        }
                        // Create JSON object with the most frequent number
                        cJSON* result = create_json_object();
                        // Add data to object
                        add_data_to_json(result,"目标状态",mostFrequentNumber);                
                        // Convert JSON object to string
                        char* result_str = json_to_string(result);
                        // Output the result
                        ESP_LOGI(RADAR_TO_ESP32_TAG,"%s\n", result_str);
                        esp32_to_mqtt(result_str, client_radar);
                        // Clean up
                        cJSON_Delete(result);
                        free(result_str);
                        _cJSON_Delete:
                            free_json_object(root);
                    }
                    break;
                                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    ESP_LOGI(RADAR_TO_ESP32_TAG, "hw fifo overflow");
                    // If fifo overflow happened, you should consider adding flow control for your application.
                    // The ISR has already reset the rx FIFO,
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(UART_PORT_NUM1);
                    xQueueReset(uart1_queue);
                    break;
                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    ESP_LOGI(RADAR_TO_ESP32_TAG, "ring buffer full");
                    // If buffer full happened, you should consider encreasing your buffer size
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(UART_PORT_NUM1);
                    xQueueReset(uart1_queue);
                    break;
                //Event of UART RX break detected
                case UART_BREAK:
                    ESP_LOGI(RADAR_TO_ESP32_TAG, "uart rx break");
                    break;
                //Event of UART parity check error
                case UART_PARITY_ERR:
                    ESP_LOGI(RADAR_TO_ESP32_TAG, "uart parity error");
                    break;
                //Event of UART frame error
                case UART_FRAME_ERR:
                    ESP_LOGI(RADAR_TO_ESP32_TAG, "uart frame error");
                    break;
                //UART_PATTERN_DET
                case UART_PATTERN_DET:
                    // uart_get_buffered_data_len(UART_PORT_NUM1, &buffered_size);
                    // int pos = uart_pattern_pop_pos(UART_PORT_NUM1);
                    // ESP_LOGI(RADAR_TO_ESP32_TAG, "[UART PATTERN DETECTED] pos: %d, buffered size: %d", pos, buffered_size);
                    // if (pos == -1) {
                    //     // There used to be a UART_PATTERN_DET event, but the pattern position queue is full so that it can not
                    //     // record the position. We should set a larger queue size.
                    //     // As an example, we directly flush the rx buffer here.
                    //     uart_flush_input(UART_PORT_NUM1);
                    // } else {
                    //     uart_read_bytes(UART_PORT_NUM1, dtmp, pos, 100 / portTICK_PERIOD_MS);
                    //     uint8_t pat[PATTERN_CHR_NUM + 1];
                    //     memset(pat, 0, sizeof(pat));
                    //     uart_read_bytes(UART_PORT_NUM1, pat, PATTERN_CHR_NUM, 100 / portTICK_PERIOD_MS);
                    //     ESP_LOGI(RADAR_TO_ESP32_TAG, "read data: %s", dtmp);
                    //     ESP_LOGI(RADAR_TO_ESP32_TAG, "read pat : %s", pat);
                    // }
                    // break;
                //Others
                default:
                    ESP_LOGI(RADAR_TO_ESP32_TAG, "uart event type: %d", event.type);
                    break;
            }
        }
    }
    free(data);
    data = NULL;
    vTaskDelete(NULL);
}

//ESP_MQTT_Communications
static void esp32_to_mqtt(char* data, esp_mqtt_client_handle_t client)
{
    const uint16_t length = strlen(data);
    ESP_LOGI(ESP32_TO_MQTT_TAG,"Update %d bytes",length);
    esp_mqtt_client_publish(client, RADAR_DATA_UPDATE_TOPIC, data, length, 1, 0);
}

//Initialize radar related functions
void radar_init(esp_mqtt_client_handle_t client)
{
    struct radar_to_esp32_task_param param0;
    param0.client_radar = client;
    uart_init();//initialize_uart
    open_radar_engineering_mode();
    xTaskCreate(radar_to_esp32, "radar_to_esp32_task", 2048*2, &param0, 10, &xUARTEventTaskHandle1);
    ESP_LOGI(RADAR_TO_ESP32_TAG, "Task_create");
    cJSON* result = create_json_object();
    add_data_to_json(result,"目标状态",3);                
    char* result_str = json_to_string(result);
    ESP_LOGI(RADAR_TO_ESP32_TAG,"%s\n", result_str);
    esp32_to_mqtt(result_str, client);
}