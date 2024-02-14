//***Header files***
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/rmt_tx.h"
#include "led_strip_encoder.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "nvs_flash.h"
//***Parameters***

//Tags
#define Light_TAG "Light control"

//Uart pin
#define BLINK_GPIO (GPIO_NUM_5)

//Number of lamp beads
#define LED_NUM 12

//Led strip resolution hz
#define RMT_LED_STRIP_RESOLUTION_HZ 12000000

//Led strip buffer
static uint8_t led_strip_pixels[LED_NUM * 3];

//rmt params
rmt_transmit_config_t tx_config = {
    .loop_count = 0, // no transfer loop
};
rmt_encoder_handle_t led_encoder = NULL;
rmt_channel_handle_t led_chan = NULL;

//Led strip set task and queue handle
static TaskHandle_t xUARTEventTaskHandle2 = NULL;
static QueueHandle_t led_strip_queue;

//Light control params
struct __Light_ctr_data{
  uint16_t red,green,blue;
}light_ctr_data; 


//***Functions***

//Functions
//Function declaration
void set_rgb(uint16_t Red, uint16_t Green, uint16_t Blue);

esp_err_t write_rgb_to_nvs(uint16_t Red, uint16_t Green, uint16_t Blue)
{
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGE(Light_TAG, "Error (%d) opening NVS handle!\n", err);
        return err;
    } else {
        ESP_LOGI(Light_TAG, "Committing RGB values to NVS...");
        err = nvs_set_u16(my_handle, "red", Red);
        err |= nvs_set_u16(my_handle, "green", Green);
        err |= nvs_set_u16(my_handle, "blue", Blue);
        if (err != ESP_OK) {
            ESP_LOGE(Light_TAG, "Error (%d) committing RGB values to NVS!\n", err);
            return err;
        } else {
            ESP_LOGI(Light_TAG, "RGB values committed to NVS successfully: red=%d, green=%d, blue=%d", Red, Green, Blue);
            err = nvs_commit(my_handle);
            if (err != ESP_OK) {
                ESP_LOGE(Light_TAG, "Error (%d) committing NVS changes!\n", err);
                return err;
            }
        }
        nvs_close(my_handle);
    }
    return ESP_OK;
}

esp_err_t get_rgb_from_nvs(uint16_t *red, uint16_t *green, uint16_t *blue)
{
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READONLY, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGE(Light_TAG, "Error (%d) opening NVS handle!\n", err);
        return err;
    } else {
        ESP_LOGI(Light_TAG, "Reading RGB values from NVS...");
        err = nvs_get_u16(my_handle, "red", red);
        err |= nvs_get_u16(my_handle, "green", green);
        err |= nvs_get_u16(my_handle, "blue", blue);
        if (err != ESP_OK) {
            ESP_LOGE(Light_TAG, "Error (%d) reading RGB values from NVS!\n", err);
            return err;
        } else {
            ESP_LOGI(Light_TAG, "RGB values read from NVS successfully: red=%d, green=%d, blue=%d", *red, *green, *blue);
        }
        nvs_close(my_handle);
    }
    return ESP_OK;
}

bool validate_rgb_with_nvs(uint16_t red, uint16_t green, uint16_t blue) {
    uint16_t nvsRed, nvsGreen, nvsBlue;
    esp_err_t nvsErr = get_rgb_from_nvs(&nvsRed, &nvsGreen, &nvsBlue);

    if (nvsErr != ESP_OK) {
        // 处理读取 NVS 错误的情况
        // ...
        return false;
    }

    return (red == nvsRed && green == nvsGreen && blue == nvsBlue);
}

void set_default_color_from_nvs() {
    uint16_t defaultRed, defaultGreen, defaultBlue;
    esp_err_t nvsReadErr = get_rgb_from_nvs(&defaultRed, &defaultGreen, &defaultBlue);

    if (nvsReadErr != ESP_OK) {
        ESP_LOGE(Light_TAG,"NVS read failed.");
        defaultRed = 255;
        defaultGreen = 255;
        defaultBlue = 255;
    }
    uint16_t tempRed, tempGreen, tempBlue;
    tempRed = (defaultRed + 1) % 256;
    tempGreen = (defaultGreen + 1) % 256;
    tempBlue = (defaultBlue + 1) % 256;
    esp_err_t nvsWriteErr = write_rgb_to_nvs(tempRed, tempGreen, tempBlue);
    if (nvsWriteErr != ESP_OK) {
        ESP_LOGE(Light_TAG,"NVS write failed.");
    }
    set_rgb(defaultRed,defaultGreen,defaultBlue);
}

static void led_strip_set_task() {
  static uint16_t oldRed = 0, oldGreen = 0, oldBlue = 0;
  static uint16_t red_t = 0, green_t = 0, blue_t = 0;
  struct __Light_ctr_data data;
  while(true){
    if(xQueueReceive(led_strip_queue,&data,portMAX_DELAY)){
      red_t = data.red;
      green_t = data.green;
      blue_t = data.blue;
      if (validate_rgb_with_nvs(red_t, green_t, blue_t)) {
        // If the RGB value is consistent with the values in NVS, no need to refresh the LED
        continue;
      }
      while (oldRed != red_t || oldGreen != green_t || oldBlue != blue_t) {
        if (oldRed < red_t) {
          oldRed++;
        } else if (oldRed > red_t) {
          oldRed--;
        }

        if (oldGreen < green_t) {
          oldGreen++;
        } else if (oldGreen > green_t) {
          oldGreen--;
        }

        if (oldBlue < blue_t) {
          oldBlue++;
        } else if (oldBlue > blue_t) {
          oldBlue--;
        }

        for (int i = 0; i < LED_NUM; i++) {
          led_strip_pixels[i * 3 + 0] = oldGreen;
          led_strip_pixels[i * 3 + 1] = oldRed;
          led_strip_pixels[i * 3 + 2] = oldBlue;
        }

        // Flush RGB values to LEDs
        ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
        // Delaying while-loop for making transition visible
        vTaskDelay(pdMS_TO_TICKS(3));
      }
      esp_err_t nvsWriteErr = write_rgb_to_nvs(red_t, green_t, blue_t);
      if (nvsWriteErr != ESP_OK) {
          ESP_LOGI(Light_TAG,"NVS write error");
      }
    }
  }
}

void set_rgb(uint16_t Red, uint16_t Green, uint16_t Blue) {
  light_ctr_data.red = Red;
  light_ctr_data.green = Green;
  light_ctr_data.blue = Blue;
  xQueueSend(led_strip_queue, (void *) &light_ctr_data, (TickType_t)portMAX_DELAY);
}

void init_led()
{
    //led strip set task and queue create
    led_strip_queue = xQueueCreate(50, sizeof( light_ctr_data ));
    xTaskCreate(led_strip_set_task, "led_strip_set_task", 2048*2, NULL, 10, &xUARTEventTaskHandle2);
    //Rmt initialize
    ESP_LOGI(Light_TAG, "Create RMT TX channel");
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT, // select source clock
        .gpio_num = BLINK_GPIO,
        .mem_block_symbols = 128, // increase the block size can make the LED less flickering
        .resolution_hz = RMT_LED_STRIP_RESOLUTION_HZ,
        .trans_queue_depth = 4, // set the number of transactions that can be pending in the background
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &led_chan));
    ESP_LOGI(Light_TAG, "Install led strip encoder");
    led_strip_encoder_config_t encoder_config = {
        .resolution = RMT_LED_STRIP_RESOLUTION_HZ,
    };
    ESP_ERROR_CHECK(rmt_new_led_strip_encoder(&encoder_config, &led_encoder));
    ESP_LOGI(Light_TAG, "Enable RMT TX channel");
    ESP_ERROR_CHECK(rmt_enable(led_chan));
    set_default_color_from_nvs();
}
