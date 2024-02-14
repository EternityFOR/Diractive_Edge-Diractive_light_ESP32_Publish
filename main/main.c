/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */


/****************************************************************************
* This is a demo for bluetooth config wifi connection to ap. You can config ESP32 to connect a softap
* or config ESP32 as a softap to be connected by other device. APP can be downloaded from github
* android source code: https://github.com/EspressifApp/EspBlufi
* iOS source code: https://github.com/EspressifApp/EspBlufiForiOS
****************************************************************************/

//***Head_Files***
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "driver/gpio.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "mqtt_init.h"
#include "ble_t.h"
#include "radar_data_handle.h"
#include "light_control.h"
#include "user_app.h"
#include "blufi_app.h"


//***Parameters***

//Tags
#define MAIN_TAG "System initialize..."
#define GPIO_NUM_Terminal (GPIO_NUM_10)
#define GPIO_NUM_Light (GPIO_NUM_4)


//***Functions***

//Functions

//Main function
void app_main(void)
{ 
    //Terminal Source
    gpio_config_t io_conf = {
    .mode = GPIO_MODE_OUTPUT,
    .pin_bit_mask = (1ULL << GPIO_NUM_Terminal),
    .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    esp_err_t err_source = gpio_set_level(GPIO_NUM_Terminal, 1); 
    if (err_source != ESP_OK) {
        ESP_LOGE(MAIN_TAG, "Failed to set pin voltage: %s", esp_err_to_name(err_source));
    } else {
        ESP_LOGI(MAIN_TAG, "Pin voltage set to HIGH");
    }

    //Light Source
    gpio_config_t io_conf_light = {
    .mode = GPIO_MODE_OUTPUT,
    .pin_bit_mask = (1ULL << GPIO_NUM_Light),
    .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf_light);
    esp_err_t err_light = gpio_set_level(GPIO_NUM_Light, 1);
    if (err_light != ESP_OK) {
        ESP_LOGE(MAIN_TAG, "Failed to set pin voltage: %s", esp_err_to_name(err_light));
    } else {
        ESP_LOGI(MAIN_TAG, "Pin voltage set to HIGH");
    }

    // Initialize NVS
    esp_err_t ret;
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
    
    //Initialize RGB
    init_led();
    set_default_color_from_nvs();

    // Initialize WIFI
    initialise_wifi();
    //Initialize Blutooth
    ble_init();

    //Multifunctional_Button
    user_app_key_init();

    //MQTT Connect
    mqtt_app_start();
    
    ESP_LOGI(MAIN_TAG, "[APP] Startup..");
    ESP_LOGI(MAIN_TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(MAIN_TAG, "[APP] IDF version: %s", esp_get_idf_version());
}
