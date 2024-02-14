//***Header files***
#include <stdint.h>
#include "esp_err.h"

//***Functions***
void set_rgb(uint16_t Red, uint16_t Green, uint16_t Blue);
void init_led();
esp_err_t get_rgb_from_nvs(uint16_t *red, uint16_t *green, uint16_t *blue);
esp_err_t write_rgb_to_nvs(uint16_t Red, uint16_t Green, uint16_t Blue);

//***Led strip color configuration functions
void set_default_color_from_nvs();