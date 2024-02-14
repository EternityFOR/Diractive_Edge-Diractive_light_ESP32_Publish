idf_component_register(SRCS "led_strip_encoder.c"
							"light_control.c"
							PRIV_REQUIRES "driver"
							PRIV_REQUIRES "nvs_flash"
							INCLUDE_DIRS "include" 
							REQUIRES "")