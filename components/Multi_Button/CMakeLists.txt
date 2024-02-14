idf_component_register(SRCS "user_app.c"
							"user_key.c"
							PRIV_REQUIRES "driver"
							PRIV_REQUIRES "esp_timer"
							PRIV_REQUIRES "light_control"
							PRIV_REQUIRES "blufi_t"
							INCLUDE_DIRS "include")