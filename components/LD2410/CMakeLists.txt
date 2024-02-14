idf_component_register(SRCS "radar_data_handle.c"
							PRIV_REQUIRES "driver"
							PRIV_REQUIRES "json"
							PRIV_REQUIRES "mqtt"
							PRIV_REQUIRES "main"
							INCLUDE_DIRS "include")