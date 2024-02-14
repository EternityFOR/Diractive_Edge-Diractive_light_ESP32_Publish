idf_component_register(SRCS "sign_mqtt.c"
							"sign_sha256.c"
                            "mqtt_init.c"
                            PRIV_REQUIRES "mqtt"
                            PRIV_REQUIRES "json"
                            PRIV_REQUIRES "ld2410"
                            PRIV_REQUIRES "light_control"
                            PRIV_REQUIRES "main"
                            INCLUDE_DIRS "include")
target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")