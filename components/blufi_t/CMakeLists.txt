idf_component_register(SRCS "blufi_app.c"
                            "blufi_security.c"
                            "blufi_init.c" 
                            PRIV_REQUIRES "driver"
                            PRIV_REQUIRES "bt"
                            PRIV_REQUIRES "nvs_flash"
                            PRIV_REQUIRES "spi_flash"
                            PRIV_REQUIRES "mbedtls"
                            INCLUDE_DIRS "include")
target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")