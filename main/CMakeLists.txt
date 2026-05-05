idf_component_register(SRCS "main.c"
                       PRIV_REQUIRES spi_flash esp_driver_mcpwm esp_timer
                       INCLUDE_DIRS ".")