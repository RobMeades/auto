idf_component_register(SRCS "auto_main.c"
                       PRIV_REQUIRES spi_flash  esp_timer esp_driver_gpio esp_driver_i2c
                       INCLUDE_DIRS "")
