idf_component_register(SRCS "main.c" "a_util.c" "a_sensor_hall_effect_tmag5273.c"
                       PRIV_REQUIRES spi_flash  esp_timer esp_driver_gpio esp_driver_i2c
                       INCLUDE_DIRS ".")
