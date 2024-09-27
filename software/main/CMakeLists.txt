idf_component_register(SRCS "main.c" "a_util.c" "a_sensor_hall_effect_tmag5273.c"
                       PRIV_REQUIRES spi_flash  esp_timer driver
                       INCLUDE_DIRS ".")
