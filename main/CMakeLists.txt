idf_component_register(
    SRCS "main.c"
         "adc_sensors.c"
         "sensor_tracker.c"
         "mq_gas.c"
         "sgp40_sensor.c"
         "lora_link.c"
         "gps_link.c"
         "status_display.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES driver esp_driver_gpio esp_driver_i2c esp_driver_uart esp_adc
)
