idf_component_register(
    SRCS "src/main.c"
         "src/adc_sensors.c"
         "src/sensor_tracker.c"
         "src/mq_gas.c"
         "src/sgp40_sensor.c"
         "src/lora_link.c"
         "src/gps_link.c"
         "src/status_display.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES driver esp_driver_gpio esp_driver_i2c esp_driver_uart esp_adc
)
