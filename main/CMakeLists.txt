idf_component_register(
    # Sensor monitor firmware: MQ7, MQ8, SGP40, FSR402, and water level sensor.
    SRCS "hello_world_main.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES driver esp_driver_gpio esp_driver_i2c esp_adc
)
