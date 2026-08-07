idf_component_register(
    # Sensor + LoRa sender firmware. For the receiver board, replace this with:
    # SRCS "rylr998_lora_rx.c"
    SRCS "hello_world_main.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES driver esp_driver_gpio esp_driver_i2c esp_adc
)
