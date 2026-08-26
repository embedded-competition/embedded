#pragma once

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/uart.h"
#include "esp_adc/adc_oneshot.h"

/* ESP32-C3 SuperMini pin assignments. Change these to match your board. */
#define MQ7_ADC_CHANNEL              ADC_CHANNEL_0 /* GPIO0 */
#define MQ8_ADC_CHANNEL              ADC_CHANNEL_1 /* GPIO1 */
#define FSR402_ADC_CHANNEL           ADC_CHANNEL_2 /* GPIO2 */
#define WATER_LEVEL_ADC_CHANNEL      ADC_CHANNEL_3 /* GPIO3 */
#define ADC_RAW_MAX                  4095
#define ADC_SATURATION_MARGIN        5

#define I2C_SDA_GPIO                 GPIO_NUM_8
#define I2C_SCL_GPIO                 GPIO_NUM_9
#define I2C_PORT                     I2C_NUM_0
#define SGP40_I2C_ADDRESS            0x59

/* RYLR998 UART: ESP32-C3 TX -> module RX, ESP32-C3 RX -> module TX. */
#define LORA_UART_PORT               UART_NUM_1
#define LORA_UART_TX_GPIO            GPIO_NUM_4
#define LORA_UART_RX_GPIO            GPIO_NUM_5
#define LORA_UART_BAUD_RATE          115200
#define LORA_UART_BUF_SIZE           512
#define LORA_NODE_ADDRESS            1
#define LORA_RECEIVER_ADDRESS        2
#define LORA_NETWORK_ID              18
#define LORA_BAND_HZ                 922100000 /* 922.1 MHz */
#define LORA_PARAMETER                "9,7,1,12"

/* GPS UART: ESP32-C3 has only UART0/UART1; UART1 is taken by the LoRa
 * module, so GPS reuses UART0 here. This requires the console to run over
 * USB-Serial-JTAG instead of UART0 (see sdkconfig). ESP32-C3 TX -> GPS RX,
 * ESP32-C3 RX -> GPS TX. */
#define GPS_UART_PORT                UART_NUM_0
#define GPS_UART_TX_GPIO             GPIO_NUM_7
#define GPS_UART_RX_GPIO             GPIO_NUM_6
#define GPS_UART_BAUD_RATE           9600
#define GPS_UART_BUF_SIZE            512
