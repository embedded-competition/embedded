#include "lora_link.h"

#include <stdio.h>
#include <string.h>

#include "board_pins.h"

#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "LORA_LINK";

#define PACKET_SIZE 26

static void lora_uart_init(void)
{
    const uart_config_t config = {
        .baud_rate = LORA_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(LORA_UART_PORT, LORA_UART_BUF_SIZE,
                                        LORA_UART_BUF_SIZE, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(LORA_UART_PORT, &config));
    ESP_ERROR_CHECK(uart_set_pin(LORA_UART_PORT, LORA_UART_TX_GPIO,
                                 LORA_UART_RX_GPIO, UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE));
}

static int lora_read_line(char *line, size_t line_size, TickType_t timeout)
{
    size_t length = 0;
    TickType_t start = xTaskGetTickCount();

    while ((xTaskGetTickCount() - start) < timeout && length < line_size - 1) {
        uint8_t ch = 0;
        if (uart_read_bytes(LORA_UART_PORT, &ch, 1, pdMS_TO_TICKS(50)) <= 0) {
            continue;
        }
        if (ch == '\r') {
            continue;
        }
        if (ch == '\n') {
            if (length == 0) {
                continue;
            }
            break;
        }
        line[length++] = (char)ch;
    }

    line[length] = '\0';
    return (int)length;
}

static bool lora_send_command(const char *command, const char *expected_reply)
{
    char response[128];
    uart_write_bytes(LORA_UART_PORT, command, strlen(command));
    uart_write_bytes(LORA_UART_PORT, "\r\n", 2);

    TickType_t start = xTaskGetTickCount();
    while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(1500)) {
        int length = lora_read_line(response, sizeof(response), pdMS_TO_TICKS(200));
        if (length <= 0) {
            continue;
        }
        if (strstr(response, expected_reply) != NULL) {
            return true;
        }
        if (strstr(response, "+ERR") != NULL) {
            return false;
        }
    }

    ESP_LOGW(TAG, "LoRa command timeout: %s", command);
    return false;
}

static uint16_t packet_crc16(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static size_t base64url_encode(const uint8_t *input, size_t input_len, char *output)
{
    static const char alphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-_";
    size_t out = 0;
    for (size_t i = 0; i < input_len; i += 3) {
        uint32_t value = (uint32_t)input[i] << 16;
        if (i + 1 < input_len) value |= (uint32_t)input[i + 1] << 8;
        if (i + 2 < input_len) value |= input[i + 2];
        output[out++] = alphabet[(value >> 18) & 0x3F];
        output[out++] = alphabet[(value >> 12) & 0x3F];
        if (i + 1 < input_len) output[out++] = alphabet[(value >> 6) & 0x3F];
        if (i + 2 < input_len) output[out++] = alphabet[value & 0x3F];
    }
    output[out] = '\0';
    return out;
}

void lora_link_init(void)
{
    lora_uart_init();

    char command[64];
    uart_flush_input(LORA_UART_PORT);
    lora_send_command("AT", "+OK");
    snprintf(command, sizeof(command), "AT+ADDRESS=%d", LORA_NODE_ADDRESS);
    lora_send_command(command, "+OK");
    snprintf(command, sizeof(command), "AT+NETWORKID=%d", LORA_NETWORK_ID);
    lora_send_command(command, "+OK");
    snprintf(command, sizeof(command), "AT+BAND=%d", LORA_BAND_HZ);
    lora_send_command(command, "+OK");
    snprintf(command, sizeof(command), "AT+PARAMETER=%s", LORA_PARAMETER);
    lora_send_command(command, "+OK");
}

static bool lora_send_sensor_data(const uint8_t *packet, size_t packet_len)
{
    char payload[40];
    char command[100];
    int length = (int)base64url_encode(packet, packet_len, payload);
    snprintf(command, sizeof(command), "AT+SEND=%d,%d,%s",
             LORA_RECEIVER_ADDRESS, length, payload);
    return lora_send_command(command, "+OK");
}

bool lora_link_send_packet(const uint8_t mac[6], uint16_t mq7, uint16_t mq8,
                           uint16_t pressure, uint16_t water, uint16_t voc,
                           float lat, float lon)
{
    uint8_t packet[PACKET_SIZE] = {0};
    memcpy(packet, mac, 6);
    memcpy(packet + 6, &mq7, 2);
    memcpy(packet + 8, &mq8, 2);
    memcpy(packet + 10, &pressure, 2);
    memcpy(packet + 12, &water, 2);
    memcpy(packet + 14, &voc, 2);
    memcpy(packet + 16, &lat, 4);
    memcpy(packet + 20, &lon, 4);
    uint16_t crc = packet_crc16(packet, 24);
    memcpy(packet + 24, &crc, 2);
    return lora_send_sensor_data(packet, sizeof(packet));
}
