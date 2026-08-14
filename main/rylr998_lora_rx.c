#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* ESP32-C3 TX -> RYLR998 RX, ESP32-C3 RX -> RYLR998 TX, 3.3V, GND */
#define LORA_UART_PORT          UART_NUM_1
#define LORA_UART_TX_GPIO       GPIO_NUM_4
#define LORA_UART_RX_GPIO       GPIO_NUM_5
#define LORA_UART_BAUD_RATE     115200
#define LORA_UART_BUF_SIZE      512

#define LORA_NODE_ADDRESS       2
#define LORA_NETWORK_ID         18
#define LORA_BAND_HZ            915000000
#define LORA_PARAMETER          "9,7,1,12"

#define COMMAND_TIMEOUT_MS      1500

static const char *TAG = "RYLR998_RX";

#define PACKET_SIZE 26

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

static int base64url_value(char c)
{
    if (c >= 'A' && c <= 'Z') return c - 'A';
    if (c >= 'a' && c <= 'z') return c - 'a' + 26;
    if (c >= '0' && c <= '9') return c - '0' + 52;
    if (c == '-') return 62;
    if (c == '_') return 63;
    return -1;
}

static int base64url_decode(const char *input, uint8_t *output, size_t output_size)
{
    size_t len = strlen(input), out = 0;
    uint32_t value = 0;
    int bits = 0;
    for (size_t i = 0; i < len; ++i) {
        int digit = base64url_value(input[i]);
        if (digit < 0) return -1;
        value = (value << 6) | (uint32_t)digit;
        bits += 6;
        if (bits >= 8) {
            bits -= 8;
            if (out >= output_size) return -1;
            output[out++] = (uint8_t)(value >> bits);
            value &= (1U << bits) - 1U;
        }
    }
    return (int)out;
}

static void lora_uart_init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = LORA_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(LORA_UART_PORT, LORA_UART_BUF_SIZE,
                                        LORA_UART_BUF_SIZE, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(LORA_UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(LORA_UART_PORT, LORA_UART_TX_GPIO,
                                 LORA_UART_RX_GPIO, UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE));
}

static int lora_read_line(char *line, size_t line_size, TickType_t timeout)
{
    size_t len = 0;
    TickType_t start = xTaskGetTickCount();

    while ((xTaskGetTickCount() - start) < timeout && len < line_size - 1) {
        uint8_t ch = 0;
        int read = uart_read_bytes(LORA_UART_PORT, &ch, 1, pdMS_TO_TICKS(50));
        if (read <= 0) {
            continue;
        }
        if (ch == '\r') {
            continue;
        }
        if (ch == '\n') {
            if (len == 0) {
                continue;
            }
            break;
        }
        line[len++] = (char)ch;
    }

    line[len] = '\0';
    return (int)len;
}

static bool lora_send_command(const char *command, const char *expected_reply)
{
    char rx_line[128];

    ESP_LOGI(TAG, "AT> %s", command);
    uart_write_bytes(LORA_UART_PORT, command, strlen(command));
    uart_write_bytes(LORA_UART_PORT, "\r\n", 2);

    TickType_t start = xTaskGetTickCount();
    while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(COMMAND_TIMEOUT_MS)) {
        int len = lora_read_line(rx_line, sizeof(rx_line), pdMS_TO_TICKS(200));
        if (len <= 0) {
            continue;
        }

        ESP_LOGI(TAG, "AT< %s", rx_line);
        if (strstr(rx_line, expected_reply) != NULL) {
            return true;
        }
        if (strstr(rx_line, "+ERR") != NULL) {
            return false;
        }
    }

    ESP_LOGW(TAG, "No expected reply for command: %s", command);
    return false;
}

static void lora_configure_module(void)
{
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

    ESP_LOGI(TAG, "RX ready: my_address=%d", LORA_NODE_ADDRESS);
}

static void handle_received_line(const char *line)
{
    int sender = 0;
    int payload_len = 0;
    int rssi = 0;
    int snr = 0;
    char payload[64] = {0};

    if (sscanf(line, "+RCV=%d,%d,%63[^,],%d,%d", &sender, &payload_len,
               payload, &rssi, &snr) != 5) {
        ESP_LOGI(TAG, "RX raw: %s", line);
        return;
    }

    uint8_t packet[PACKET_SIZE];
    if (payload_len != 35 || base64url_decode(payload, packet, sizeof(packet)) != PACKET_SIZE) {
        ESP_LOGW(TAG, "Invalid packet from address=%d len=%d", sender, payload_len);
        return;
    }
    uint16_t received_crc;
    memcpy(&received_crc, packet + 24, 2);
    if (packet_crc16(packet, 24) != received_crc) {
        ESP_LOGW(TAG, "CRC error from address=%d", sender);
        return;
    }
    uint16_t mq7, mq8, pressure, water, voc;
    float lat, lon;
    memcpy(&mq7, packet + 6, 2); memcpy(&mq8, packet + 8, 2);
    memcpy(&pressure, packet + 10, 2); memcpy(&water, packet + 12, 2);
    memcpy(&voc, packet + 14, 2); memcpy(&lat, packet + 16, 4); memcpy(&lon, packet + 20, 4);
    ESP_LOGI(TAG, "OK from=%d MAC=%02X:%02X:%02X:%02X:%02X:%02X MQ7=%u MQ8=%u pressure=%u water=%u VOC=%u lat=%.6f lon=%.6f RSSI=%d SNR=%d",
             sender, packet[0], packet[1], packet[2], packet[3], packet[4], packet[5],
             mq7, mq8, pressure, water, voc, lat, lon, rssi, snr);
}

void app_main(void)
{
    char rx_line[160];

    lora_uart_init();
    lora_configure_module();

    while (true) {
        int len = lora_read_line(rx_line, sizeof(rx_line), pdMS_TO_TICKS(1000));
        if (len > 0) {
            handle_received_line(rx_line);
        }
    }
}
