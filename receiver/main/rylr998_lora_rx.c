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
        /* RYLR998 can deliver one +RCV frame in multiple UART bursts. */
        int read = uart_read_bytes(LORA_UART_PORT, &ch, 1, pdMS_TO_TICKS(250));
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
    int payload_offset = 0;
    char body[128] = {0};
    char *rssi_separator = NULL;
    char *snr_separator = NULL;

    /* RYLR998 format: +RCV=address,length,payload,rssi,snr.
     * The payload itself is CSV, so split the final two fields from the end. */
    if (sscanf(line, "+RCV=%d,%d,%n", &sender, &payload_len, &payload_offset) != 2
            || payload_offset <= 0) {
        ESP_LOGI(TAG, "RX raw: %s", line);
        return;
    }

    strncpy(body, line + payload_offset, sizeof(body) - 1);
    snr_separator = strrchr(body, ',');
    if (snr_separator == NULL) {
        ESP_LOGI(TAG, "RX raw: %s", line);
        return;
    }
    *snr_separator = '\0';
    if (sscanf(snr_separator + 1, "%d", &snr) != 1) {
        ESP_LOGI(TAG, "RX raw: %s", line);
        return;
    }

    rssi_separator = strrchr(body, ',');
    if (rssi_separator == NULL) {
        ESP_LOGI(TAG, "RX raw: %s", line);
        return;
    }
    *rssi_separator = '\0';
    if (sscanf(rssi_separator + 1, "%d", &rssi) != 1) {
        ESP_LOGI(TAG, "RX raw: %s", line);
        return;
    }

    ESP_LOGI(TAG, "Received sensor data: %s", body);
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
