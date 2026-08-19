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
#define LORA_UART_BAUD_RATE     9600
#define LORA_UART_BUF_SIZE      512
#define MAX_PAYLOAD_LENGTH      128

#define LORA_NODE_ADDRESS       1
#define LORA_RECEIVER_ADDRESS   2
#define LORA_NETWORK_ID         18
#define LORA_BAND_HZ            922100000 /* 922.1 MHz */
#define LORA_PARAMETER          "9,7,1,12"

#define COMMAND_TIMEOUT_MS      1500

static const char *TAG = "RYLR998_TX";

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

    ESP_LOGI(TAG, "TX ready: my_address=%d receiver_address=%d",
             LORA_NODE_ADDRESS, LORA_RECEIVER_ADDRESS);
}

static bool lora_send_payload(const char *payload)
{
    char command[MAX_PAYLOAD_LENGTH + 48];
    int payload_len = (int)strlen(payload);

    if (payload_len == 0 || payload_len > MAX_PAYLOAD_LENGTH) {
        ESP_LOGW(TAG, "Payload length must be 1-%d bytes", MAX_PAYLOAD_LENGTH);
        return false;
    }

    snprintf(command, sizeof(command), "AT+SEND=%d,%d,%s",
             LORA_RECEIVER_ADDRESS, payload_len, payload);
    return lora_send_command(command, "+OK");
}

void app_main(void)
{
    char payload[MAX_PAYLOAD_LENGTH + 1];

    lora_uart_init();
    lora_configure_module();

    ESP_LOGI(TAG, "Type a line in the USB serial monitor and press Enter to send.");
    ESP_LOGI(TAG, "Destination=%d, max payload=%d bytes",
             LORA_RECEIVER_ADDRESS, MAX_PAYLOAD_LENGTH);

    while (true) {
        if (fgets(payload, sizeof(payload), stdin) == NULL) {
            /* USB Serial/JTAG can report a temporary EOF before monitor input
             * is available. Keep the firmware alive and try again. */
            clearerr(stdin);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        payload[strcspn(payload, "\r\n")] = '\0';

        if (payload[0] == '\0') {
            continue;
        }

        if (lora_send_payload(payload)) {
            ESP_LOGI(TAG, "Sent data: %s", payload);
        } else {
            ESP_LOGW(TAG, "Send failed: %s", payload);
        }
    }
}
