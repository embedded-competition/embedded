#include "gps_link.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "board_pins.h"

#include "driver/uart.h"
#include "esp_err.h"

#define GPS_NMEA_LINE_MAX  96
#define GPS_NMEA_FIELD_MAX 15

typedef struct {
    float lat;
    float lon;
    bool has_fix;
} gps_fix_t;

static gps_fix_t s_gps_fix;

void gps_link_init(void)
{
    const uart_config_t config = {
        .baud_rate = GPS_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(GPS_UART_PORT, GPS_UART_BUF_SIZE,
                                        GPS_UART_BUF_SIZE, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(GPS_UART_PORT, &config));
    ESP_ERROR_CHECK(uart_set_pin(GPS_UART_PORT, GPS_UART_TX_GPIO,
                                 GPS_UART_RX_GPIO, UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE));
}

/* Converts an NMEA ddmm.mmmm (or dddmm.mmmm) field to signed decimal degrees. */
static float nmea_to_decimal_degrees(const char *ddmm_field, char hemisphere)
{
    float raw = strtof(ddmm_field, NULL);
    float degrees = floorf(raw / 100.0f);
    float minutes = raw - degrees * 100.0f;
    float decimal = degrees + minutes / 60.0f;
    return (hemisphere == 'S' || hemisphere == 'W') ? -decimal : decimal;
}

/* Splits an NMEA sentence on commas in place, preserving empty fields (unlike
 * strtok, which would collapse consecutive commas and misalign every field
 * after them -- exactly what happens on a "no fix" GGA sentence). */
static int nmea_split_fields(char *line, char **fields, int max_fields)
{
    int count = 0;
    char *cursor = line;
    fields[count++] = cursor;
    while (count < max_fields) {
        char *comma = strchr(cursor, ',');
        if (comma == NULL) {
            break;
        }
        *comma = '\0';
        cursor = comma + 1;
        fields[count++] = cursor;
    }
    return count;
}

/* Parses a GGA sentence ($--GGA,time,lat,N/S,lon,E/W,fix_quality,...) and
 * updates s_gps_fix. Ignores every other NMEA sentence type. `line` is
 * modified in place. */
static void gps_parse_line(char *line)
{
    if (line[0] != '$' || strlen(line) < 6 || strncmp(line + 3, "GGA", 3) != 0) {
        return;
    }

    char *fields[GPS_NMEA_FIELD_MAX] = {0};
    int field_count = nmea_split_fields(line, fields, GPS_NMEA_FIELD_MAX);

    /* fields: 0=talker+GGA 1=time 2=lat 3=N/S 4=lon 5=E/W 6=fix_quality */
    if (field_count < 7 || fields[2][0] == '\0' || fields[4][0] == '\0'
            || atoi(fields[6]) <= 0) {
        s_gps_fix.has_fix = false;
        return;
    }

    s_gps_fix.lat = nmea_to_decimal_degrees(fields[2], fields[3][0]);
    s_gps_fix.lon = nmea_to_decimal_degrees(fields[4], fields[5][0]);
    s_gps_fix.has_fix = true;
}

void gps_link_update(void)
{
    static char line[GPS_NMEA_LINE_MAX];
    static size_t line_length = 0;

    uint8_t chunk[64];
    int length;
    while ((length = uart_read_bytes(GPS_UART_PORT, chunk, sizeof(chunk), 0)) > 0) {
        for (int i = 0; i < length; ++i) {
            char ch = (char)chunk[i];
            if (ch == '\r') {
                continue;
            }
            if (ch == '\n') {
                if (line_length > 0) {
                    line[line_length] = '\0';
                    gps_parse_line(line);
                    line_length = 0;
                }
                continue;
            }
            if (line_length < GPS_NMEA_LINE_MAX - 1) {
                line[line_length++] = ch;
            } else {
                line_length = 0; /* Overflowed a line; drop and resync. */
            }
        }
    }
}

bool gps_link_get_fix(float *lat, float *lon)
{
    if (!s_gps_fix.has_fix) {
        return false;
    }
    *lat = s_gps_fix.lat;
    *lon = s_gps_fix.lon;
    return true;
}
