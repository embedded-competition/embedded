#pragma once

#include <stdbool.h>
#include <stdint.h>

void lora_link_init(void);

/* Builds the 26-byte binary sensor packet (see lora-frame.md) and sends it
 * base64url-encoded over LoRa. Pass lat/lon as 0.0f when GPS has no fix. */
bool lora_link_send_packet(const uint8_t mac[6], uint16_t mq7, uint16_t mq8,
                           uint16_t pressure, uint16_t water, uint16_t voc,
                           float lat, float lon);
