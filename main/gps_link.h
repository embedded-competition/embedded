#pragma once

#include <stdbool.h>

void gps_link_init(void);

/* Drains and parses whatever GPS UART bytes have arrived. Non-blocking;
 * call once per loop iteration. */
void gps_link_update(void);

/* Returns true and fills *lat/*lon if the GPS currently has a fix. */
bool gps_link_get_fix(float *lat, float *lon);
