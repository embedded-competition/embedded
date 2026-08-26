| offset | size | 필드 | 타입 | 값 | 노드 산출식 |
|---|---|---|---|---|---|
| 0 | 6 | `mac` | uint8[6] | — | `esp_read_mac(ESP_MAC_WIFI_STA)` |
| 6 | 2 | `mq7` | uint16 | 0~1000 | `filtered * 1000 / 4095` |
| 8 | 2 | `mq8` | uint16 | 0~1000 | `filtered * 1000 / 4095` |
| 10 | 2 | `pressure` | uint16 | 0~1000 | `filtered * 1000 / 4095` (FSR402) |
| 12 | 2 | `water` | uint16 | 0~1000 | `filtered * 1000 / 4095` |
| 14 | 2 | `voc` | uint16 | 0~1000 | `(1 - filtered / 65535) * 1000` (SGP40, 반전) |
| 16 | 4 | `lat` | float32 | ±90 | 미장착 |
| 20 | 4 | `lon` | float32 | ±180 | 미장착 |
| 24 | 2 | `crc` | uint16 | — | CRC-16/CCITT-FALSE, 대상 0~23 |
| — | **26** | **합계** | | | base64url **35자** |
