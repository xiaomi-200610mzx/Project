#ifndef YMODEM_OTA_H
#define YMODEM_OTA_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void ymodem_ota_start(void);
void ymodem_ota_cancel(void);
bool ymodem_ota_is_active(void);
void ymodem_ota_process_bytes(const uint8_t *data, uint32_t len);
void ymodem_ota_task(void);

#ifdef __cplusplus
}
#endif

#endif /* YMODEM_OTA_H */
