#ifndef __KEY_APP_H
#define __KEY_APP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

extern uint8_t key_val, key_old, key_down, key_up;

uint8_t key_read(void);
void key_task(void);

#ifdef __cplusplus
}
#endif

#endif /* __KEY_APP_H */
