#include "fun.h"
#include "key_app.h"

extern uint8_t ucLed[6];

uint8_t key_val, key_old, key_down, key_up;

uint8_t key_read(void)
{
    uint8_t temp = 0;
    if(KEY1_LEVEL == RESET) temp = 1;
    if(KEY2_LEVEL == RESET) temp = 2;
    if(KEY3_LEVEL == RESET) temp = 3;
    if(KEY4_LEVEL == RESET) temp = 4;
    if(KEY5_LEVEL == RESET) temp = 5;
    if(KEY6_LEVEL == RESET) temp = 6;
    return temp;
}

void key_task(void)
{
    key_val = key_read();
    key_down = key_val & (key_old ^ key_val);
    key_up = ~key_val & (key_old ^ key_val);
    key_old = key_val;

    if(key_down == 1) ucLed[0] ^= 1;
    if(key_down == 2) ucLed[1] ^= 1;
    if(key_down == 3) ucLed[2] ^= 1;
    if(key_down == 4) ucLed[3] ^= 1;
    if(key_down == 5) ucLed[4] ^= 1;
    if(key_down == 6) ucLed[5] ^= 1;
}
