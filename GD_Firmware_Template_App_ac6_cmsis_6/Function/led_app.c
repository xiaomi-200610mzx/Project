#include "fun.h"

uint8_t ucLed[6] = {1,0,1,0,1,1};

/**
 * @brief Display or close LEDs by state array.
 * @param ucLed LED state array.
 */
void led_disp(uint8_t *ucLed)
{
    uint8_t temp = 0x00;
    static uint8_t temp_old = 0xff;

    if (ucLed == NULL)
    {
        return;
    }
    /* Board LEDs are active-low, so the SET macros invert the visible level. */
    for (int i = 0; i < 6; i++)
    {
        if (ucLed[i])
        {
            temp |= (1 << i);
        }
    }

    if (temp_old != temp)
    {
        LED1_SET(temp & 0x01);
        LED2_SET(temp & 0x02);
        LED3_SET(temp & 0x04);
        LED4_SET(temp & 0x08);
        LED5_SET(temp & 0x10);
        LED6_SET(temp & 0x20);

        temp_old = temp;
    }
}

/**
 * @brief LED display task.
 */
void led_task(void)
{
    led_disp(ucLed);
}
