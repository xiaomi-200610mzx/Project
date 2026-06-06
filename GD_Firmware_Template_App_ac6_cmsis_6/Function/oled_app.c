
#include "fun.h"

extern uint16_t adc_value[2];

/**
 * @brief Print formatted text on the OLED using 6x8 ASCII font.
 * @param x Character position on the X axis, range 0-127.
 * @param y Character page on the Y axis, range 0-3.
 * @return Formatted string length returned by vsnprintf().
 **/
int oled_printf(uint8_t x, uint8_t y, const char *format, ...)
{
  char buffer[512];
  va_list arg;
  int len;

  va_start(arg, format);
  len = vsnprintf(buffer, sizeof(buffer), format, arg);
  va_end(arg);

  OLED_ShowStr(x, y, buffer, 8);
  return len;
}

void oled_task(void)
{
    oled_printf(0, 0, "KEY STA: %d%d%d%d%d%d", KEY1_LEVEL, KEY2_LEVEL, KEY3_LEVEL, KEY4_LEVEL, KEY5_LEVEL, KEY6_LEVEL);
    oled_printf(0, 1, "uwTick:%lld", (long long)get_system_ms());
    oled_printf(0, 2, "ADC: ");
}

/* CUSTOM EDIT */
