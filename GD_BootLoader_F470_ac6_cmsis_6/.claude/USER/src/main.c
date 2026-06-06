#include "fun.h"
#include "bl_core.h"

int main(void)
{
    systick_config();
    bsp_usart_init();

    bootloader_run();

    while(1) {
        bsp_enter_deepsleep();
        bootloader_run();
    }
}
