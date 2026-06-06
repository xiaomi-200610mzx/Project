#include "fun.h"
#include "ota_uart.h"
#include "ymodem_ota.h"

__IO uint16_t tx_count = 0;
__IO uint8_t rx_flag = 0;
__IO uint16_t uart_dma_len = 0;
uint8_t uart_dma_buffer[OTA_UART_RXBUF_SIZE] = {0};
__IO uint8_t debug_rx_flag = 0;
__IO uint16_t debug_uart_dma_len = 0;
uint8_t debug_uart_dma_buffer[DEBUG_UART_RXBUF_SIZE] = {0};
extern uint8_t rxbuffer[OTA_UART_RXBUF_SIZE];

/*
 * DMA writes OTA UART bytes into rxbuffer as a circular buffer. This cursor
 * tracks how far the application has already handed bytes to ota_uart.c.
 */
static uint32_t s_ota_dma_old_pos = 0U;

void debug_uart_frame_callback(const uint8_t *data, uint16_t len)
{
    (void)data;
    my_printf(DEBUG_USART, "debug rx len=%u\r\n", len);
}

int my_printf(uint32_t usart_periph, const char *format, ...)
{
    char buffer[512];
    va_list arg;
    int len;

    va_start(arg, format);
    len = vsnprintf(buffer, sizeof(buffer), format, arg);
    va_end(arg);

    for(tx_count = 0; tx_count < len; tx_count++) {
        usart_data_transmit(usart_periph, buffer[tx_count]);
        while(RESET == usart_flag_get(usart_periph, USART_FLAG_TBE));
    }

    return len;
}

void ota_reset_state(void)
{
    ota_uart_reset_state();
}

static void ota_uart_dma_poll(void)
{
    uint32_t pos;

    /*
     * In circular DMA mode, the current write position is buffer_size - NDTR.
     * When it wraps, feed the tail segment first and then the new head segment.
     */
    pos = OTA_UART_RXBUF_SIZE - dma_transfer_number_get(OTA_UART_DMA, OTA_UART_DMA_CH);
    if(pos == s_ota_dma_old_pos) {
        return;
    }

    if(pos > s_ota_dma_old_pos) {
        if(ymodem_ota_is_active()) {
            ymodem_ota_process_bytes(&rxbuffer[s_ota_dma_old_pos], pos - s_ota_dma_old_pos);
        } else {
            ota_uart_process_frame(&rxbuffer[s_ota_dma_old_pos], pos - s_ota_dma_old_pos);
        }
    } else {
        if(ymodem_ota_is_active()) {
            ymodem_ota_process_bytes(&rxbuffer[s_ota_dma_old_pos], OTA_UART_RXBUF_SIZE - s_ota_dma_old_pos);
        } else {
            ota_uart_process_frame(&rxbuffer[s_ota_dma_old_pos], OTA_UART_RXBUF_SIZE - s_ota_dma_old_pos);
        }
        if(pos > 0U) {
            if(ymodem_ota_is_active()) {
                ymodem_ota_process_bytes(rxbuffer, pos);
            } else {
                ota_uart_process_frame(rxbuffer, pos);
            }
        }
    }

    s_ota_dma_old_pos = pos;
}

void uart_task(void)
{
    if(debug_rx_flag) {
        debug_uart_frame_callback(debug_uart_dma_buffer, debug_uart_dma_len);
        memset(debug_uart_dma_buffer, 0, sizeof(debug_uart_dma_buffer));
        debug_uart_dma_len = 0;
        debug_rx_flag = 0;
    }

    /* Poll raw OTA bytes first, then let the OTA parser consume complete frames. */
    ota_uart_dma_poll();
    if(ymodem_ota_is_active()) {
        ymodem_ota_task();
    } else {
        ota_uart_task();
    }
}
