#include "fun.h"
#include "usart_app.h"
#include "ota_uart.h"
#include "ring_buffer.h"
#include "bl_partition.h"
#include "bl_param.h"
#include "ota_protocol.h"
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#define OTA_RING_BUFFER_SIZE        (16 * 1024)
#define OTA_FLOW_CTRL_PAUSE_LEVEL   (OTA_RING_BUFFER_SIZE * 80 / 100)
#define OTA_FLOW_CTRL_RESUME_LEVEL  (OTA_RING_BUFFER_SIZE * 20 / 100)

/*
 * Receive-side state machine. Bytes arrive from UART DMA into a software ring
 * buffer, then ota_uart_task() consumes full OTA frames from that ring.
 */
typedef enum
{
    OTA_STATE_WAIT_HEADER = 0,
    OTA_STATE_RECV_PAYLOAD
} ota_state_t;

typedef struct
{
    ota_state_t state;
    uint32_t app_version;
    uint32_t expected_size;
    uint32_t expected_crc32;
    uint32_t recv_size;
    uint32_t crc_acc;
    uint16_t expected_seq;
    bool flow_paused;
} ota_context_t;

/* Current OTA session metadata and write progress. */
static ota_context_t s_ota_ctx = {
    OTA_STATE_WAIT_HEADER,
    0U,
    0U,
    0U,
    0U,
    0U,
    0U,
    false
};

/* Full parameter page cache used because flash rewrite requires page erase. */
static uint8_t s_param_page_cache[BL_PARAM_PAGE_SIZE];

/* Software ring buffer decouples UART DMA bursts from flash write latency. */
static uint8_t s_ring_buffer_storage[OTA_RING_BUFFER_SIZE];
static ring_buffer_t s_ring_buffer;

/* Same reflected CRC32 algorithm used by BootLoader and sender metadata. */
static uint32_t crc32_update(uint32_t crc, const uint8_t *data, uint32_t len)
{
    uint32_t i;
    uint32_t j;

    for(i = 0U; i < len; i++) {
        crc ^= data[i];
        for(j = 0U; j < 8U; j++) {
            if((crc & 1U) != 0U) {
                crc = (crc >> 1U) ^ 0xEDB88320UL;
            } else {
                crc >>= 1U;
            }
        }
    }

    return crc;
}

static uint32_t crc32_finalize(uint32_t crc)
{
    return crc ^ 0xFFFFFFFFUL;
}

static uint32_t crc32_calc(const uint8_t *data, uint32_t len)
{
    return crc32_finalize(crc32_update(0xFFFFFFFFUL, data, len));
}

static void ota_uart_send_bytes(const uint8_t *data, uint32_t len)
{
    uint32_t i;

    for (i = 0U; i < len; i++) {
        usart_data_transmit(OTA_UART_PERIPH, data[i]);
        while (RESET == usart_flag_get(OTA_UART_PERIPH, USART_FLAG_TBE)) {
        }
    }
    while (RESET == usart_flag_get(OTA_UART_PERIPH, USART_FLAG_TC)) {
    }
}

static void ota_uart_send_text(const char *text)
{
    ota_uart_send_bytes((const uint8_t *)text, (uint32_t)strlen(text));
}

static void ota_send_reply(uint8_t type, uint8_t status, uint16_t seq, uint32_t offset)
{
    ota_frame_header_t reply;

    memset(&reply, 0, sizeof(reply));
    reply.magic = OTA_FRAME_MAGIC;
    reply.type = type;
    reply.status = status;
    reply.seq = seq;
    reply.offset = offset;
    ota_uart_send_bytes((const uint8_t *)&reply, (uint32_t)sizeof(reply));
}

static void ota_send_ack(uint16_t seq, uint32_t offset)
{
    ota_send_reply(OTA_FRAME_TYPE_ACK, 0U, seq, offset);
}

static void ota_send_nack(uint16_t seq, uint32_t offset, uint8_t reason)
{
    ota_send_reply(OTA_FRAME_TYPE_NACK, reason, seq, offset);
}

/* Parameter CRC excludes param_crc32, matching BootLoader validation. */
static uint32_t param_calc_crc(const bl_param_t *param)
{
    return crc32_calc((const uint8_t *)param, (uint32_t)offsetof(bl_param_t, param_crc32));
}

/* Flash helpers run from RAM so erase/program does not execute from busy flash. */
__attribute__((section(".ramfunc"), noinline))
static bool flash_wait_ready(void)
{
    uint32_t timeout = 0x3FFFFFUL;

    while((RESET != fmc_flag_get(FMC_FLAG_BUSY)) && (timeout > 0U)) {
        timeout--;
    }

    return (timeout > 0U);
}

__attribute__((section(".ramfunc"), noinline))
static void flash_clear_flags(void)
{
    fmc_flag_clear(FMC_FLAG_END);
    fmc_flag_clear(FMC_FLAG_WPERR);
    fmc_flag_clear(FMC_FLAG_PGSERR);
    fmc_flag_clear(FMC_FLAG_PGMERR);
}

__attribute__((section(".ramfunc"), noinline))
static bool flash_erase_pages(uint32_t start_addr, uint32_t size)
{
    uint32_t end_addr;
    uint32_t page_addr;

    if((size == 0U) || (start_addr < BL_FLASH_BASE_ADDR)) {
        return false;
    }

    end_addr = start_addr + size - 1U;
    if(end_addr > BL_FLASH_END_ADDR) {
        return false;
    }

    /* FMC erase granularity is one internal flash page. */
    page_addr = start_addr - (start_addr % BL_FLASH_PAGE_SIZE);
    end_addr = end_addr - (end_addr % BL_FLASH_PAGE_SIZE);

    fmc_unlock();
    flash_clear_flags();

    while(page_addr <= end_addr) {
        fmc_page_erase(page_addr);
        if(!flash_wait_ready()) {
            fmc_lock();
            return false;
        }
        flash_clear_flags();
        page_addr += BL_FLASH_PAGE_SIZE;
    }

    fmc_lock();
    return true;
}

__attribute__((section(".ramfunc"), noinline))
static bool flash_program_bytes(uint32_t addr, const uint8_t *data, uint32_t size)
{
    uint32_t i;
    uint32_t word_value;

    if((data == NULL) || (size == 0U)) {
        return false;
    }

    if((addr < BL_FLASH_BASE_ADDR) || ((addr + size - 1U) > BL_FLASH_END_ADDR)) {
        return false;
    }

    fmc_unlock();
    flash_clear_flags();

    /* Program aligned words first, then any remaining bytes. */
    while (((addr & 3UL) == 0UL) && (size >= 4U)) {
        memcpy(&word_value, data, sizeof(word_value));
        fmc_word_program(addr, word_value);
        if(!flash_wait_ready()) {
            fmc_lock();
            return false;
        }
        addr += 4UL;
        data += 4U;
        size -= 4U;
    }

    for(i = 0U; i < size; i++) {
        fmc_byte_program(addr + i, data[i]);
        if(!flash_wait_ready()) {
            fmc_lock();
            return false;
        }
    }

    flash_clear_flags();
    fmc_lock();
    return true;
}

static void param_set_default(bl_param_t *param)
{
    memset(param, 0, sizeof(bl_param_t));
    param->magic = BL_PARAM_MAGIC;
    param->version = BL_PARAM_VERSION;
    param->update_flag = BL_UPDATE_FLAG_IDLE;
    param->app1_addr = BL_APP1_START_ADDR;
    param->app2_addr = BL_APP2_START_ADDR;
    param->tail_magic = BL_PARAM_TAIL_MAGIC;
    param->param_crc32 = param_calc_crc(param);
}

static bool param_is_min_valid(const bl_param_t *param)
{
    return (param->magic == BL_PARAM_MAGIC) &&
           (param->tail_magic == BL_PARAM_TAIL_MAGIC) &&
           (param->version == BL_PARAM_VERSION);
}

static bool param_commit_update(uint32_t app_size, uint32_t app_crc32)
{
    bl_param_t main_param;
    bl_param_t backup_param;

    /*
     * Mark the freshly downloaded APP2 image as pending. BootLoader will
     * validate APP2, copy it to APP1, then clear or fail this flag on reset.
     */
    memcpy(s_param_page_cache, (const void *)BL_PARAM_PAGE_ADDR, BL_PARAM_PAGE_SIZE);
    memcpy(&main_param, (const void *)BL_PARAM_MAIN_ADDR, sizeof(bl_param_t));
    memcpy(&backup_param, (const void *)BL_PARAM_BACKUP_ADDR, sizeof(bl_param_t));

    if(!param_is_min_valid(&main_param)) {
        if(param_is_min_valid(&backup_param)) {
            main_param = backup_param;
        } else {
            param_set_default(&main_param);
        }
    }

    main_param.app_size = app_size;
    main_param.app_crc32 = app_crc32;
    main_param.app1_addr = BL_APP1_START_ADDR;
    main_param.app2_addr = BL_APP2_START_ADDR;
    main_param.update_flag = BL_UPDATE_FLAG_PENDING;
    main_param.last_error = BL_ERR_NONE;
    main_param.param_crc32 = param_calc_crc(&main_param);
    backup_param = main_param;

    memcpy(&s_param_page_cache[BL_PARAM_MAIN_ADDR - BL_PARAM_PAGE_ADDR], &main_param, sizeof(bl_param_t));
    memcpy(&s_param_page_cache[BL_PARAM_BACKUP_ADDR - BL_PARAM_PAGE_ADDR], &backup_param, sizeof(bl_param_t));

    if(!flash_erase_pages(BL_PARAM_PAGE_ADDR, BL_PARAM_PAGE_SIZE)) {
        return false;
    }

    return flash_program_bytes(BL_PARAM_PAGE_ADDR, s_param_page_cache, BL_PARAM_PAGE_SIZE);
}

void ota_uart_reset_state(void)
{
    /* Reset parser/session state, not the already downloaded flash contents. */
    memset(&s_ota_ctx, 0, sizeof(s_ota_ctx));
    s_ota_ctx.state = OTA_STATE_WAIT_HEADER;
    s_ota_ctx.crc_acc = 0xFFFFFFFFUL;
    s_ota_ctx.expected_seq = 0U;
    s_ota_ctx.flow_paused = false;
    ring_buffer_init(&s_ring_buffer, s_ring_buffer_storage, OTA_RING_BUFFER_SIZE);
}

static void ota_check_flow_control(void)
{
    uint32_t used = ring_buffer_available(&s_ring_buffer);

    /*
     * The sender script watches these text markers. They prevent the ring
     * buffer from filling while flash erase/program operations take time.
     */
    if (!s_ota_ctx.flow_paused && used >= OTA_FLOW_CTRL_PAUSE_LEVEL) {
        s_ota_ctx.flow_paused = true;
        ota_uart_send_text("PAUSE\r\n");
    } else if (s_ota_ctx.flow_paused && used <= OTA_FLOW_CTRL_RESUME_LEVEL) {
        s_ota_ctx.flow_paused = false;
        ota_uart_send_text("RESUME\r\n");
    }
}

static bool ota_begin(const ota_header_t *header)
{
    uint32_t erase_size;

    if((header->app_size == 0U) || (header->app_size > BL_APP2_SIZE) || (header->magic != OTA_FRAME_MAGIC)) {
        return false;
    }

    /* Erase APP2 staging pages before the first DATA frame is accepted. */
    erase_size = (header->app_size + BL_FLASH_PAGE_SIZE - 1U) & ~(BL_FLASH_PAGE_SIZE - 1U);
    if(!flash_erase_pages(BL_APP2_START_ADDR, erase_size)) {
        return false;
    }

    s_ota_ctx.state = OTA_STATE_RECV_PAYLOAD;
    s_ota_ctx.app_version = header->app_version;
    s_ota_ctx.expected_size = header->app_size;
    s_ota_ctx.expected_crc32 = header->app_crc32;
    s_ota_ctx.recv_size = 0U;
    s_ota_ctx.crc_acc = 0xFFFFFFFFUL;
    s_ota_ctx.expected_seq = 1U;

    my_printf(DEBUG_USART, "OTA: begin, ver=0x%08X, size=%u, crc=0x%08X\r\n",
              s_ota_ctx.app_version, s_ota_ctx.expected_size, s_ota_ctx.expected_crc32);
    return true;
}

static bool ota_write_payload(const uint8_t *payload, uint32_t len)
{
    uint32_t flash_addr;

    if((payload == NULL) || (len == 0U)) {
        return true;
    }

    if((s_ota_ctx.recv_size + len) > s_ota_ctx.expected_size) {
        my_printf(DEBUG_USART, "[OTA] Size overflow!\r\n");
        return false;
    }

    flash_addr = BL_APP2_START_ADDR + s_ota_ctx.recv_size;

    /* DATA frames are written directly into APP2 in offset order. */
    if(!flash_program_bytes(flash_addr, payload, len)) {
        my_printf(DEBUG_USART, "[OTA] Flash write failed!\r\n");
        return false;
    }

    s_ota_ctx.crc_acc = crc32_update(s_ota_ctx.crc_acc, payload, len);
    s_ota_ctx.recv_size += len;
    return true;
}

static bool ota_finalize_if_done(void)
{
    uint32_t crc_value;

    if(s_ota_ctx.recv_size != s_ota_ctx.expected_size) {
        return false;
    }

    crc_value = crc32_finalize(s_ota_ctx.crc_acc);
    /* Final CRC validates the whole APP2 payload before BootLoader is armed. */
    if(crc_value != s_ota_ctx.expected_crc32) {
        my_printf(DEBUG_USART, "OTA: crc mismatch, exp=0x%08X got=0x%08X\r\n",
                  s_ota_ctx.expected_crc32, crc_value);
        return false;
    }

    if(!param_commit_update(s_ota_ctx.expected_size, s_ota_ctx.expected_crc32)) {
        my_printf(DEBUG_USART, "OTA: param commit failed\r\n");
        return false;
    }

    return true;
}

static bool ota_parse_start_payload(const uint8_t *payload, uint16_t len, ota_header_t *header)
{
    ota_image_header_v2_t header_v2;
    uint32_t crc_expect;

    if ((payload == NULL) || (header == NULL)) {
        return false;
    }

    if (len == sizeof(ota_header_t)) {
        memcpy(header, payload, sizeof(ota_header_t));
        return true;
    }

    /* V2 header protects target address/type so a wrong image is rejected early. */
    if (len != sizeof(ota_image_header_v2_t)) {
        return false;
    }

    memcpy(&header_v2, payload, sizeof(header_v2));
    if ((header_v2.magic != OTA_FRAME_MAGIC) ||
        (header_v2.header_version != OTA_IMAGE_HEADER_V2_VERSION) ||
        (header_v2.header_size != sizeof(ota_image_header_v2_t)) ||
        (header_v2.target_addr != BL_APP1_START_ADDR) ||
        (header_v2.image_type != OTA_IMAGE_TYPE_APP1) ||
        (header_v2.app_size == 0U) ||
        (header_v2.app_size > BL_APP2_SIZE)) {
        return false;
    }

    crc_expect = crc32_calc(payload, (uint32_t)offsetof(ota_image_header_v2_t, header_crc32));
    if (crc_expect != header_v2.header_crc32) {
        return false;
    }

    header->magic = header_v2.magic;
    header->app_version = header_v2.app_version;
    header->app_size = header_v2.app_size;
    header->app_crc32 = header_v2.app_crc32;
    return true;
}

void ota_uart_process_frame(const uint8_t *data, uint32_t len)
{
    if((data == NULL) || (len == 0U)) {
        return;
    }

    /* Called by the UART/DMA polling layer with newly received raw bytes. */
    if (!ring_buffer_write(&s_ring_buffer, data, len)) {
        my_printf(DEBUG_USART, "[OTA] Ring buffer overflow!\r\n");
        return;
    }

    ota_check_flow_control();
}

void ota_uart_task(void)
{
    ota_frame_header_t frame;
    ota_header_t ota_header;
    uint8_t payload[OTA_FRAME_MAX_PAYLOAD];
    uint32_t available;

    while (1) {
        /* Wait until at least a complete header is available in the ring. */
        available = ring_buffer_available(&s_ring_buffer);
        if (available < sizeof(ota_frame_header_t)) {
            return;
        }

        (void)ring_buffer_peek(&s_ring_buffer, (uint8_t *)&frame, sizeof(frame));
        if (frame.magic != OTA_FRAME_MAGIC) {
            /* Resynchronize by dropping bytes until the next frame magic. */
            ring_buffer_drop(&s_ring_buffer, 1U);
            continue;
        }

        if (frame.length > OTA_FRAME_MAX_PAYLOAD) {
            ring_buffer_drop(&s_ring_buffer, 1U);
            ota_send_nack(frame.seq, s_ota_ctx.recv_size, OTA_NACK_BAD_LENGTH);
            continue;
        }

        if (available < ((uint32_t)sizeof(ota_frame_header_t) + frame.length)) {
            /* Header is present but payload is still arriving. */
            return;
        }

        ring_buffer_drop(&s_ring_buffer, (uint32_t)sizeof(ota_frame_header_t));
        if (frame.length > 0U) {
            (void)ring_buffer_read(&s_ring_buffer, payload, frame.length);
        }

        if ((frame.length > 0U) && (crc32_calc(payload, frame.length) != frame.crc32)) {
            ota_send_nack(frame.seq, s_ota_ctx.recv_size, OTA_NACK_BAD_CRC);
            continue;
        }

        if (frame.type == OTA_FRAME_TYPE_START) {
            /* START opens a new transfer and erases APP2 staging space. */
            if ((frame.seq != 0U) || (frame.offset != 0U) ||
                !ota_parse_start_payload(payload, frame.length, &ota_header)) {
                ota_send_nack(frame.seq, s_ota_ctx.recv_size, OTA_NACK_BAD_LENGTH);
                continue;
            }
            if ((s_ota_ctx.state == OTA_STATE_RECV_PAYLOAD) &&
                (ota_header.magic == OTA_FRAME_MAGIC) &&
                (ota_header.app_version == s_ota_ctx.app_version) &&
                (ota_header.app_size == s_ota_ctx.expected_size) &&
                (ota_header.app_crc32 == s_ota_ctx.expected_crc32)) {
                /* Duplicate START after retry: acknowledge current progress. */
                ota_send_ack(frame.seq, s_ota_ctx.recv_size);
                continue;
            }
            if (!ota_begin(&ota_header)) {
                ota_send_nack(frame.seq, 0U, OTA_NACK_FLASH);
                ota_uart_reset_state();
                return;
            }
            ota_send_ack(frame.seq, s_ota_ctx.recv_size);
        } else if (frame.type == OTA_FRAME_TYPE_DATA) {
            /* DATA must be strictly sequential to keep flash writes simple. */
            if (s_ota_ctx.state != OTA_STATE_RECV_PAYLOAD) {
                ota_send_nack(frame.seq, s_ota_ctx.recv_size, OTA_NACK_BAD_STATE);
                continue;
            }
            if ((uint16_t)(s_ota_ctx.expected_seq - 1U) == frame.seq &&
                (frame.offset + frame.length) <= s_ota_ctx.recv_size) {
                /* Sender retried the previous frame after a lost ACK. */
                ota_send_ack(frame.seq, s_ota_ctx.recv_size);
                continue;
            }
            if (frame.seq != s_ota_ctx.expected_seq) {
                ota_send_nack(frame.seq, s_ota_ctx.recv_size, OTA_NACK_BAD_SEQ);
                continue;
            }
            if (frame.offset != s_ota_ctx.recv_size) {
                ota_send_nack(frame.seq, s_ota_ctx.recv_size, OTA_NACK_BAD_OFFSET);
                continue;
            }
            if (!ota_write_payload(payload, frame.length)) {
                my_printf(DEBUG_USART, "OTA: write failed at %u\r\n", s_ota_ctx.recv_size);
                ota_send_nack(frame.seq, s_ota_ctx.recv_size, OTA_NACK_FLASH);
                ota_uart_reset_state();
                return;
            }
            s_ota_ctx.expected_seq++;
            ota_send_ack(frame.seq, s_ota_ctx.recv_size);

            if ((s_ota_ctx.recv_size % 4096U) == 0U || s_ota_ctx.recv_size == s_ota_ctx.expected_size) {
                my_printf(DEBUG_USART, "[OTA] Progress: %u/%u (%u%%)\r\n",
                          s_ota_ctx.recv_size, s_ota_ctx.expected_size,
                          s_ota_ctx.recv_size * 100U / s_ota_ctx.expected_size);
            }
        } else if (frame.type == OTA_FRAME_TYPE_END) {
            /* END arms BootLoader only after all bytes and CRC match. */
            if ((s_ota_ctx.state != OTA_STATE_RECV_PAYLOAD) || (frame.seq != s_ota_ctx.expected_seq) ||
                (frame.offset != s_ota_ctx.recv_size) || (frame.length != 0U) ||
                (frame.crc32 != s_ota_ctx.expected_crc32)) {
                ota_send_nack(frame.seq, s_ota_ctx.recv_size, OTA_NACK_FINAL);
                continue;
            }
            if (!ota_finalize_if_done()) {
                ota_send_nack(frame.seq, s_ota_ctx.recv_size, OTA_NACK_FINAL);
                ota_uart_reset_state();
                return;
            }
            ota_send_ack(frame.seq, s_ota_ctx.recv_size);
            my_printf(DEBUG_USART, "OTA: done, reset to bootloader\r\n");
            __disable_irq();
            NVIC_SystemReset();
        } else {
            ota_send_nack(frame.seq, s_ota_ctx.recv_size, OTA_NACK_BAD_STATE);
        }

        ota_check_flow_control();
    }
}
