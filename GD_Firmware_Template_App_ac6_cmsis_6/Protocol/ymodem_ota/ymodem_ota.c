#include "ymodem_ota.h"
#include "fun.h"
#include "usart_app.h"
#include "bl_partition.h"
#include "bl_param.h"
#include <stdbool.h>
#include <stddef.h>

#define YMODEM_SOH                 0x01U
#define YMODEM_STX                 0x02U
#define YMODEM_EOT                 0x04U
#define YMODEM_ACK                 0x06U
#define YMODEM_NAK                 0x15U
#define YMODEM_CAN                 0x18U
#define YMODEM_CRC_REQ             0x43U
#define YMODEM_SUB                 0x1AU

#define YMODEM_PACKET_128          128U
#define YMODEM_PACKET_1K           1024U
#define YMODEM_PACKET_OVERHEAD     5U
#define YMODEM_C_INTERVAL_MS       1000U
#define YMODEM_FINAL_RESET_MS      3000U

typedef enum {
    YMODEM_STATE_IDLE = 0,
    YMODEM_STATE_WAIT_START,
    YMODEM_STATE_RECV_PACKET,
    YMODEM_STATE_WAIT_FINAL_EMPTY
} ymodem_state_t;

typedef struct {
    ymodem_state_t state;
    uint8_t packet[3U + YMODEM_PACKET_1K + 2U];
    uint32_t packet_index;
    uint32_t packet_total;
    uint32_t packet_data_size;
    uint32_t app_size;
    uint32_t recv_size;
    uint32_t crc_acc;
    uint32_t last_c_tick;
    uint32_t done_tick;
    uint8_t expected_blk;
    bool eot_seen;
    bool reset_pending;
} ymodem_ctx_t;

static ymodem_ctx_t s_ymodem;
static uint8_t s_param_page_cache[BL_PARAM_PAGE_SIZE];

static void ymodem_send_byte(uint8_t value)
{
    usart_data_transmit(OTA_UART_PERIPH, value);
    while(RESET == usart_flag_get(OTA_UART_PERIPH, USART_FLAG_TBE)) {
    }
}

static void ymodem_send_ctrl(uint8_t value)
{
    ymodem_send_byte(value);
    while(RESET == usart_flag_get(OTA_UART_PERIPH, USART_FLAG_TC)) {
    }
}

static uint16_t crc16_xmodem(const uint8_t *data, uint32_t len)
{
    uint16_t crc = 0U;
    uint32_t i;
    uint8_t bit;

    for(i = 0U; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for(bit = 0U; bit < 8U; bit++) {
            if((crc & 0x8000U) != 0U) {
                crc = (uint16_t)((crc << 1) ^ 0x1021U);
            } else {
                crc <<= 1;
            }
        }
    }

    return crc;
}

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

static uint32_t param_calc_crc(const bl_param_t *param)
{
    return crc32_calc((const uint8_t *)param, (uint32_t)offsetof(bl_param_t, param_crc32));
}

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

    while(((addr & 3UL) == 0UL) && (size >= 4U)) {
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

static uint32_t parse_decimal_size(const uint8_t *text)
{
    uint32_t value = 0U;

    while((*text >= (uint8_t)'0') && (*text <= (uint8_t)'9')) {
        value = (value * 10U) + (uint32_t)(*text - (uint8_t)'0');
        text++;
    }

    return value;
}

static uint32_t ymodem_block0_size(const uint8_t *payload, uint32_t len)
{
    uint32_t i = 0U;

    while((i < len) && (payload[i] != 0U)) {
        i++;
    }

    if(i >= len) {
        return 0U;
    }

    i++;
    return parse_decimal_size(&payload[i]);
}

static void ymodem_reset_session(bool keep_active)
{
    memset(&s_ymodem, 0, sizeof(s_ymodem));
    s_ymodem.state = keep_active ? YMODEM_STATE_WAIT_START : YMODEM_STATE_IDLE;
    s_ymodem.expected_blk = 1U;
    s_ymodem.crc_acc = 0xFFFFFFFFUL;
    s_ymodem.last_c_tick = get_system_ms();
}

void ymodem_ota_start(void)
{
    if(ymodem_ota_is_active()) {
        return;
    }

    ymodem_reset_session(true);
    my_printf(DEBUG_USART, "YMODEM: wait file on OTA UART\r\n");
    ymodem_send_ctrl(YMODEM_CRC_REQ);
}

void ymodem_ota_cancel(void)
{
    ymodem_reset_session(false);
}

bool ymodem_ota_is_active(void)
{
    return s_ymodem.state != YMODEM_STATE_IDLE;
}

static void ymodem_abort(const char *reason)
{
    ymodem_send_ctrl(YMODEM_CAN);
    ymodem_send_ctrl(YMODEM_CAN);
    my_printf(DEBUG_USART, "YMODEM: abort %s\r\n", reason);
    ymodem_reset_session(false);
}

static bool ymodem_begin_file(const uint8_t *payload, uint32_t len)
{
    uint32_t erase_size;

    if((payload[0] == 0U) || (len == 0U)) {
        ymodem_send_ctrl(YMODEM_ACK);
        ymodem_reset_session(false);
        return false;
    }

    s_ymodem.app_size = ymodem_block0_size(payload, len);
    if((s_ymodem.app_size == 0U) || (s_ymodem.app_size > BL_APP2_SIZE)) {
        ymodem_abort("bad size");
        return false;
    }

    s_ymodem.recv_size = 0U;
    s_ymodem.expected_blk = 1U;
    s_ymodem.crc_acc = 0xFFFFFFFFUL;

    my_printf(DEBUG_USART, "YMODEM: begin size=%u\r\n", s_ymodem.app_size);
    ymodem_send_ctrl(YMODEM_ACK);

    erase_size = (s_ymodem.app_size + BL_FLASH_PAGE_SIZE - 1U) & ~(BL_FLASH_PAGE_SIZE - 1U);
    if(!flash_erase_pages(BL_APP2_START_ADDR, erase_size)) {
        ymodem_abort("erase");
        return false;
    }

    ymodem_send_ctrl(YMODEM_CRC_REQ);
    return true;
}

static bool ymodem_write_data(const uint8_t *payload, uint32_t len)
{
    uint32_t write_len;

    if(s_ymodem.recv_size >= s_ymodem.app_size) {
        return true;
    }

    write_len = s_ymodem.app_size - s_ymodem.recv_size;
    if(write_len > len) {
        write_len = len;
    }

    if(!flash_program_bytes(BL_APP2_START_ADDR + s_ymodem.recv_size, payload, write_len)) {
        ymodem_abort("write");
        return false;
    }

    s_ymodem.crc_acc = crc32_update(s_ymodem.crc_acc, payload, write_len);
    s_ymodem.recv_size += write_len;

    return true;
}

static void ymodem_finish_file(void)
{
    uint32_t app_crc32;

    if(s_ymodem.recv_size != s_ymodem.app_size) {
        ymodem_abort("incomplete");
        return;
    }

    app_crc32 = crc32_finalize(s_ymodem.crc_acc);
    if(!param_commit_update(s_ymodem.app_size, app_crc32)) {
        ymodem_abort("param");
        return;
    }

    my_printf(DEBUG_USART, "YMODEM: done size=%u crc=0x%08X\r\n", s_ymodem.app_size, app_crc32);
    s_ymodem.reset_pending = true;
    s_ymodem.done_tick = get_system_ms();
}

static void ymodem_handle_eot(void)
{
    if(!s_ymodem.eot_seen) {
        s_ymodem.eot_seen = true;
        ymodem_send_ctrl(YMODEM_NAK);
        return;
    }

    ymodem_send_ctrl(YMODEM_ACK);
    ymodem_send_ctrl(YMODEM_CRC_REQ);
    s_ymodem.state = YMODEM_STATE_WAIT_FINAL_EMPTY;
    ymodem_finish_file();
}

static void ymodem_handle_packet(void)
{
    uint8_t blk = s_ymodem.packet[1];
    uint8_t blk_inv = s_ymodem.packet[2];
    const uint8_t *payload = &s_ymodem.packet[3];
    uint16_t crc_rx;
    uint16_t crc_calc;

    if(((uint8_t)(blk + blk_inv)) != 0xFFU) {
        ymodem_send_ctrl(YMODEM_NAK);
        return;
    }

    crc_rx = ((uint16_t)s_ymodem.packet[3U + s_ymodem.packet_data_size] << 8) |
             (uint16_t)s_ymodem.packet[4U + s_ymodem.packet_data_size];
    crc_calc = crc16_xmodem(payload, s_ymodem.packet_data_size);
    if(crc_rx != crc_calc) {
        ymodem_send_ctrl(YMODEM_NAK);
        return;
    }

    if(blk == 0U) {
        if(s_ymodem.state == YMODEM_STATE_WAIT_FINAL_EMPTY) {
            ymodem_send_ctrl(YMODEM_ACK);
            return;
        }
        (void)ymodem_begin_file(payload, s_ymodem.packet_data_size);
        return;
    }

    if(blk == s_ymodem.expected_blk) {
        if(ymodem_write_data(payload, s_ymodem.packet_data_size)) {
            s_ymodem.expected_blk++;
            ymodem_send_ctrl(YMODEM_ACK);
            if((s_ymodem.recv_size % 4096U) == 0U || s_ymodem.recv_size == s_ymodem.app_size) {
                my_printf(DEBUG_USART, "YMODEM: %u/%u\r\n", s_ymodem.recv_size, s_ymodem.app_size);
            }
        }
    } else if(blk == (uint8_t)(s_ymodem.expected_blk - 1U)) {
        ymodem_send_ctrl(YMODEM_ACK);
    } else {
        ymodem_send_ctrl(YMODEM_NAK);
    }
}

static void ymodem_feed_byte(uint8_t byte)
{
    if(s_ymodem.state == YMODEM_STATE_WAIT_START || s_ymodem.state == YMODEM_STATE_WAIT_FINAL_EMPTY) {
        if(byte == YMODEM_SOH || byte == YMODEM_STX) {
            s_ymodem.packet_index = 0U;
            s_ymodem.packet_data_size = (byte == YMODEM_SOH) ? YMODEM_PACKET_128 : YMODEM_PACKET_1K;
            s_ymodem.packet_total = 3U + s_ymodem.packet_data_size + 2U;
            s_ymodem.packet[s_ymodem.packet_index++] = byte;
            s_ymodem.state = YMODEM_STATE_RECV_PACKET;
        } else if(byte == YMODEM_EOT) {
            ymodem_handle_eot();
        } else if(byte == YMODEM_CAN) {
            ymodem_abort("cancel");
        }
        return;
    }

    if(s_ymodem.state == YMODEM_STATE_RECV_PACKET) {
        s_ymodem.packet[s_ymodem.packet_index++] = byte;
        if(s_ymodem.packet_index >= s_ymodem.packet_total) {
            s_ymodem.state = s_ymodem.reset_pending ? YMODEM_STATE_WAIT_FINAL_EMPTY : YMODEM_STATE_WAIT_START;
            ymodem_handle_packet();
        }
    }
}

void ymodem_ota_process_bytes(const uint8_t *data, uint32_t len)
{
    uint32_t i;

    if((data == NULL) || !ymodem_ota_is_active()) {
        return;
    }

    for(i = 0U; i < len; i++) {
        ymodem_feed_byte(data[i]);
        if(!ymodem_ota_is_active()) {
            break;
        }
    }
}

void ymodem_ota_task(void)
{
    uint32_t now;

    if(!ymodem_ota_is_active()) {
        return;
    }

    now = get_system_ms();
    if(s_ymodem.reset_pending) {
        if((now - s_ymodem.done_tick) >= YMODEM_FINAL_RESET_MS) {
            NVIC_SystemReset();
        }
        return;
    }

    if((s_ymodem.state == YMODEM_STATE_WAIT_START) &&
       ((now - s_ymodem.last_c_tick) >= YMODEM_C_INTERVAL_MS)) {
        s_ymodem.last_c_tick = now;
        ymodem_send_ctrl(YMODEM_CRC_REQ);
    }
}
