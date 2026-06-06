#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include "gd32f4xx.h"
#include "bl_core.h"
#include "bl_config.h"
#include "bl_param.h"
#include "fun.h"
#include "usart_app.h"
#include "systick.h"

typedef void (*app_entry_t)(void);

/* Scratch copy of the full parameter page before erase/rewrite. */
static uint8_t bl_page_cache[BL_PARAM_PAGE_SIZE];

/* Same reflected CRC32 algorithm used by APP OTA metadata. */
static uint32_t bl_crc32_calc(const uint8_t *data, uint32_t len)
{
    uint32_t crc;
    uint32_t i;
    uint32_t j;

    crc = 0xFFFFFFFFUL;
    for(i = 0UL; i < len; i++) {
        crc ^= data[i];
        for(j = 0UL; j < 8UL; j++) {
            if((crc & 1UL) != 0UL) {
                crc = (crc >> 1UL) ^ 0xEDB88320UL;
            } else {
                crc >>= 1UL;
            }
        }
    }

    return crc ^ 0xFFFFFFFFUL;
}

static uint32_t bl_param_calc_crc(const bl_param_t *param)
{
    return bl_crc32_calc((const uint8_t *)param, (uint32_t)offsetof(bl_param_t, param_crc32));
}

static bool bl_wait_fmc_ready(void)
{
    uint32_t timeout = 0x3FFFFFUL;

    while((RESET != fmc_flag_get(FMC_FLAG_BUSY)) && (timeout > 0UL)) {
        timeout--;
    }

    return (timeout > 0UL);
}

static void bl_flash_clear_flags(void)
{
    fmc_flag_clear(FMC_FLAG_END);
    fmc_flag_clear(FMC_FLAG_WPERR);
    fmc_flag_clear(FMC_FLAG_PGSERR);
    fmc_flag_clear(FMC_FLAG_PGMERR);
}

static bool bl_flash_erase_pages(uint32_t start_addr, uint32_t size)
{
    uint32_t page_addr;
    uint32_t end_addr;

    if((size == 0UL) || (start_addr < BL_FLASH_BASE_ADDR)) {
        return false;
    }

    end_addr = start_addr + size - 1UL;
    if(end_addr > BL_FLASH_END_ADDR) {
        return false;
    }

    page_addr = start_addr - (start_addr % BL_FLASH_PAGE_SIZE);
    end_addr = end_addr - (end_addr % BL_FLASH_PAGE_SIZE);

    fmc_unlock();
    bl_flash_clear_flags();

    while(page_addr <= end_addr) {
        fmc_page_erase(page_addr);
        if(!bl_wait_fmc_ready()) {
            fmc_lock();
            return false;
        }
        bl_flash_clear_flags();
        page_addr += BL_FLASH_PAGE_SIZE;
    }

    fmc_lock();
    return true;
}

static bool bl_flash_program_bytes(uint32_t addr, const uint8_t *data, uint32_t size)
{
    uint32_t i;
    uint32_t word_value;

    if((data == NULL) || (size == 0UL)) {
        return false;
    }

    if((addr < BL_FLASH_BASE_ADDR) || ((addr + size - 1UL) > BL_FLASH_END_ADDR)) {
        return false;
    }

    fmc_unlock();
    bl_flash_clear_flags();

    while(((addr & 3UL) == 0UL) && (size >= 4UL)) {
        memcpy(&word_value, data, sizeof(word_value));
        fmc_word_program(addr, word_value);
        if(!bl_wait_fmc_ready()) {
            fmc_lock();
            return false;
        }
        addr += 4UL;
        data += 4UL;
        size -= 4UL;
    }

    for(i = 0UL; i < size; i++) {
        fmc_byte_program(addr + i, data[i]);
        if(!bl_wait_fmc_ready()) {
            fmc_lock();
            return false;
        }
    }

    bl_flash_clear_flags();
    fmc_lock();
    return true;
}

static bool bl_is_app_vector_valid(uint32_t app_base)
{
    uint32_t msp;
    uint32_t reset_handler;

    /*
     * A valid Cortex-M image starts with an SRAM MSP and a reset handler in
     * flash. This catches empty flash (0xFFFFFFFF) before jumping.
     */
    msp = *(volatile uint32_t *)app_base;
    reset_handler = *(volatile uint32_t *)(app_base + 4UL);

    if((msp & 0x2FFE0000UL) != 0x20000000UL) {
        return false;
    }

    if((reset_handler < BL_FLASH_BASE_ADDR) || (reset_handler > BL_FLASH_END_ADDR)) {
        return false;
    }

    return true;
}

static bool bl_is_param_valid(const bl_param_t *param)
{
    uint32_t crc_expect;

    /*
     * Validate both fixed markers and semantic fields before trusting the
     * pending update request. The CRC covers fields before param_crc32.
     */
    if(param->magic != BL_PARAM_MAGIC) {
        return false;
    }
    if(param->tail_magic != BL_PARAM_TAIL_MAGIC) {
        return false;
    }
    if(param->version != BL_PARAM_VERSION) {
        return false;
    }
    if((param->app1_addr != BL_APP1_START_ADDR) || (param->app2_addr != BL_APP2_START_ADDR)) {
        return false;
    }
    if((param->app_size > BL_APP1_SIZE) || (param->app_size > BL_APP2_SIZE)) {
        return false;
    }

    crc_expect = bl_param_calc_crc(param);
    if(crc_expect != param->param_crc32) {
        return false;
    }

    return true;
}

static void bl_param_set_default(bl_param_t *param)
{
    memset(param, 0, sizeof(bl_param_t));
    param->magic = BL_PARAM_MAGIC;
    param->version = BL_PARAM_VERSION;
    param->update_flag = BL_UPDATE_FLAG_IDLE;
    param->app1_addr = BL_APP1_START_ADDR;
    param->app2_addr = BL_APP2_START_ADDR;
    param->log_write_index = 0UL;
    param->tail_magic = BL_PARAM_TAIL_MAGIC;
    param->param_crc32 = bl_param_calc_crc(param);
}

static bool bl_commit_param_page(const bl_param_t *param, const bl_log_entry_t *log_entry, bool append_log)
{
    uint32_t log_index;
    uint32_t log_offset;
    const uint8_t *page_ptr;
    bl_param_t main_copy;
    bl_param_t backup_copy;

    /*
     * Flash can only be erased as a page. Preserve log entries and unrelated
     * bytes in RAM, patch both parameter copies, then rewrite the full page.
     */
    memcpy(bl_page_cache, (const void *)BL_PARAM_PAGE_ADDR, BL_PARAM_PAGE_SIZE);

    memcpy(&main_copy, param, sizeof(bl_param_t));
    main_copy.param_crc32 = bl_param_calc_crc(&main_copy);
    memcpy(&backup_copy, &main_copy, sizeof(bl_param_t));

    memcpy(&bl_page_cache[BL_PARAM_MAIN_ADDR - BL_PARAM_PAGE_ADDR], &main_copy, sizeof(bl_param_t));
    memcpy(&bl_page_cache[BL_PARAM_BACKUP_ADDR - BL_PARAM_PAGE_ADDR], &backup_copy, sizeof(bl_param_t));

    if(append_log && (log_entry != NULL)) {
        log_index = main_copy.log_write_index % BL_LOG_ENTRY_COUNT;
        log_offset = (BL_LOG_ADDR - BL_PARAM_PAGE_ADDR) + (log_index * BL_LOG_ENTRY_SIZE);
        memcpy(&bl_page_cache[log_offset], log_entry, sizeof(bl_log_entry_t));
    }

    if(!bl_flash_erase_pages(BL_PARAM_PAGE_ADDR, BL_PARAM_PAGE_SIZE)) {
        return false;
    }

    page_ptr = (const uint8_t *)bl_page_cache;
    return bl_flash_program_bytes(BL_PARAM_PAGE_ADDR, page_ptr, BL_PARAM_PAGE_SIZE);
}

bool bl_commit_param(bl_param_t *param)
{
    bl_param_t repaired;

    /*
     * Commit a normalized parameter block. If the caller passed an incomplete
     * block, keep update metadata but rebuild all fixed fields.
     */
    if(!bl_is_param_valid(param)) {
        bl_param_set_default(&repaired);
        repaired.update_flag = param->update_flag;
        repaired.app_size = param->app_size;
        repaired.app_crc32 = param->app_crc32;
        repaired.last_error = param->last_error;
        *param = repaired;
    }

    param->magic = BL_PARAM_MAGIC;
    param->version = BL_PARAM_VERSION;
    param->app1_addr = BL_APP1_START_ADDR;
    param->app2_addr = BL_APP2_START_ADDR;
    param->tail_magic = BL_PARAM_TAIL_MAGIC;

    return bl_commit_param_page(param, NULL, false);
}

static void bl_log_prepare(bl_log_entry_t *entry, uint32_t seq, uint32_t event_id, uint32_t result,
                           uint32_t value0, uint32_t value1, uint32_t value2)
{
    memset(entry, 0, sizeof(bl_log_entry_t));
    entry->magic = BL_LOG_MAGIC;
    entry->seq = seq;
    entry->event_id = event_id;
    entry->result = result;
    entry->value0 = value0;
    entry->value1 = value1;
    entry->value2 = value2;
    entry->crc32 = bl_crc32_calc((const uint8_t *)entry, (uint32_t)offsetof(bl_log_entry_t, crc32));
}

void bl_log_dump_uart(void)
{
    uint32_t i;
    const bl_log_entry_t *entry;
    uint32_t crc_expect;
    uint32_t valid_count = 0UL;

    my_printf(DEBUG_USART, "BL log dump:\r\n");
    for(i = 0UL; i < BL_LOG_ENTRY_COUNT; i++) {
        entry = (const bl_log_entry_t *)(BL_LOG_ADDR + (i * BL_LOG_ENTRY_SIZE));
        if(entry->magic != BL_LOG_MAGIC) {
            continue;
        }

        crc_expect = bl_crc32_calc((const uint8_t *)entry, (uint32_t)offsetof(bl_log_entry_t, crc32));
        my_printf(DEBUG_USART,
                  "  [%02u] seq=%u event=%u result=%u v0=0x%08X v1=0x%08X v2=0x%08X crc=%s\r\n",
                  i,
                  entry->seq,
                  entry->event_id,
                  entry->result,
                  entry->value0,
                  entry->value1,
                  entry->value2,
                  (crc_expect == entry->crc32) ? "OK" : "BAD");
        valid_count++;
    }

    if(valid_count == 0UL) {
        my_printf(DEBUG_USART, "  <empty>\r\n");
    }
}

static bool bl_copy_app2_to_app1(uint32_t app_size)
{
    uint8_t buffer[BL_COPY_CHUNK_SIZE];
    uint32_t copied = 0UL;
    uint32_t erase_size;
    uint32_t left;
    uint32_t chunk_size;
    const uint8_t *src;

    if((app_size == 0UL) || (app_size > BL_APP1_SIZE) || (app_size > BL_APP2_SIZE)) {
        return false;
    }

    /*
     * Only erase the pages needed by the new APP image. Remaining APP1 pages
     * are left untouched, but boot uses app_size for CRC validation.
     */
    erase_size = (app_size + BL_FLASH_PAGE_SIZE - 1UL) & ~(BL_FLASH_PAGE_SIZE - 1UL);
    if(!bl_flash_erase_pages(BL_APP1_START_ADDR, erase_size)) {
        return false;
    }

    while(copied < app_size) {
        left = app_size - copied;
        chunk_size = (left > BL_COPY_CHUNK_SIZE) ? BL_COPY_CHUNK_SIZE : left;
        src = (const uint8_t *)(BL_APP2_START_ADDR + copied);

        memcpy(buffer, src, chunk_size);
        if(!bl_flash_program_bytes(BL_APP1_START_ADDR + copied, buffer, chunk_size)) {
            return false;
        }
        copied += chunk_size;
    }

    return true;
}

static uint32_t bl_crc32_flash(uint32_t start_addr, uint32_t size)
{
    return bl_crc32_calc((const uint8_t *)start_addr, size);
}

static void bl_jump_to_app(uint32_t app_base)
{
    app_entry_t app_entry;
    uint32_t app_reset_handler;
    uint32_t i;

    /*
     * Leave BootLoader cleanly: stop SysTick, clear all pending/enabled NVIC
     * interrupts, relocate vector table, load APP MSP, then call reset handler.
     */
    __disable_irq();

    SysTick->CTRL = 0UL;
    SysTick->LOAD = 0UL;
    SysTick->VAL = 0UL;

    for(i = 0UL; i < 8UL; i++) {
        NVIC->ICER[i] = 0xFFFFFFFFUL;
        NVIC->ICPR[i] = 0xFFFFFFFFUL;
    }

    __DSB();
    __ISB();

    SCB->VTOR = app_base;
    __set_MSP(*(volatile uint32_t *)app_base);

    app_reset_handler = *(volatile uint32_t *)(app_base + 4UL);
    app_entry = (app_entry_t)app_reset_handler;

    __enable_irq();
    app_entry();
}

void bootloader_run(void)
{
    bl_param_t main_param;
    bl_param_t backup_param;
    bl_param_t working_param;
    bl_log_entry_t log_entry;
    bool main_valid;
    bool backup_valid;
    bool need_repair = false;
    bool update_ok;
    uint32_t app_crc;

    /*
     * Read two persistent copies. The copy with the larger update_counter wins
     * when both are valid; otherwise the valid side repairs the broken side.
     */
    memcpy(&main_param, (const void *)BL_PARAM_MAIN_ADDR, sizeof(bl_param_t));
    memcpy(&backup_param, (const void *)BL_PARAM_BACKUP_ADDR, sizeof(bl_param_t));

    main_valid = bl_is_param_valid(&main_param);
    backup_valid = bl_is_param_valid(&backup_param);

    if(main_valid && backup_valid) {
        if(main_param.update_counter >= backup_param.update_counter) {
            working_param = main_param;
        } else {
            working_param = backup_param;
            need_repair = true;
        }
    } else if(main_valid) {
        working_param = main_param;
        need_repair = true;
    } else if(backup_valid) {
        working_param = backup_param;
        need_repair = true;
    } else {
        bl_param_set_default(&working_param);
        need_repair = true;
        working_param.last_error = BL_ERR_PARAM_INVALID;
    }

    if(need_repair) {
        /* Record that BootLoader repaired the parameter page before continuing. */
        bl_log_prepare(&log_entry, working_param.update_counter + working_param.fail_counter,
                       BL_LOG_EVENT_PARAM_RECOVER, 1UL, main_valid ? 1UL : 0UL, backup_valid ? 1UL : 0UL,
                       working_param.last_error);
        working_param.log_write_index = (working_param.log_write_index + 1UL) % BL_LOG_ENTRY_COUNT;
        (void)bl_commit_param_page(&working_param, &log_entry, true);
    }

    if(working_param.update_flag == BL_UPDATE_FLAG_PENDING) {
        /*
         * Update path:
         *   1. Validate APP2 vector and payload CRC.
         *   2. Copy APP2 into APP1.
         *   3. Validate APP1 CRC.
         *   4. Commit result, then reset so the normal boot path jumps to APP1.
         */
        if((working_param.app_size == 0UL) || (working_param.app_size > BL_APP2_SIZE) ||
           !bl_is_app_vector_valid(BL_APP2_START_ADDR)) {
            working_param.update_flag = BL_UPDATE_FLAG_FAILED;
            working_param.fail_counter++;
            working_param.last_error = BL_ERR_APP2_INVALID;

            bl_log_prepare(&log_entry, working_param.update_counter + working_param.fail_counter,
                           BL_LOG_EVENT_UPDATE_FAIL, 0UL, BL_ERR_APP2_INVALID, working_param.app_size, 0UL);
            working_param.log_write_index = (working_param.log_write_index + 1UL) % BL_LOG_ENTRY_COUNT;
            (void)bl_commit_param_page(&working_param, &log_entry, true);
            NVIC_SystemReset();
        }

        app_crc = bl_crc32_flash(BL_APP2_START_ADDR, working_param.app_size);
        if(app_crc != working_param.app_crc32) {
            working_param.update_flag = BL_UPDATE_FLAG_FAILED;
            working_param.fail_counter++;
            working_param.last_error = BL_ERR_APP2_INVALID;

            bl_log_prepare(&log_entry, working_param.update_counter + working_param.fail_counter,
                           BL_LOG_EVENT_UPDATE_FAIL, 0UL, BL_ERR_APP2_INVALID,
                           working_param.app_crc32, app_crc);
            working_param.log_write_index = (working_param.log_write_index + 1UL) % BL_LOG_ENTRY_COUNT;
            (void)bl_commit_param_page(&working_param, &log_entry, true);
            NVIC_SystemReset();
        }

        update_ok = bl_copy_app2_to_app1(working_param.app_size);
        if(update_ok) {
            app_crc = bl_crc32_flash(BL_APP1_START_ADDR, working_param.app_size);
            if(app_crc == working_param.app_crc32) {
                working_param.update_flag = BL_UPDATE_FLAG_IDLE;
                working_param.update_counter++;
                working_param.last_error = BL_ERR_NONE;

                bl_log_prepare(&log_entry, working_param.update_counter + working_param.fail_counter,
                               BL_LOG_EVENT_UPDATE_OK, 1UL, working_param.app_size, working_param.app_crc32, app_crc);
            } else {
                working_param.update_flag = BL_UPDATE_FLAG_FAILED;
                working_param.fail_counter++;
                working_param.last_error = BL_ERR_COPY_FAILED;

                bl_log_prepare(&log_entry, working_param.update_counter + working_param.fail_counter,
                               BL_LOG_EVENT_UPDATE_FAIL, 0UL, BL_ERR_COPY_FAILED,
                               working_param.app_crc32, app_crc);
            }
        } else {
            working_param.update_flag = BL_UPDATE_FLAG_FAILED;
            working_param.fail_counter++;
            working_param.last_error = BL_ERR_COPY_FAILED;

            bl_log_prepare(&log_entry, working_param.update_counter + working_param.fail_counter,
                           BL_LOG_EVENT_UPDATE_FAIL, 0UL, BL_ERR_COPY_FAILED, 0UL, 0UL);
        }

        working_param.log_write_index = (working_param.log_write_index + 1UL) % BL_LOG_ENTRY_COUNT;
        (void)bl_commit_param_page(&working_param, &log_entry, true);
        NVIC_SystemReset();
    }

    if(bl_is_app_vector_valid(BL_APP1_START_ADDR)) {
        /* Normal boot path: no pending update, APP1 vector table looks valid. */
        my_printf(DEBUG_USART, "BL: jumping to app...\r\n");
        bl_jump_to_app(BL_APP1_START_ADDR);
    }

    bl_log_dump_uart();
    /* No runnable APP image. Keep BootLoader alive for debug instead of jumping. */
    working_param.last_error = BL_ERR_APP1_INVALID;
    bl_log_prepare(&log_entry, working_param.update_counter + working_param.fail_counter,
                   BL_LOG_EVENT_JUMP_FAIL, 0UL, BL_ERR_APP1_INVALID, BL_APP1_START_ADDR, 0UL);
    working_param.log_write_index = (working_param.log_write_index + 1UL) % BL_LOG_ENTRY_COUNT;
    (void)bl_commit_param_page(&working_param, &log_entry, true);

    while(1) {
    }
}
