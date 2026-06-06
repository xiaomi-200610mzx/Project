#ifndef COMMON_BL_PARTITION_H
#define COMMON_BL_PARTITION_H

#include <stdint.h>

/*
 * Internal flash layout used by both BootLoader and APP.
 *
 * APP1 is the executable image region. APP2 is the staging region used by
 * OTA download code. On reset, BootLoader validates APP2 and copies it to
 * APP1 only when the parameter page marks an update as pending.
 */
#define BL_FLASH_BASE_ADDR          0x08000000UL
#define BL_FLASH_TOTAL_SIZE         0x00080000UL
#define BL_FLASH_END_ADDR           (BL_FLASH_BASE_ADDR + BL_FLASH_TOTAL_SIZE - 1UL)
#define BL_FLASH_PAGE_SIZE          0x00001000UL

/* BootLoader image. Keep the linker scatter file inside this range. */
#define BL_BOOT_START_ADDR          0x08000000UL
#define BL_BOOT_SIZE                0x0000C000UL
#define BL_BOOT_END_ADDR            (BL_BOOT_START_ADDR + BL_BOOT_SIZE - 1UL)

/* One flash page: main parameter copy, backup copy, and compact log entries. */
#define BL_PARAM_START_ADDR         0x0800C000UL
#define BL_PARAM_SIZE               0x00001000UL

/* Active application slot. BootLoader jumps here after validation. */
#define BL_APP1_START_ADDR          0x0800D000UL
#define BL_APP1_SIZE                0x00038000UL
#define BL_APP1_END_ADDR            (BL_APP1_START_ADDR + BL_APP1_SIZE - 1UL)

/* OTA staging slot. APP writes new firmware here, BootLoader copies it to APP1. */
#define BL_APP2_START_ADDR          0x08045000UL
#define BL_APP2_SIZE                0x00038000UL
#define BL_APP2_END_ADDR            (BL_APP2_START_ADDR + BL_APP2_SIZE - 1UL)

/* Reserved for user data outside BootLoader update/copy operations. */
#define BL_DATA_START_ADDR          0x0807D000UL
#define BL_DATA_SIZE                0x00003000UL
#define BL_DATA_END_ADDR            (BL_DATA_START_ADDR + BL_DATA_SIZE - 1UL)

#endif /* COMMON_BL_PARTITION_H */
