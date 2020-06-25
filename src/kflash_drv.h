/*
 * Copyright 2020 Jay Heng
 */

#ifndef __KFLASH_DRV_H__
#define __KFLASH_DRV_H__

#include <stdbool.h>
#include <stdint.h>
#include "flexspi_nor_flash.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define KFLASH_INSTANCE          (1)
#define KFLASH_CONFIG_OPTION     (0xc0403007)

#define KFLASH_BASE_ADDRESS      (0x30000000)
#define KFLASH_SECTOR_SIZE       (0x1000)
#define KFLASH_PAGE_SIZE         (256)

/* It is restriction of low level flash driver */
#define KFLASH_PROGRAM_ALIGNMENT (4)
/* For SDR mode, it is 1; For DDR mode, it is 2 */
#define KFLASH_PROGRAM_UNIT      (2)

typedef enum
{
    kFlashMemInfo_PageSize,
    kFlashMemInfo_SectorSize,
    kFlashMemInfo_BlockSize,
    kFlashMemInfo_TotalSize,
} kflash_mem_info_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

status_t kflash_drv_init(void);

uint32_t kflash_drv_get_info(kflash_mem_info_t flashInfo);

status_t kflash_drv_erase_region(uint32_t start, uint32_t length);

status_t kflash_drv_program_region(uint32_t dstAddr, const uint32_t *src, uint32_t length);

#ifdef __cplusplus
}
#endif

#endif // __KFLASH_DRV_H__
