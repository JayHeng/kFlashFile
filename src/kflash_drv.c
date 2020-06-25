/*
 * Copyright 2020 Jay Heng
 */

#include "kflash_drv.h"
#include "fsl_cache.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/


/*******************************************************************************
 * Prototypes
 ******************************************************************************/


/*******************************************************************************
 * Variables
 ******************************************************************************/

static flexspi_nor_config_t s_flashConfig;

/*******************************************************************************
 * Code
 ******************************************************************************/

status_t kflash_drv_init(void)
{
    status_t status;

    serial_nor_config_option_t configOption;
    configOption.option0.U = KFLASH_CONFIG_OPTION;

    memset(&s_flashConfig, 0x0, sizeof(flexspi_nor_config_t));

    status = flexspi_nor_get_config(KFLASH_INSTANCE, &s_flashConfig, &configOption);
    if (status != kStatus_Success)
    {
        return status;
    }

    status = flexspi_nor_flash_init(KFLASH_INSTANCE, &s_flashConfig);
    if (status != kStatus_Success)
    {
        return status;
    }

    return kStatus_Success;
}

uint32_t kflash_drv_get_info(kflash_mem_info_t flashInfo)
{
    if (kFlashMemInfo_PageSize == flashInfo)
    {
        if (s_flashConfig.pageSize)
        {
            return s_flashConfig.pageSize;
        }
        else
        {
            return KFLASH_PAGE_SIZE;
        }
    }
    else if (kFlashMemInfo_SectorSize == flashInfo)
    {
        if (s_flashConfig.sectorSize)
        {
            return s_flashConfig.sectorSize;
        }
        else
        {
            return KFLASH_SECTOR_SIZE;
        }
    }
    else if (kFlashMemInfo_BlockSize == flashInfo)
    {
        return s_flashConfig.blockSize;
    }
    else if (kFlashMemInfo_TotalSize == flashInfo)
    {
        if (s_flashConfig.memConfig.sflashA1Size)
        {
            return s_flashConfig.memConfig.sflashA1Size;
        }
        else if (s_flashConfig.memConfig.sflashA2Size)
        {
            return s_flashConfig.memConfig.sflashA2Size;
        }
        else if (s_flashConfig.memConfig.sflashB1Size)
        {
            return s_flashConfig.memConfig.sflashB1Size;
        }
        else if (s_flashConfig.memConfig.sflashB2Size)
        {
            return s_flashConfig.memConfig.sflashB2Size;
        }
    }

    return 0;
}

status_t kflash_drv_erase_region(uint32_t start, uint32_t length)
{
    status_t status;

    status = flexspi_nor_flash_erase(KFLASH_INSTANCE, &s_flashConfig, start - KFLASH_BASE_ADDRESS, length);
    if (status != kStatus_Success)
    {
        return status;
    }

    DCACHE_InvalidateByRange(start, length);

    return kStatus_Success;
}

status_t kflash_drv_program_region(uint32_t dstAddr, const uint32_t *src, uint32_t length)
{
    status_t status;

    status = flexspi_nor_flash_program(KFLASH_INSTANCE, &s_flashConfig, dstAddr - KFLASH_BASE_ADDRESS, src, length);
    if (status != kStatus_Success)
    {
        return status;
    }

    DCACHE_InvalidateByRange(dstAddr, length);

    return kStatus_Success;
}

