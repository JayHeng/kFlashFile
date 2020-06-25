/*
 * Copyright 2020 Jay Heng
 */

#include "kflash_file.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define KFLASH_MEM_BLANK (~(0))

#define KFLASH_HDR_MAGIC_DAT (0xABECEDA8)
#define KFLASH_HDR_MAGIC_LEN (4)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static status_t kflash_check_allocation(kflash_file_t *flashFile, uint32_t memStart, uint32_t memSize, uint32_t fileSize);
static void kflash_update_active_start(kflash_file_t *flashFile);
static bool kflash_is_valid_file_found(kflash_file_t *flashFile);
static void kflash_invert_mem_data(uint8_t *src, uint32_t length);
static void kflash_pull_file_data(kflash_file_t *flashFile);
static bool kflash_is_mem_blank(uint32_t start, uint32_t length);
static status_t kflash_clear_data_region(kflash_file_t *flashFile);
static status_t kflash_clear_header_region(kflash_file_t *flashFile);
static uint32_t kflash_calculate_position_value(kflash_file_t *flashFile);
static status_t kflash_push_file_header(kflash_file_t *flashFile);
static status_t kflash_clean_file_system(kflash_file_t *flashFile);
static bool kflash_is_valid_file_range(kflash_file_t *flashFile, uint32_t offset, uint32_t size);
static bool kflash_is_file_data_changed(kflash_file_t *flashFile, uint32_t offset, uint8_t *data, uint32_t size);
static bool kflash_can_file_be_overwritten(kflash_file_t *flashFile, uint32_t offset, uint8_t *data, uint32_t size);
static status_t kflash_push_file_data(kflash_file_t *flashFile, uint32_t offset, uint32_t size, bool isOverwrite);

/*******************************************************************************
 * Variables
 ******************************************************************************/


/*******************************************************************************
 * Code
 ******************************************************************************/

status_t kflash_file_init(kflash_file_t *flashFile,
                          uint32_t memStart,
                          uint32_t memSize,
                          uint32_t fileSize)
{
    status_t status;

    /* Check params */
    if (NULL == flashFile)
    {
        return kStatus_KFLASHFILE_InitFail;
    }

    /* Init FlexSPI module and flash */
    status = kflash_drv_init();
    if (status != kStatus_Success)
    {
        return status;
    }

    /* Check and set intial file info (Const) */
    status = kflash_check_allocation(flashFile, memStart, memSize, fileSize);
    if (status != kStatus_Success)
    {
        return status;
    }

    /* Check if there is valid file */
    if (kflash_is_valid_file_found(flashFile))
    {
        kflash_pull_file_data(flashFile);
    }
    else
    {
        status = kflash_clean_file_system(flashFile);
        if (status != kStatus_Success)
        {
            return status;
        }
    }

    return kStatus_Success;
}

status_t kflash_file_read(kflash_file_t *flashFile,
                          uint32_t offset,
                          uint8_t *data,
                          uint32_t size)
{
    if (kflash_is_valid_file_range(flashFile, offset, size))
    {
        memcpy(data, &flashFile->buffer[offset], size);
    }
    else
    {
        return kStatus_KFLASHFILE_RangeError;
    }

    return kStatus_Success;
}

status_t kflash_file_save(kflash_file_t *flashFile,
                          uint32_t offset,
                          uint8_t *data,
                          uint32_t size)
{
    status_t status;

    /* Validate the user range of file data */
    if (kflash_is_valid_file_range(flashFile, offset, size))
    {
        /* Check if the file data is changed, if no, nothing needs to do */
        if (!kflash_is_file_data_changed(flashFile, offset, data, size))
        {
            return kStatus_Success;
        }
        /* Check if new file data can be overwritten in flash directly */
        else if (kflash_can_file_be_overwritten(flashFile, offset, data, size))
        {
            memcpy(&flashFile->buffer[offset], data, size);
            status = kflash_push_file_data(flashFile, offset, size, true);
            if (status != kStatus_Success)
            {
                return status;
            }
        }
        else
        {
            uint32_t sectorSize = kflash_drv_get_info(kFlashMemInfo_SectorSize);
            uint32_t managedEnd = flashFile->managedStart + flashFile->managedSize - sectorSize * KFLASH_HDR_SECTORS;
            uint32_t recordSwitchPos = (sectorSize - KFLASH_HDR_MAGIC_LEN) / sizeof(KFLASH_HDR_POS_TYPE);

            /* Find new position for file data */
            if ((managedEnd - flashFile->activedSize) == flashFile->activedStart)
            {
                /* One round finished, We need to program file data from the managedStart.
                 * We just erase the first data sector of managed flash region here.
                 */
                status = kflash_drv_erase_region(flashFile->managedStart, sectorSize);
                if (status != kStatus_Success)
                {
                    return status;
                }
                flashFile->activedStart = flashFile->managedStart;
            }
            else if (flashFile->managedStart == flashFile->activedStart)
            {
                /* if it is the first round, nothing needs to do here.
                 * if it is not first round, we should erase the second to the last data sector.
                 */
                if (!kflash_is_mem_blank(flashFile->managedStart + sectorSize, sectorSize))
                {
                    status = kflash_drv_erase_region(flashFile->managedStart + sectorSize, flashFile->managedSize  - sectorSize * (KFLASH_HDR_SECTORS + 1));
                    if (status != kStatus_Success)
                    {
                        return status;
                    }
                }
                flashFile->activedStart += flashFile->activedSize;
            }
            else
            {
                flashFile->activedStart += flashFile->activedSize;
            }

            /* Flush file data into new position */
            memcpy(&flashFile->buffer[offset], data, size);
            status = kflash_push_file_data(flashFile, 0, flashFile->activedSize, false);
            if (status != kStatus_Success)
            {
                return status;
            }

            /* Handle file header to record new position */
            if (!((flashFile->recordedPos + 1) % recordSwitchPos))
            {
                uint32_t posSectorAddr;
                uint32_t preRecordIdx;

                flashFile->recordedIdx = flashFile->recordedIdx % KFLASH_HDR_SECTORS + 1;
                posSectorAddr = managedEnd + sectorSize * (flashFile->recordedIdx - 1);
                /* Need to erase next header sector */
                if (!kflash_is_mem_blank(posSectorAddr, sectorSize))
                {
                    status = kflash_drv_erase_region(posSectorAddr, sectorSize);
                    if (status != kStatus_Success)
                    {
                        return status;
                    }
                }
                flashFile->recordedPos = 0;
                kflash_push_file_header(flashFile);

                preRecordIdx = (flashFile->recordedIdx == 1) ? KFLASH_HDR_SECTORS : (flashFile->recordedIdx - 1);
                posSectorAddr = managedEnd + sectorSize * (preRecordIdx - 1);
                /* Need to clear previous header sector */
                if (!kflash_is_mem_blank(posSectorAddr, sectorSize))
                {
                    status = kflash_drv_erase_region(posSectorAddr, sectorSize);
                    if (status != kStatus_Success)
                    {
                        return status;
                    }
                }
            }
            else
            {
                flashFile->recordedPos++;
                kflash_push_file_header(flashFile);
            }
        }
    }
    else
    {
        return kStatus_KFLASHFILE_RangeError;
    }

    return kStatus_Success;
}

status_t kflash_file_deinit(kflash_file_t *flashFile)
{
    return kflash_clear_header_region(flashFile);
}

static status_t kflash_check_allocation(kflash_file_t *flashFile, uint32_t memStart, uint32_t memSize, uint32_t fileSize)
{
    uint32_t pageSize;
    uint32_t sectorSize;
    uint32_t totalSize;

    if (memStart >= KFLASH_BASE_ADDRESS)
    {
        memStart -= KFLASH_BASE_ADDRESS;
    }

    sectorSize = kflash_drv_get_info(kFlashMemInfo_SectorSize);
    if (memStart % sectorSize)
    {
        return kStatus_KFLASHFILE_AllocateFail;
    }
    flashFile->managedStart = memStart + KFLASH_BASE_ADDRESS;

    if ((memSize % sectorSize) || ((memSize / sectorSize) < KFLASH_MIN_SECTORS))
    {
        return kStatus_KFLASHFILE_AllocateFail;
    }
    flashFile->managedSize = memSize;

    totalSize = kflash_drv_get_info(kFlashMemInfo_TotalSize);;
    if (totalSize && ((memStart + memSize) > totalSize))
    {
        return kStatus_KFLASHFILE_AllocateFail;
    }

    pageSize = kflash_drv_get_info(kFlashMemInfo_PageSize);
    if ((fileSize % pageSize) || (fileSize > sectorSize) || (sectorSize % fileSize) || (fileSize > sizeof(flashFile->buffer)))
    {
        return kStatus_KFLASHFILE_AllocateFail;
    }
    flashFile->activedSize = fileSize;

    return kStatus_Success;
}

static void kflash_update_active_start(kflash_file_t *flashFile)
{
    uint32_t sectorSize = kflash_drv_get_info(kFlashMemInfo_SectorSize);
    uint32_t magicAddr = flashFile->managedStart + flashFile->managedSize - sectorSize * (KFLASH_HDR_SECTORS + 1 - flashFile->recordedIdx);
    uint32_t posValue = *(KFLASH_HDR_POS_TYPE *)(magicAddr + KFLASH_HDR_MAGIC_LEN + flashFile->recordedPos * sizeof(KFLASH_HDR_POS_TYPE));

    flashFile->activedStart = flashFile->managedStart + flashFile->activedSize * posValue;
}

static bool kflash_is_valid_file_found(kflash_file_t *flashFile)
{
    uint32_t sectorSize = kflash_drv_get_info(kFlashMemInfo_SectorSize);
    uint32_t magicAddr = flashFile->managedStart + flashFile->managedSize;

    /* Clear file header recorder info */
    flashFile->recordedIdx = 0;

    /* File header should be in the last two sectors of managed flash region*/
    for (uint32_t idx = KFLASH_HDR_SECTORS; idx > 0; idx--)
    {
        magicAddr -= sectorSize;
        if (KFLASH_HDR_MAGIC_DAT == (*(uint32_t *)magicAddr))
        {
            uint32_t posAddr = magicAddr + KFLASH_HDR_MAGIC_LEN;
            uint32_t posEnd = magicAddr + sectorSize;
            flashFile->recordedPos = 0;
            while (posAddr < posEnd)
            {
                if ((KFLASH_HDR_POS_TYPE)KFLASH_MEM_BLANK == (*(KFLASH_HDR_POS_TYPE *)posAddr))
                {
                    /* recordedPos means current active file position*/
                    if (flashFile->recordedPos)
                    {
                        flashFile->recordedPos--;
                    }
                    break;
                }
                else
                {
                    posAddr += sizeof(KFLASH_HDR_POS_TYPE);
                    flashFile->recordedPos++;
                }
            }
            /* Record file header info */
            flashFile->recordedIdx = idx;
            kflash_update_active_start(flashFile);
            break;
        }
    }

    return (flashFile->recordedIdx != 0);
}

static void kflash_invert_mem_data(uint8_t *src, uint32_t length)
{
    while (length--)
    {
        *src = ~(*src);
        src++;
    }
}

static void kflash_pull_file_data(kflash_file_t *flashFile)
{
    memcpy(flashFile->buffer, (uint8_t *)flashFile->activedStart, flashFile->activedSize);

    /* As we always program inverted file data to flash, that's why we do this here */
    kflash_invert_mem_data(flashFile->buffer, flashFile->activedSize);
}

static bool kflash_is_mem_blank(uint32_t start, uint32_t length)
{
    while (length--)
    {
        if ((uint8_t)KFLASH_MEM_BLANK != (*(uint8_t *)start))
        {
            return false;
        }
        start++;
    }

    return true;
}

static status_t kflash_clear_data_region(kflash_file_t *flashFile)
{
    status_t status;
    uint32_t datSectorIdx = 0;
    uint32_t sectorSize = kflash_drv_get_info(kFlashMemInfo_SectorSize);
    uint32_t totalDatSectors = flashFile->managedSize / sectorSize - KFLASH_HDR_SECTORS;
    uint32_t eraseAddr = flashFile->managedStart;

    /* Make sure all data sectors are blank */
    while (datSectorIdx < totalDatSectors)
    {
        if (!kflash_is_mem_blank(eraseAddr, sectorSize))
        {
            status = kflash_drv_erase_region(eraseAddr, sectorSize);
            if (status != kStatus_Success)
            {
                return status;
            }
        }
        datSectorIdx++;
        eraseAddr += sectorSize;
    }

    return kStatus_Success;
}
static status_t kflash_clear_header_region(kflash_file_t *flashFile)
{
    status_t status;
    uint32_t hdrSectorIdx = 0;
    uint32_t sectorSize = kflash_drv_get_info(kFlashMemInfo_SectorSize);
    uint32_t eraseAddr = flashFile->managedStart + flashFile->managedSize - sectorSize * KFLASH_HDR_SECTORS;

    /* Make sure all header sectors are blank */
    while (hdrSectorIdx < KFLASH_HDR_SECTORS)
    {
        if (!kflash_is_mem_blank(eraseAddr, sectorSize))
        {
            status = kflash_drv_erase_region(eraseAddr, sectorSize);
            if (status != kStatus_Success)
            {
                return status;
            }
        }
        hdrSectorIdx++;
        eraseAddr += sectorSize;
    }

    return kStatus_Success;
}

static uint32_t kflash_calculate_position_value(kflash_file_t *flashFile)
{
    return ((flashFile->activedStart - flashFile->managedStart) / flashFile->activedSize);
}

static status_t kflash_push_file_header(kflash_file_t *flashFile)
{
    status_t status;
    uint32_t sectorSize = kflash_drv_get_info(kFlashMemInfo_SectorSize);
    uint32_t magicAddr = flashFile->managedStart + flashFile->managedSize - sectorSize * (KFLASH_HDR_SECTORS + 1 - flashFile->recordedIdx);
    uint32_t magicValue = KFLASH_HDR_MAGIC_DAT;
    uint32_t posAddr = magicAddr + KFLASH_HDR_MAGIC_LEN + flashFile->recordedPos * sizeof(KFLASH_HDR_POS_TYPE);
    uint32_t posValue = kflash_calculate_position_value(flashFile);

    if ((uint32_t)KFLASH_MEM_BLANK == (*(uint32_t *)magicAddr))
    {
        status = kflash_drv_program_region(magicAddr, &magicValue, KFLASH_HDR_MAGIC_LEN);
        if (status != kStatus_Success)
        {
            return status;
        }
    }

    if ((KFLASH_HDR_POS_TYPE)KFLASH_MEM_BLANK == (*(KFLASH_HDR_POS_TYPE *)posAddr))
    {
        status = kflash_drv_program_region(posAddr, &posValue, sizeof(KFLASH_HDR_POS_TYPE));
        if (status != kStatus_Success)
        {
            return status;
        }
    }
    else
    {
        return kStatus_KFLASHFILE_PositionError;
    }

    return kStatus_Success;
}

static status_t kflash_clean_file_system(kflash_file_t *flashFile)
{
    status_t status;

    status = kflash_clear_data_region(flashFile);
    if (status != kStatus_Success)
    {
        return status;
    }

    status = kflash_clear_header_region(flashFile);
    if (status != kStatus_Success)
    {
        return status;
    }

    /* Set more file info (Volatile) */
    flashFile->activedStart = flashFile->managedStart;
    flashFile->recordedIdx = 1;
    flashFile->recordedPos = 0;
    memset(flashFile->buffer, 0x0, flashFile->activedSize);

    /* Fill initial file header into flash */
    kflash_push_file_header(flashFile);

    return kStatus_Success;
}

static bool kflash_is_valid_file_range(kflash_file_t *flashFile, uint32_t offset, uint32_t size)
{
    return ((offset + size) <= flashFile->activedSize);
}

static bool kflash_is_file_data_changed(kflash_file_t *flashFile, uint32_t offset, uint8_t *data, uint32_t size)
{
    return (memcmp(&flashFile->buffer[offset], data, size) != 0);
}

static bool kflash_can_file_be_overwritten(kflash_file_t *flashFile, uint32_t offset, uint8_t *data, uint32_t size)
{
    while (size--)
    {
        /* Check if src and dest are the same, or src is 0x00 if they are different */
        if ((memcmp(&flashFile->buffer[offset], data, 1) != 0) && flashFile->buffer[offset])
        {
            return false;
        }
        offset++;
        data++;
    }

    return true;
}

static status_t kflash_push_file_data(kflash_file_t *flashFile, uint32_t offset, uint32_t size, bool isOverwrite)
{
    status_t status;

    if (isOverwrite)
    {
        size = ALIGN_UP(offset + size, (uint32_t)KFLASH_PROGRAM_ALIGNMENT);
        offset = ALIGN_DOWN(offset, (uint32_t)KFLASH_PROGRAM_ALIGNMENT);
        size -= offset;
    }
    else
    {
        if (!kflash_is_mem_blank(flashFile->activedStart, flashFile->activedSize))
        {
            return kStatus_KFLASHFILE_PositionError;
        }
        offset = 0;
        size = flashFile->activedSize;
    }

    /* We should program inverted file data to flash, so we have more chance to overwrite the flash */
    kflash_invert_mem_data(&flashFile->buffer[offset], size);

    status = kflash_drv_program_region(flashFile->activedStart, (const uint32_t *)&flashFile->buffer[offset], size);

    /* We need to recover file data back */
    kflash_invert_mem_data(&flashFile->buffer[offset], size);

    return status;
}

