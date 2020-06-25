/*
 * Copyright 2020 Jay Heng
 */

#ifndef __KFLASH_FILE__
#define __KFLASH_FILE__

#include "kflash_drv.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* The Flash size used to save file header info, at least 2 sectors */
#define KFLASH_HDR_SECTORS     (2)
/* The data type used to record file data position in flash */
#define KFLASH_HDR_POS_TYPE    uint16_t   /* uint16_t or uint32_t */
/* The min size of managed flash region */
#define KFLASH_MIN_SECTORS     (KFLASH_HDR_SECTORS + 2)
/* The max size of file data */
#define KFLASH_MAX_FILE_SIZE   (KFLASH_PAGE_SIZE * 2)

/* KFLASH file status */
enum _kflash_file_status
{
    kStatusGroup_KFLASHFILE            = 250,
    kStatus_KFLASHFILE_InitFail        = MAKE_STATUS(kStatusGroup_KFLASHFILE, 0),
    kStatus_KFLASHFILE_AllocateFail    = MAKE_STATUS(kStatusGroup_KFLASHFILE, 1),
    kStatus_KFLASHFILE_RangeError      = MAKE_STATUS(kStatusGroup_KFLASHFILE, 2),
    kStatus_KFLASHFILE_PositionError   = MAKE_STATUS(kStatusGroup_KFLASHFILE, 3),
};

typedef struct {
    uint32_t managedStart;
    uint32_t managedSize;
    uint32_t activedStart;
    uint32_t activedSize;
    /* Valid range: 1 - KFLASH_HDR_SECTORS */
    uint32_t recordedIdx;
    /* *(KFLASH_HDR_POS_TYPE *)(hdrSectorStartAddr + 4 + recordedPos * sizeof(KFLASH_HDR_POS_TYPE)) =
       (activedStart - managedStart) / sectorSize */
    uint32_t recordedPos;
    uint8_t buffer[KFLASH_MAX_FILE_SIZE];
} kflash_file_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/* Allocate file system or reuse existing file system */
status_t kflash_file_init(kflash_file_t *flashFile, uint32_t memStart, uint32_t memSize, uint32_t fileSize);
/* Get actived file data from managed flash region */
status_t kflash_file_read(kflash_file_t *flashFile, uint32_t offset, uint8_t *data, uint32_t size);
/* Save new file data into managed flash region */
status_t kflash_file_save(kflash_file_t *flashFile, uint32_t offset, uint8_t *data, uint32_t size);
/* Just clear all the file headers for managed flash region */
status_t kflash_file_deinit(kflash_file_t *flashFile);

#ifdef __cplusplus
}
#endif

#endif // __KFLASH_FILE__
