/*
 * Copyright 2018 - 2019 NXP
 *
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#if !defined(__TARGET_CONFIG_H__)
#define __TARGET_CONFIG_H__

#include <stdint.h>

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

//! @brief Constants for FlexSPI features.
enum
{
    kFlexSpi_AhbMemoryMaxSizeMB = (256u * 1024u * 1024u),
};

//! @brief Version constants for the target.
enum _target_version_constants
{
    kTarget_Version_Name = 'T',
    kTarget_Version_Major = 2,
    kTarget_Version_Minor = 0,
    kTarget_Version_Bugfix = 0
};

//!@brief FlexSPI related definitions
enum
{
    kFlexSpi1_AMBA_Base = 0x30000000u,
    kFlexSpi1_ALIAS_Base = 0x08000000u,
    kFlexSpi2_AMBA_Base = 0x60000000u,
    kFlexSpi_Key_Store_Offset = 0x800,
};

//!@brief Memory index definitions
enum
{
    kIndexITCM = 0,
    kIndexDTCM = 1,
    kIndexOCRAM = 2,
    kIndexFlexSpi1 = 3,
    kIndexFlexSpi1Alias = 4,
    kIndexFlexSpi2 = 5,
    kIndexSemc = 6,
};

//!@brief PIT backward compatible defintion
#define kCLOCK_Pit kCLOCK_Pit1

//!@brief FLEXSPI clock definitions
enum
{
    kFlexSpiSerialClk_30MHz = 1,
    kFlexSpiSerialClk_50MHz = 2,
    kFlexSpiSerialClk_60MHz = 3,
    kFlexSpiSerialClk_80MHz = 4,
    kFlexSpiSerialClk_100MHz = 5,
    kFlexSpiSerialClk_120MHz = 6,
    kFlexSpiSerialClk_133MHz = 7,
    kFlexSpiSerialClk_166MHz = 8,
    kFlexSpiSerialClk_200MHz = 9,
};

//!@brief FLEXSPI instnaces
enum
{
    kFlexspiInstance_1 = 1,
    kFlexspiInstance_2 = 2,
};

//!@brief FLEXSPI Boot Clock Source
enum
{
    kFlexSpiBootClkcSrc = 4,
};

//!@brief LPSPI clock definitions
enum
{
    kRecoveryBoot_LpSpiRootClkFreq = 40000000ul,
};

//!@brief Number of MPU entries
#define MPU_ENTRY_ITCM_INDEX (3)
#define MPU_ENTRY_DTCM_INDEX (4)
#define MPU_ENTRY_OCRAM_INDEX (5)
#define MPU_ENTRY_IMG_MEM_INDEX (6)

#define MPU_ENTRIES (8)

//!@brief ROMCP related definitions
#define ROMCP_BASE (0x40CA4000u) //!@brief ROMCP base address
//!@brief ROM Pach entries
enum
{
    kRompatchFuse_StartIndex = 0x90,
    kRomPatchEntries_Default = 32 - 1,
    kRomPatchEntries_Max = kRomPatchEntries_Default,
};

enum
{
    kMEM_ROM_BASE = 0x00200000u,
    kMEM_DTCM_BASE = 0x20000000u,
    kVectorTable_Size = 0x400,
};

enum
{
    kProduct_USB_PID = 0x013d,
    kProduct_USB_VID = 0x1fc9,
};

// Default deley cell value
#define FLEXSPI_DELAY_CELL_SDR_DEFAULT 41
#define FLEXSPI_DELAY_CELL_DDR_DEFAULT 21

//!@brief ROMCP lock related definitions
enum
{
    kStickyBits_ROMCP_Lock = 5u,
    kStickyBits_ROMCP_Lock_Mask = (1ul << kStickyBits_ROMCP_Lock),
};

//!@brief ROM Readout Protection related definitions
enum
{
    kStickyBits_RomReadout_Lock = 3u,
    kStickyBits_RomReadout_Lock_Mask = (1ul << kStickyBits_RomReadout_Lock),
};

enum
{
    kLpsrGpr11_ReadoutLogicActive = 24,
    kLpsrGpr11_ReadoutLogicActive_Mask = (1ul << kLpsrGpr11_ReadoutLogicActive),
};

#define LPSR_GPR41 (*(volatile uint32_t *)0x40c0c0a4)

//#define XSPI_FLASH_DUMMY_CYCLE_PROBE_OFFSET (0x400)

//!@brief FLEXSPI NOR configuration macros for SIP package
#define FLEXSPI_SIP_NOR_DEFAULT_CONFIG()          \
    do                                            \
    {                                             \
        memCfg->sflashA1Size = 0;                 \
        memCfg->sflashA2Size = 0;                 \
        memCfg->sflashB1Size = 16 * 1024u * 1024; \
        memCfg->sflashB2Size = 0;                 \
    } while (0)

#define FLEXSPI_SIP_NOR_AUTO_PROBE_CONFIG()                                         \
    do                                                                              \
    {                                                                               \
        flashAutoProbeType = kFlashAutoProbeType_QuadSpiNor;                        \
        configOption.option0.B.option_size = 1;                                     \
        configOption.option1.B.flash_connection = kSerialNorConnection_SinglePortB; \
    } while (0)

#endif // __TARGET_CONFIG_H__

#define SERIAL_BOOT_INIT (0)
#define SERIAL_BOOT_INIT_LOAD (1)
#define SERIAL_BOOT_FULL_LOAD (2)
#define MASTERBOOT_JUMP_APP (3)

extern void handle_soc_pending_debug_request(uint32_t arg);
#define MASTERBOOT_SERIAL_DOWNLOADER_HOOK(arg) \
    do                                         \
    {                                          \
        handle_soc_pending_debug_request(arg); \
    } while (0);

#define MASTERBOOT_JUMP_APP_HOOK()                             \
    do                                                         \
    {                                                          \
        handle_soc_pending_debug_request(MASTERBOOT_JUMP_APP); \
    } while (0);

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
