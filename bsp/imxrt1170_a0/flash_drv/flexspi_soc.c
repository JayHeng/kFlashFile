/*
 * Copyright 2018 - 2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#include "flexspi_nor_flash.h"
#include "fusemap.h"
//#include "masterboot/masterboot.h"
//#include "microseconds/microseconds.h"
#include "peripherals_pinmux.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define FREQ_396MHz (396000000U)
#define FREQ_480MHz (480000000U)
#define FREQ_528MHz (528000000U)
#define FREQ_24MHz (24000000U)
#define FREQ_1MHz (1000000U)

//!@brief OTFAD related bitmask
#define IOMUXC_GPR34_OTFAD1_EN_MASK (1u << 1)
#define IOMUXC_GPR35_OTFAD2_EN_MASK (1u << 1)

#define FLEXSPI_REMAP_ADDR_UNIT_IN_BYTES (4096ul)

#define FLEXSPI_AHB_RX_BUFFER_SIZE (4096u)

//!@brief Address remap related defintions
typedef struct
{
    __IO uint32_t START;
    __IO uint32_t END;
    __IO uint32_t OFFSET;
} FLEXSPI_ADDR_REMAP_Type;

enum
{
    kImageSize_SecImageOffset = 0, // Image Size = Secondary Image Offset
    kImageSize_1MB = 1,
    kImageSize_2MB = 2,
    kImageSize_3MB = 3,
    kImageSize_4MB = 4,
    kImageSiz_5MB = 5,
    kImageSiz_6MB = 6,
    kImageSiz_7MB = 7,
    kImageSiz_8MB = 8,
    kImageSiz_9MB = 9,
    kImageSiz_10MB = 10,
    kImageSiz_11MB = 11,
    kImageSiz_12MB = 12,
    kImageSize_256KB = 13,
    kImageSize_512KB = 14,
    kImageSize_768KB = 15,
};

enum
{
    kHoldTime_500US = 0,
    kHoldTime_1MS = 1,
    kHoldTime_3MS = 2,
    kHoldTime_10MS = 3,
};

//!@brief FLEXSPI ROOT CLOCK soruce related definitions
enum
{
    kFlexSpiClkSrc_RC48M_Div2 = 0,
    kFlexSpiClkSrc_OSC = 1,
    kFlexSpiClkSrc_RC400M = 2,
    kFlexSpiClkSrc_RC4M_16M = 3,
    kFlexSpiClkSrc_PLL_480_PFD0 = 4,
    kFlexSpiClkSrc_PLL_528 = 5,
    kFlexSpiClkSrc_PLL_528_PFD2 = 6,
    kFlexSpiClkSrc_PLL_480 = 7,
};

enum
{
    kFlexSpiClkFreqSrc_Normal = 0,
    kFlexSpiClkFreqSrc_RC400M = 1,
    kFlexSpiClkFreqSrc_PLL480M = 2,
};

typedef struct
{
    uint8_t muxPadIndex;       // Index in IOMUXC_MUX_CTL and IOMUXC_PAD_CTL , 0 means this feature is not supported
    uint8_t muxValue;          // MUX value of Pin
    uint8_t selectInputIndex;  // index in IOMUXC_SELECT_INPUT, 0 means this feature is not supported
    uint8_t selecctInputValue; // Selece Input value
} flexspi_pad_attribute_t;

typedef struct
{
    flexspi_pad_attribute_t A_SS0;
    flexspi_pad_attribute_t A_SS1;
    flexspi_pad_attribute_t A_SCLK;
    flexspi_pad_attribute_t A_SCLKB;
    flexspi_pad_attribute_t A_DQS;
    flexspi_pad_attribute_t A_DATA[8];
    flexspi_pad_attribute_t B_SS0;
    flexspi_pad_attribute_t B_SS1;
    flexspi_pad_attribute_t B_SCLK;
    flexspi_pad_attribute_t B_SCLKB;
    flexspi_pad_attribute_t B_DQS;
    flexspi_pad_attribute_t B_DATA[8];
    uint32_t csPadSetting;
    uint32_t sclkPadSetting;
    uint32_t dataPadSetting;
    uint32_t dqsPadSetting;
} flexspi_pad_data_t;

enum
{
    kFlexspiPadDataIndex_FlexSpi1_Primary = 0,
    kFlexspiPadDataIndex_FlexSpi1_Secondary = 1,
    kFlexspiPadDataIndex_FlexSpi2_Primary = 2,
    kFlexspiPadDataIndex_FlexSpi2_Secondary = 3,
    kFlexspiPadDataIndex_Max,
};

typedef struct
{
    uint32_t muxPadIndex;
    uint32_t muxValue;
    GPIO_Type *gpio;
    uint32_t pinIndex;
} gpio_pad_data_t;

typedef struct
{
    gpio_pad_data_t SS0;
    gpio_pad_data_t SCLK;
    gpio_pad_data_t SIN;
    uint32_t padSetting;
    uint32_t clkPadSetting;
} flash_jedec_hw_pin_t;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void flexspi_nor_jedec_hw_reset(uint32_t instance);
static void flexspi_jedec_hw_reset(volatile GPIO_Type *gpio, uint32_t cs_pin, uint32_t clk_pin, uint32_t sin_pin);
bool is_flexspi_clock_enabled(uint32_t instance);

/*******************************************************************************
 * Variables
 ******************************************************************************/
// Secondary option of the DQS pin for Primary FLEXSPI1 Pin group
const flexspi_pad_attribute_t k_flexspi1_A_DQS3 = { kIOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_B1_18, 6,
                                                    kIOMUXC_FLEXSPI1_I_DQS_FA_SELECT_INPUT, 0 };
// Reset Pin Definitions
const gpio_pad_data_t k_resetPins[2] = {
    { SW_MUX_CTL_PAD_FLEXSPI_RESET_IDX, FLEXSPI_RESET_PIN_MUX, FLEXSPI_RESET_PIN_GPIO, FLEXSPI_RESET_PIN_INDEX },
    { SW_MUX_CTL_PAD_FLEXSPI_SEC_RESET_IDX, FLEXSPI_RESET_SEC_PIN_MUX, FLEXSPI_RESET_SEC_PIN_GPIO,
      FLEXSPI_RESET_SEC_PIN_INDEX },
};

#define GET_JEDEC_HW_RST_PIN_LIST(inst, port, group) ((inst - 1) * 4 + (group)*2 + port)

#define GET_FLEXSPI_PIN_TBL_IDX(inst, group) ((inst - 1) * 2 + group)

enum
{
    kFlexSpi_PortA = 0,
    kFlexSpi_PortB = 1,
};

enum
{
    kFlexSpi_PrimaryGroup = 0,
    kFlexSpi_SecondaryGroup = 1,
};

// JEDEC HW Reset Pin Group
const flash_jedec_hw_pin_t k_jedecHwResetPins[] = {
    // FLEXSPI1A
    {
        .SS0 = { SW_MUX_CTL_PAD_FLEXSPI1A_SS0_B_IDX, 5, GPIO4, 15 },
        .SCLK = { SW_MUX_CTL_PAD_FLEXSPI1A_SCLK_IDX, 5, GPIO4, 16 },
        .SIN = { SW_MUX_CTL_PAD_FLEXSPI1A_DATA0_IDX, 5, GPIO4, 17 },
        .padSetting = 0x4,    // Pull up, high driver
        .clkPadSetting = 0x8, // Pull down, high driver
    },
    // FLEXSPI1B
    {
        .SS0 = { SW_MUX_CTL_PAD_FLEXSPI1B_SS0_B_IDX, 5, GPIO4, 7 },
        .SCLK = { SW_MUX_CTL_PAD_FLEXSPI1B_SCLK_IDX, 5, GPIO4, 13 },
        .SIN = { SW_MUX_CTL_PAD_FLEXSPI1B_DATA0_IDX, 5, GPIO4, 12 },
        .padSetting = 0x4,    // Pull up, high driver
        .clkPadSetting = 0x8, // Pull down, high driver
    },
    // FLEXSPI1A_SEC
    {
        .SS0 = { SW_MUX_CTL_PAD_FLEXSPI1A_SEC_SS0_B_IDX, 5, GPIO3, 17 },
        .SCLK = { SW_MUX_CTL_PAD_FLEXSPI1A_SEC_SCLK_IDX, 5, GPIO3, 18 },
        .SIN = { SW_MUX_CTL_PAD_FLEXSPI1A_SEC_DATA0_IDX, 5, GPIO3, 19 },
        .padSetting = 0xf,    // Pull up, high driver
        .clkPadSetting = 0x7, // Pull down, high driver
    },
    // FLEXSPI1B_SEC
    {
        .SS0 = { 0, 0, NULL, 0 },
        .SCLK = { 0, 0, NULL, 0 },
        .SIN = { 0, 0, NULL, 0 },
    },
    // FLEXSPI2A
    {
        .SS0 = { SW_MUX_CTL_PAD_FLEXSPI2A_SS0_B_IDX, 5, GPIO2, 22 },
        .SCLK = { SW_MUX_CTL_PAD_FLEXSPI2A_SCLK_IDX, 5, GPIO2, 20 },
        .SIN = { SW_MUX_CTL_PAD_FLEXSPI2A_DATA0_IDX, 5, GPIO2, 23 },
        .padSetting = 0x4,    // Pull up, high driver
        .clkPadSetting = 0x8, // Pull down, high driver
    },
    // FLEXSPI2B
    {
        .SS0 = { SW_MUX_CTL_PAD_FLEXSPI2B_SS0_B_IDX, 5, GPIO2, 18 },
        .SCLK = { SW_MUX_CTL_PAD_FLEXSPI2B_SCLK_IDX, 5, GPIO2, 19 },
        .SIN = { SW_MUX_CTL_PAD_FLEXSPI2B_DATA0_IDX, 5, GPIO2, 16 },
        .padSetting = 0x4,    // Pull up, high driver
        .clkPadSetting = 0x8, // Pull down, high driver
    },
    // FLEXSPI2A_SEC
    {
        .SS0 = { SW_MUX_CTL_PAD_FLEXSPI2A_SEC_SS0_B_IDX, 5, GPIO4, 3 },
        .SCLK = { SW_MUX_CTL_PAD_FLEXSPI2A_SEC_SCLK_IDX, 5, GPIO4, 4 },
        .SIN = { SW_MUX_CTL_PAD_FLEXSPI2A_SEC_DATA0_IDX, 5, GPIO4, 5 },
        .padSetting = 0x4,    // Pull up, high driver
        .clkPadSetting = 0x8, // Pull down, high driver
    },
    // FLEXSPI2B_SEC
    {
        .SS0 = { 0, 0, NULL, 0 },
        .SCLK = { 0, 0, NULL, 0 },
        .SIN = { 0, 0, NULL, 0 },
    },
};

// FlexSPI NOR Pin setting list
const flexspi_pad_data_t k_flexspi_pads[kFlexspiPadDataIndex_Max] = {
    // FLEXSPI1 PRIMARY
    {
        .A_SS0 = { SW_PAD_CTL_PAD_FLEXSPI1A_SS0_B_IDX, 1, 0, 0 },
        .A_SS1 = { SW_PAD_CTL_PAD_FLEXSPI1A_SS1_B_IDX, 9, 0, 0 },
        .A_SCLK = { SW_PAD_CTL_PAD_FLEXSPI1A_SCLK_IDX, 1, kIOMUXC_FLEXSPI1_I_SCK_FA_SELECT_INPUT, 1 },
        .A_SCLKB = { SW_PAD_CTL_PAD_FLEXSPI1A_SCLK_B_IDX, 1, kIOMUXC_FLEXSPI1_I_SCK_FB_SELECT_INPUT, 1 },
        .A_DQS = { SW_MUX_CTL_PAD_FLEXSPI1A_DQS_IDX, 1, kIOMUXC_FLEXSPI1_I_DQS_FA_SELECT_INPUT, 2 },
        .A_DATA =
            {
                // 0
                { SW_PAD_CTL_PAD_FLEXSPI1A_DATA0_IDX, 1, kIOMUXC_FLEXSPI1_I_IO_FA_SELECT_INPUT_0, 1 },
                // 1
                { SW_PAD_CTL_PAD_FLEXSPI1A_DATA1_IDX, 1, kIOMUXC_FLEXSPI1_I_IO_FA_SELECT_INPUT_1, 1 },
                // 2
                { SW_PAD_CTL_PAD_FLEXSPI1A_DATA2_IDX, 1, kIOMUXC_FLEXSPI1_I_IO_FA_SELECT_INPUT_2, 1 },
                // 3
                { SW_PAD_CTL_PAD_FLEXSPI1A_DATA3_IDX, 1, kIOMUXC_FLEXSPI1_I_IO_FA_SELECT_INPUT_3, 1 },
                // 4
                { SW_PAD_CTL_PAD_FLEXSPI1B_DATA0_IDX, 1, kIOMUXC_FLEXSPI1_I_IO_FB_SELECT_INPUT_0, 1 },
                // 5
                { SW_PAD_CTL_PAD_FLEXSPI1B_DATA1_IDX, 1, kIOMUXC_FLEXSPI1_I_IO_FB_SELECT_INPUT_1, 1 },
                // 6
                { SW_PAD_CTL_PAD_FLEXSPI1B_DATA2_IDX, 1, kIOMUXC_FLEXSPI1_I_IO_FB_SELECT_INPUT_2, 1 },
                // 7
                { SW_PAD_CTL_PAD_FLEXSPI1B_DATA3_IDX, 1, kIOMUXC_FLEXSPI1_I_IO_FB_SELECT_INPUT_3, 1 },
            },
        .B_SS0 = { SW_MUX_CTL_PAD_FLEXSPI1B_SS0_B_IDX, 8, 0, 0 },
        .B_SS1 = { SW_MUX_CTL_PAD_FLEXSPI1B_SS1_B_IDX, 9, 0, 0 },
        .B_SCLK = { SW_MUX_CTL_PAD_FLEXSPI1B_SCLK_IDX, 1, kIOMUXC_FLEXSPI1_I_SCK_FB_SELECT_INPUT, 1 },
        .B_DQS = { SW_MUX_CTL_PAD_FLEXSPI1B_DQS_IDX, 8, 0, 0 },
        .B_DATA =
            {
                // 0
                { SW_PAD_CTL_PAD_FLEXSPI1B_DATA0_IDX, 1, kIOMUXC_FLEXSPI1_I_IO_FB_SELECT_INPUT_0, 1 },
                // 1
                { SW_PAD_CTL_PAD_FLEXSPI1B_DATA1_IDX, 1, kIOMUXC_FLEXSPI1_I_IO_FB_SELECT_INPUT_1, 1 },
                // 2
                { SW_PAD_CTL_PAD_FLEXSPI1B_DATA2_IDX, 1, kIOMUXC_FLEXSPI1_I_IO_FB_SELECT_INPUT_2, 1 },
                // 3
                { SW_PAD_CTL_PAD_FLEXSPI1B_DATA3_IDX, 1, kIOMUXC_FLEXSPI1_I_IO_FB_SELECT_INPUT_3, 1 },
            },
        .csPadSetting = IOMUXC_SW_PAD_CTL_PAD_PULL(3),   // No-Pull
        .sclkPadSetting = IOMUXC_SW_PAD_CTL_PAD_PULL(3), // No-Pull
        .dataPadSetting = IOMUXC_SW_PAD_CTL_PAD_PULL(3), // No-Pull
        .dqsPadSetting = IOMUXC_SW_PAD_CTL_PAD_PULL(2),  // Pull down
    },
    // FLEXSPI1 SECONDARY
    {
        .A_SS0 = { SW_PAD_CTL_PAD_FLEXSPI1A_SEC_SS0_B_IDX, 3, 0, 0 },
        .A_SCLK = { SW_PAD_CTL_PAD_FLEXSPI1A_SEC_SCLK_IDX, 3, kIOMUXC_FLEXSPI1_I_SCK_FA_SELECT_INPUT, 0 },
        .A_SCLKB = { SW_PAD_CTL_PAD_FLEXSPI1B_SEC_SCLK_IDX, 3, kIOMUXC_FLEXSPI1_I_SCK_FB_SELECT_INPUT, 0 },
        .A_DQS = { SW_MUX_CTL_PAD_FLEXSPI1A_SEC_DQS_IDX, 3, kIOMUXC_FLEXSPI1_I_DQS_FA_SELECT_INPUT, 1 },
        .A_DATA =
            {
                // 0
                { SW_PAD_CTL_PAD_FLEXSPI1A_SEC_DATA0_IDX, 3, kIOMUXC_FLEXSPI1_I_IO_FA_SELECT_INPUT_0, 0 },
                // 1
                { SW_PAD_CTL_PAD_FLEXSPI1A_SEC_DATA1_IDX, 3, kIOMUXC_FLEXSPI1_I_IO_FA_SELECT_INPUT_1, 0 },
                // 2
                { SW_PAD_CTL_PAD_FLEXSPI1A_SEC_DATA2_IDX, 3, kIOMUXC_FLEXSPI1_I_IO_FA_SELECT_INPUT_2, 0 },
                // 3
                { SW_PAD_CTL_PAD_FLEXSPI1A_SEC_DATA3_IDX, 3, kIOMUXC_FLEXSPI1_I_IO_FA_SELECT_INPUT_3, 0 },
                // 4
                { SW_PAD_CTL_PAD_FLEXSPI1B_SEC_DATA0_IDX, 3, kIOMUXC_FLEXSPI1_I_IO_FB_SELECT_INPUT_0, 0 },
                // 5
                { SW_PAD_CTL_PAD_FLEXSPI1B_SEC_DATA1_IDX, 3, kIOMUXC_FLEXSPI1_I_IO_FB_SELECT_INPUT_1, 0 },
                // 6
                { SW_PAD_CTL_PAD_FLEXSPI1B_SEC_DATA2_IDX, 3, kIOMUXC_FLEXSPI1_I_IO_FB_SELECT_INPUT_2, 0 },
                // 7
                { SW_PAD_CTL_PAD_FLEXSPI1B_SEC_DATA3_IDX, 3, kIOMUXC_FLEXSPI1_I_IO_FB_SELECT_INPUT_3, 0 },
            },
        .csPadSetting = IOMUXC_SW_PAD_CTL_PAD_SRE(1) | IOMUXC_SW_PAD_CTL_PAD_DSE(1),   // No pull, fast drive
        .sclkPadSetting = IOMUXC_SW_PAD_CTL_PAD_SRE(1) | IOMUXC_SW_PAD_CTL_PAD_DSE(1), // No pull, fast drive
        .dataPadSetting = IOMUXC_SW_PAD_CTL_PAD_SRE(1) | IOMUXC_SW_PAD_CTL_PAD_DSE(1), // No pull, fast drive
        .dqsPadSetting = IOMUXC_SW_PAD_CTL_PAD_SRE(1) | IOMUXC_SW_PAD_CTL_PAD_DSE(1) |
                         IOMUXC_SW_PAD_CTL_PAD_PUE(1), // Pull down
    },
    // FLEXSPI2 PRIMARY
    {
        .A_SS0 = { SW_PAD_CTL_PAD_FLEXSPI2A_SS0_B_IDX, 4, 0, 0 },
        .A_SCLK = { SW_PAD_CTL_PAD_FLEXSPI2A_SCLK_IDX, 4, kIOMUXC_FLEXSPI2_I_SCK_FA_SELECT_INPUT, 0 },
        .A_SCLKB = { SW_PAD_CTL_PAD_FLEXSPI2A_SCLK_B_IDX, 4, 0, 0 },
        .A_DQS = { SW_MUX_CTL_PAD_FLEXSPI2A_DQS_IDX, 4, 0, 0 },
        .A_DATA =
            {
                // 0
                { SW_PAD_CTL_PAD_FLEXSPI2A_DATA0_IDX, 4, kIOMUXC_FLEXSPI2_I_IO_FA_SELECT_INPUT_0, 0 },
                // 1
                { SW_PAD_CTL_PAD_FLEXSPI2A_DATA1_IDX, 4, kIOMUXC_FLEXSPI2_I_IO_FA_SELECT_INPUT_1, 0 },
                // 2
                { SW_PAD_CTL_PAD_FLEXSPI2A_DATA2_IDX, 4, kIOMUXC_FLEXSPI2_I_IO_FA_SELECT_INPUT_2, 0 },
                // 3
                { SW_PAD_CTL_PAD_FLEXSPI2A_DATA3_IDX, 4, kIOMUXC_FLEXSPI2_I_IO_FA_SELECT_INPUT_3, 0 },
                // 4
                { SW_PAD_CTL_PAD_FLEXSPI2A_DATA4_IDX, 4, 0, 0 },
                // 5
                { SW_PAD_CTL_PAD_FLEXSPI2A_DATA5_IDX, 4, 0, 0 },
                // 6
                { SW_PAD_CTL_PAD_FLEXSPI2A_DATA6_IDX, 4, 0, 0 },
                // 7
                { SW_PAD_CTL_PAD_FLEXSPI2A_DATA7_IDX, 4, 0, 0 },
            },
        .B_SS0 = { SW_MUX_CTL_PAD_FLEXSPI2B_SS0_B_IDX, 4, 0, 0 },
        .B_SCLK = { SW_MUX_CTL_PAD_FLEXSPI2B_SCLK_IDX, 4, 0, 0 },
        .B_DQS = { SW_MUX_CTL_PAD_FLEXSPI2B_DQS_IDX, 4, 0, 0 },
        .B_DATA =
            {
                // 0
                { SW_PAD_CTL_PAD_FLEXSPI2B_DATA0_IDX, 4, 0, 0 },
                // 1
                { SW_PAD_CTL_PAD_FLEXSPI2B_DATA1_IDX, 4, 0, 0 },
                // 2
                { SW_PAD_CTL_PAD_FLEXSPI2B_DATA2_IDX, 4, 0, 0 },
                // 3
                { SW_PAD_CTL_PAD_FLEXSPI2B_DATA3_IDX, 4, 0, 0 },
                // 4
                { SW_PAD_CTL_PAD_FLEXSPI2B_DATA4_IDX, 4, 0, 0 },
                // 5
                { SW_PAD_CTL_PAD_FLEXSPI2B_DATA5_IDX, 4, 0, 0 },
                // 6
                { SW_PAD_CTL_PAD_FLEXSPI2B_DATA6_IDX, 4, 0, 0 },
                // 7
                { SW_PAD_CTL_PAD_FLEXSPI2B_DATA7_IDX, 4, 0, 0 },
            },
        .csPadSetting = IOMUXC_SW_PAD_CTL_PAD_PULL(3),   // No-pull
        .sclkPadSetting = IOMUXC_SW_PAD_CTL_PAD_PULL(3), // No-pull
        .dataPadSetting = IOMUXC_SW_PAD_CTL_PAD_PULL(3), // No-pull
        .dqsPadSetting = IOMUXC_SW_PAD_CTL_PAD_PULL(2),  // Pull down
    },
    // FLEXSPI2 SECONDARY
    {
        .A_SS0 = { SW_PAD_CTL_PAD_FLEXSPI2A_SEC_SS0_B_IDX, 6, 0, 0 },
        .A_SCLK = { SW_PAD_CTL_PAD_FLEXSPI2A_SEC_SCLK_IDX, 6, kIOMUXC_FLEXSPI2_I_SCK_FA_SELECT_INPUT, 1 },
        .A_DATA =
            {
                // 0
                { SW_PAD_CTL_PAD_FLEXSPI2A_SEC_DATA0_IDX, 6, kIOMUXC_FLEXSPI2_I_IO_FA_SELECT_INPUT_0, 1 },
                // 1
                { SW_PAD_CTL_PAD_FLEXSPI2A_SEC_DATA1_IDX, 6, kIOMUXC_FLEXSPI2_I_IO_FA_SELECT_INPUT_1, 1 },
                // 2
                { SW_PAD_CTL_PAD_FLEXSPI2A_SEC_DATA2_IDX, 6, kIOMUXC_FLEXSPI2_I_IO_FA_SELECT_INPUT_2, 1 },
                // 3
                { SW_PAD_CTL_PAD_FLEXSPI2A_SEC_DATA3_IDX, 6, kIOMUXC_FLEXSPI2_I_IO_FA_SELECT_INPUT_3, 1 },
            },
        .csPadSetting = IOMUXC_SW_PAD_CTL_PAD_PULL(3),   // No-pull
        .sclkPadSetting = IOMUXC_SW_PAD_CTL_PAD_PULL(3), // No-pull
        .dataPadSetting = IOMUXC_SW_PAD_CTL_PAD_PULL(3), // No-pull
        .dqsPadSetting = IOMUXC_SW_PAD_CTL_PAD_PULL(2),  // Pull down
    },
};

/*******************************************************************************
 * Codes
 ******************************************************************************/
static void flexspi_jedec_hw_reset(volatile GPIO_Type *gpio, uint32_t cs_pin, uint32_t clk_pin, uint32_t sin_pin)
{
    do
    {
        if (gpio == NULL)
        {
            break;
        }

        bool reset_start = false;
        //debug_printf("GPIO_ADDR=%x, cs_pin=%x, clk_pin=%x, sin_pin=%x\n", &gpio->DR, cs_pin, clk_pin, sin_pin);
        gpio->DR_SET = FLEXSPI_BITMASK(sin_pin);
        gpio->DR_SET = FLEXSPI_BITMASK(cs_pin);
        gpio->DR_CLEAR = FLEXSPI_BITMASK(clk_pin);
        gpio->GDIR |= FLEXSPI_BITMASK(cs_pin) | FLEXSPI_BITMASK(clk_pin) | FLEXSPI_BITMASK(sin_pin);
        flexspi_sw_delay_us(5);

        for (volatile uint32_t i = 0; i < 4; i++)
        {
            gpio->DR_CLEAR = FLEXSPI_BITMASK(cs_pin);
            if (!reset_start)
            {
                gpio->DR_CLEAR = FLEXSPI_BITMASK(sin_pin);
                reset_start = true;
            }
            flexspi_sw_delay_us(5);
            gpio->DR_SET = FLEXSPI_BITMASK(cs_pin);
            gpio->DR_TOGGLE = FLEXSPI_BITMASK(sin_pin);
            flexspi_sw_delay_us(5);
        }
    } while (0);
}

//!@brief Get Clock for FlexSPI peripheral
status_t flexspi_get_clock(uint32_t instance, flexspi_clock_type_t type, uint32_t *freq)
{
    uint32_t clockFrequency = 0;
    status_t status = kStatus_Success;

    if ((freq == NULL) || (type > kFlexSpiClock_IpgClock))
    {
        return kStatus_InvalidArgument;
    }

    if ((instance != kFlexspiInstance_1) && (instance != kFlexspiInstance_2))
    {
        return kStatus_InvalidArgument;
    }
#ifndef BL_TARGET_FPGA
    switch (type)
    {
        case kFlexSpiClock_CoreClock:
            clockFrequency = CLOCK_GetCoreSysClkFreq();
            break;
        //        case kFlexSpiClock_AhbClock:
        //        {
        //            // Note: In I.MXRT1060, actual AHB clock is IPG_CLOCK_ROOT
        //            clockFrequency = CLOCK_GetFreq(kCLOCK_IpgClk);
        //            debug_printf("BootROM: AHB Clock = %dMHz\n", clockFrequency / FREQ_1MHz);
        //        }
        //        break;
        case kFlexSpiClock_SerialRootClock:
        {
            uint32_t srcRootClk;
            uint32_t flexspiClkSrc = 0;
            uint32_t flexspiClkDiv = 0;
            volatile uint32_t *clkDivRegister = NULL;
            if (instance == kFlexspiInstance_1)
            {
                clkDivRegister = &CCM->CLOCK_ROOT[kCLOCK_Root_Flexspi1].CONTROL;
            }
            else if (instance == kFlexspiInstance_2)
            {
                clkDivRegister = &CCM->CLOCK_ROOT[kCLOCK_Root_Flexspi2].CONTROL;
            }
            // FLEXPI CLK SEL
            flexspiClkSrc = (*clkDivRegister & CCM_CLOCK_ROOT_CONTROL_MUX_MASK) >> CCM_CLOCK_ROOT_CONTROL_MUX_SHIFT;
            // FLEXSPI CLK DIV
            flexspiClkDiv =
                1 + ((*clkDivRegister & CCM_CLOCK_ROOT_CONTROL_DIV_MASK) >> CCM_CLOCK_ROOT_CONTROL_DIV_SHIFT);

            switch (flexspiClkSrc)
            {
                case kFlexSpiClkSrc_RC48M_Div2:
                    srcRootClk = 24000000u;
                    break;
                case kFlexSpiClkSrc_OSC:
                    if (FUSE_BOOT_OSC_REF_VALUE)
                    {
                        srcRootClk = 192000000u;
                    }
                    else
                    {
                        srcRootClk = 24000000u;
                    }
                    break;
                case kFlexSpiClkSrc_RC400M:
                    srcRootClk = 400000000u;
                    break;
                case kFlexSpiClkSrc_RC4M_16M:
                    srcRootClk = 16000000u;
                    break;
                case kFlexSpiClkSrc_PLL_480_PFD0:
                    srcRootClk = 664620000u;
                    break;
                case kFlexSpiClkSrc_PLL_528:
                    srcRootClk = 528000000u;
                    break;
                case kFlexSpiClkSrc_PLL_528_PFD2:
                    srcRootClk = 396000000u;
                    break;
                case kFlexSpiClkSrc_PLL_480:
                    srcRootClk = 480000000u;
                    break;
                default:
                    status = kStatus_OutOfRange;
                    break;
            }

            if (status != kStatus_Success)
            {
                break;
            }

            clockFrequency = srcRootClk / flexspiClkDiv;
            //debug_printf("BootROM: FlexSPI Serial Root Clock = %dMHz\n", clockFrequency / FREQ_1MHz);
        }
        break;
        default:
            status = kStatus_InvalidArgument;
            break;
    }
#else
    switch (type)
    {
        case kFlexSpiClock_CoreClock:
            clockFrequency = 24000000;
            break;
        case kFlexSpiClock_SerialRootClock:
            clockFrequency = 12000000;
            break;
        default:
            status = kStatus_InvalidArgument;
            break;
    }
#endif

    if (status == kStatus_Success)
    {
        *freq = clockFrequency;
    }

    return status;
}

static void flexspi_nor_jedec_hw_reset(uint32_t instance)
{
    if ((instance != kFlexspiInstance_1) && (instance != kFlexspiInstance_2))
    {
        return;
    }

    volatile GPIO_Type *gpio = NULL;
    uint32_t ss0_pin = 0;
    uint32_t clk_pin = 0;
    uint32_t sin_pin = 0;

    uint32_t group = FUSE_FLEXSPI_PIN_GROUP_SEL_VALUE;

    // Note:The JEDEC Hardware reset now only works on Adesto EcoXiP series, and the RT1170 doesn't support parallel
    //      Octal FLASH yet, so now, only single instance connection is supported.
    uint32_t port = kFlexSpi_PortA;
    if (FUSE_FLEXSPI_CONNECTION_SEL_VALUE == kSerialNorConnection_SinglePortB)
    {
        port = kFlexSpi_PortB;
    }

    uint32_t pinListIndex = GET_JEDEC_HW_RST_PIN_LIST(instance, port, group);

    const flash_jedec_hw_pin_t *jedecHwResetPin = &k_jedecHwResetPins[pinListIndex];

    // SS0
    IOMUXC->SW_MUX_CTL_PAD[jedecHwResetPin->SS0.muxPadIndex] = jedecHwResetPin->SS0.muxValue;
    IOMUXC->SW_PAD_CTL_PAD[jedecHwResetPin->SS0.muxPadIndex] = jedecHwResetPin->padSetting;
    // SCLK
    IOMUXC->SW_MUX_CTL_PAD[jedecHwResetPin->SCLK.muxPadIndex] = jedecHwResetPin->SCLK.muxValue;
    IOMUXC->SW_PAD_CTL_PAD[jedecHwResetPin->SCLK.muxPadIndex] = jedecHwResetPin->clkPadSetting;
    // SIN
    IOMUXC->SW_MUX_CTL_PAD[jedecHwResetPin->SIN.muxPadIndex] = jedecHwResetPin->SIN.muxValue;
    IOMUXC->SW_PAD_CTL_PAD[jedecHwResetPin->SIN.muxPadIndex] = jedecHwResetPin->padSetting;

    gpio = jedecHwResetPin->SCLK.gpio;
    ss0_pin = jedecHwResetPin->SS0.pinIndex;
    clk_pin = jedecHwResetPin->SCLK.pinIndex;
    sin_pin = jedecHwResetPin->SIN.pinIndex;

    flexspi_jedec_hw_reset(gpio, ss0_pin, clk_pin, sin_pin);
}
//!@brief Configure IOMUX for FlexSPI Peripheral
void flexspi_iomux_config(uint32_t instance, flexspi_mem_config_t *config)
{
    if ((instance != kFlexspiInstance_1) && (instance != kFlexspiInstance_2))
    {
        return;
    }

    uint32_t group = 0;

    if (config->controllerMiscOption & FLEXSPI_BITMASK(kFlexSpiMiscOffset_SecondPinMux))
    {
        //debug_printf("%s, secondary pin group is selected\n", __func__);
        group = kFlexSpi_SecondaryGroup;
    }

    // Get the index of tha pad setting entry
    uint32_t padSettingIndex = GET_FLEXSPI_PIN_TBL_IDX(instance, group);

    const flexspi_pad_data_t *padData = &k_flexspi_pads[padSettingIndex];

    uint32_t csPadCtrlValue = config->csPadSettingOverride ? config->csPadSettingOverride : padData->csPadSetting;
    uint32_t dqsPadCtrlValue = config->dqsPadSettingOverride ? config->dqsPadSettingOverride : padData->dqsPadSetting;
    uint32_t sclkPadCtrlValue =
        config->sclkPadSettingOverride ? config->sclkPadSettingOverride : padData->sclkPadSetting;
    uint32_t dataPadCtrlValue =
        config->dataPadSettingOverride ? config->dataPadSettingOverride : padData->dataPadSetting;

    /*
    debug_printf(
        "BootROM: flexspi_iomux_config(), instance=%x, csPadCtrl=%x, dqsPadCtrl=%x, sclkPadCtl=%x, dataPadCtl=%x\n",
        instance, csPadCtrlValue, dqsPadCtrlValue, sclkPadCtrlValue, dataPadCtrlValue);
    */

    if (config->sflashA1Size || config->sflashA2Size)
    {
        // Configure Chip Select Pin
        if (config->sflashA1Size)
        {
            IOMUXC->SW_MUX_CTL_PAD[padData->A_SS0.muxPadIndex] = padData->A_SS0.muxValue;
            IOMUXC->SW_PAD_CTL_PAD[padData->A_SS0.muxPadIndex] = csPadCtrlValue;
        }
        if (config->sflashA2Size)
        {
            if (padData->A_SS1.muxPadIndex)
            {
                IOMUXC->SW_MUX_CTL_PAD[padData->A_SS1.muxPadIndex] = padData->A_SS1.muxValue;
                IOMUXC->SW_PAD_CTL_PAD[padData->A_SS1.muxPadIndex] = csPadCtrlValue;
            }
        }

        // Configure Clock
        if (padData->A_SCLK.muxPadIndex)
        {
            IOMUXC->SW_MUX_CTL_PAD[padData->A_SCLK.muxPadIndex] =
                (uint32_t)padData->A_SCLK.muxValue | IOMUXC_SW_MUX_CTL_PAD_SION(1);
            IOMUXC->SW_PAD_CTL_PAD[padData->A_SCLK.muxPadIndex] = sclkPadCtrlValue;
        }

        // Configure DATA0-DATA3
        for (uint32_t i = 0; i < 4; i++)
        {
            const flexspi_pad_attribute_t *padAttribute = &padData->A_DATA[i];

            IOMUXC->SW_MUX_CTL_PAD[padAttribute->muxPadIndex] = padAttribute->muxValue;
            IOMUXC->SW_PAD_CTL_PAD[padAttribute->muxPadIndex] = dataPadCtrlValue;
            if (padAttribute->selectInputIndex)
            {
                IOMUXC->SELECT_INPUT[padAttribute->selectInputIndex] = padAttribute->selecctInputValue;
            }
        }

        // Configure Data 4-Data7
        if (config->sflashPadType == kSerialFlash_8Pads)
        {
            if (padData->A_DATA[4].muxPadIndex > 0)
            {
                for (uint32_t i = 4; i < 8; i++)
                {
                    const flexspi_pad_attribute_t *padAttribute = &padData->A_DATA[i];
                    IOMUXC->SW_MUX_CTL_PAD[padAttribute->muxPadIndex] = padAttribute->muxValue;
                    IOMUXC->SW_PAD_CTL_PAD[padAttribute->muxPadIndex] = dataPadCtrlValue;
                    if (padAttribute->selectInputIndex)
                    {
                        IOMUXC->SELECT_INPUT[padAttribute->selectInputIndex] = padAttribute->selecctInputValue;
                    }
                }
            }
        }

        // Configure DQS pad
        if ((config->readSampleClkSrc == kFlexSPIReadSampleClk_ExternalInputFromDqsPad) ||
            (config->readSampleClkSrc == kFlexSPIReadSampleClk_LoopbackFromDqsPad))
        {
            const flexspi_pad_attribute_t *dqsPadAttr = NULL;
            if (config->controllerMiscOption & FLEXSPI_BITMASK(kFlexSpiMiscOffset_SecondDqsPinMux))
            {
                dqsPadAttr = &k_flexspi1_A_DQS3;
                // Override the pad setting for this option
                dqsPadCtrlValue =
                    config->dqsPadSettingOverride ? config->dqsPadSettingOverride : IOMUXC_SW_PAD_CTL_PAD_PULL(2);
            }
            else
            {
                dqsPadAttr = &padData->A_DQS;
            }

            // FLEXSPIA_DQS
            if ((dqsPadAttr != NULL) && dqsPadAttr->muxPadIndex)
            {
                IOMUXC->SW_MUX_CTL_PAD[dqsPadAttr->muxPadIndex] =
                    (uint32_t)dqsPadAttr->muxValue | IOMUXC_SW_MUX_CTL_PAD_SION(1);
                IOMUXC->SW_PAD_CTL_PAD[dqsPadAttr->muxPadIndex] = dqsPadCtrlValue;
                if (dqsPadAttr->selectInputIndex)
                {
                    IOMUXC->SELECT_INPUT[dqsPadAttr->selectInputIndex] = dqsPadAttr->selecctInputValue;
                }
            }
        }

        // Configure Differential Clock pin
        if (flexspi_is_differential_clock_enable(config))
        {
            if (padData->A_SCLKB.muxPadIndex)
            {
                IOMUXC->SW_MUX_CTL_PAD[padData->A_SCLKB.muxPadIndex] =
                    (uint32_t)padData->A_SCLKB.muxValue | IOMUXC_SW_MUX_CTL_PAD_SION(1);
                IOMUXC->SW_PAD_CTL_PAD[padData->A_SCLKB.muxPadIndex] = sclkPadCtrlValue;
                if (padData->A_SCLKB.selectInputIndex)
                {
                    IOMUXC->SELECT_INPUT[padData->A_SCLKB.selectInputIndex] = padData->A_SCLKB.selecctInputValue;
                }
            }
        }
    } // if (config->sflashA1Size || config->sflashA2Size)

    if (config->sflashB1Size || config->sflashB2Size)
    {
        // Configure Chip Select Pin
        if (config->sflashB1Size)
        {
            if (padData->B_SS0.muxPadIndex)
            {
                IOMUXC->SW_MUX_CTL_PAD[padData->B_SS0.muxPadIndex] = padData->B_SS0.muxValue;
                IOMUXC->SW_PAD_CTL_PAD[padData->B_SS0.muxPadIndex] = csPadCtrlValue;
            }
        }
        if (config->sflashB2Size)
        {
            if (padData->B_SS1.muxPadIndex)
            {
                IOMUXC->SW_MUX_CTL_PAD[padData->B_SS1.muxPadIndex] = padData->B_SS1.muxValue;
                IOMUXC->SW_PAD_CTL_PAD[padData->B_SS1.muxPadIndex] = csPadCtrlValue;
            }
        }

        // Only configure the remaining pads if the pad setting is available on the SoC
        if (padData->B_SS0.muxPadIndex || padData->B_SS1.muxPadIndex)
        {
            // Configure Clock
            if (padData->B_SCLK.muxPadIndex)
            {
                IOMUXC->SW_MUX_CTL_PAD[padData->B_SCLK.muxPadIndex] =
                    (uint32_t)padData->B_SCLK.muxValue | IOMUXC_SW_MUX_CTL_PAD_SION(1);
                IOMUXC->SW_PAD_CTL_PAD[padData->B_SCLK.muxPadIndex] = sclkPadCtrlValue;
            }

            // Configure DATA0-DATA3
            for (uint32_t i = 0; i < 4; i++)
            {
                const flexspi_pad_attribute_t *padAttribute = &padData->B_DATA[i];
                IOMUXC->SW_MUX_CTL_PAD[padAttribute->muxPadIndex] = padAttribute->muxValue;
                IOMUXC->SW_PAD_CTL_PAD[padAttribute->muxPadIndex] = dataPadCtrlValue;
                if (padAttribute->selectInputIndex)
                {
                    IOMUXC->SELECT_INPUT[padAttribute->selectInputIndex] = padAttribute->selecctInputValue;
                }
            }

            // Configure Data 4-Data7
            if (config->sflashPadType == kSerialFlash_8Pads)
            {
                if (padData->B_DATA[4].muxPadIndex > 0)
                {
                    for (uint32_t i = 4; i < 8; i++)
                    {
                        const flexspi_pad_attribute_t *padAttribute = &padData->B_DATA[i];
                        IOMUXC->SW_MUX_CTL_PAD[padAttribute->muxPadIndex] = padAttribute->muxValue;
                        IOMUXC->SW_PAD_CTL_PAD[padAttribute->muxPadIndex] = dataPadCtrlValue;
                        if (padAttribute->selectInputIndex)
                        {
                            IOMUXC->SELECT_INPUT[padAttribute->selectInputIndex] = padAttribute->selecctInputValue;
                        }
                    }
                }
            }

            // Configure DQS pad
            if ((config->readSampleClkSrc == kFlexSPIReadSampleClk_ExternalInputFromDqsPad) ||
                (config->readSampleClkSrc == kFlexSPIReadSampleClk_LoopbackFromDqsPad))
            {
                // FLEXSPIA_DQS
                if (padData->B_DQS.muxPadIndex)
                {
                    IOMUXC->SW_MUX_CTL_PAD[padData->B_DQS.muxPadIndex] =
                        (uint32_t)padData->B_DQS.muxValue | IOMUXC_SW_MUX_CTL_PAD_SION(1);
                    IOMUXC->SW_PAD_CTL_PAD[padData->B_DQS.muxPadIndex] = dqsPadCtrlValue;
                    if (padData->B_DQS.selectInputIndex)
                    {
                        IOMUXC->SELECT_INPUT[padData->B_DQS.selectInputIndex] = padData->B_DQS.selecctInputValue;
                    }
                }
            }

            // Configure Differential Clock pin
            if (flexspi_is_differential_clock_enable(config))
            {
                if (padData->B_SCLKB.muxPadIndex)
                {
                    IOMUXC->SW_MUX_CTL_PAD[padData->B_SCLKB.muxPadIndex] =
                        (uint32_t)padData->B_SCLKB.muxValue | IOMUXC_SW_MUX_CTL_PAD_SION(1);
                    IOMUXC->SW_PAD_CTL_PAD[padData->B_SCLKB.muxPadIndex] = sclkPadCtrlValue;
                    if (padData->B_SCLKB.selectInputIndex)
                    {
                        IOMUXC->SELECT_INPUT[padData->B_SCLKB.selectInputIndex] = padData->B_SCLKB.selecctInputValue;
                    }
                }
            }
        }
    } // if (config->sflashB1Size || config->sflashB2Size)
}

void flexspi_nor_hw_reset(uint32_t instance, uint32_t reset_logic)
{
    if ((instance != kFlexspiInstance_1) && (instance != kFlexspiInstance_2))
    {
        return;
    }

    if (reset_logic == kFlashResetLogic_ResetPin)
    {
        volatile GPIO_Type *gpio = NULL;
        uint32_t pin = 0;

        uint32_t resetPinSel = FUSE_FLEXSPI_RESET_PIN_SEL_VALUE;
        const gpio_pad_data_t *padData = &k_resetPins[resetPinSel];

        IOMUXC->SW_MUX_CTL_PAD[padData->muxPadIndex] = padData->muxValue;
        IOMUXC->SW_PAD_CTL_PAD[padData->muxPadIndex] = FLEXSPI_RESET_PIN_SW_PAD_CTRL_VAL;
        gpio = padData->gpio;
        pin = padData->pinIndex;

        // High
        gpio->DR_SET = (1U << pin);

        // Output mode
        gpio->GDIR |= (1U << pin);

        // High
        gpio->DR_SET = (1U << pin);
        flexspi_sw_delay_us(250);

        // Low
        gpio->DR_CLEAR = (1U << pin);
        flexspi_sw_delay_us(250);

        // High
        gpio->DR_SET = (1U << pin);
        flexspi_sw_delay_us(500);

        // Restore GPIO setting to default mode
        gpio->GDIR &= ~(1U << pin);
        gpio->DR_CLEAR = (1U << pin);
    }
    else if (reset_logic == kFlashResetLogic_JedecHwReset)
    {
        flexspi_nor_jedec_hw_reset(instance);
    }

    // Restore to default state
    flexspi_nor_write_persistent(0);
}

void flexspi_sw_delay_us(uint64_t us)
{
    uint32_t ticks_per_us = CLOCK_GetCoreSysClkFreq() / 1000000;
    while (us--)
    {
        register uint32_t ticks = 1 + ticks_per_us / 4;
        while (ticks--)
        {
            __NOP();
        }
    }
}

//!@brief Write FlexSPI persistent content
status_t flexspi_nor_write_persistent(const uint32_t data)
{
    SRC->GPR[2] = data;

    return kStatus_Success;
}
//!@brief Read FlexSPI persistent content
status_t flexspi_nor_read_persistent(uint32_t *data)
{
    *data = SRC->GPR[2];

    return kStatus_Success;
}

//!@brief Get maximum frequency supported by FlexSPI
status_t flexspi_get_max_supported_freq(uint32_t instance, uint32_t *freq, uint32_t clkMode)
{
    status_t status = kStatus_InvalidArgument;
    do
    {
        if (freq == NULL)
        {
            break;
        }

        *freq = (166UL * 1000 * 1000);
        status = kStatus_Success;

    } while (0);

    return status;
}

//!@brief Set FlexSPI failsafe setting
status_t flexspi_set_failsafe_setting(flexspi_mem_config_t *config)
{
    (void)config;

    return kStatus_Success;
}

// Update Clock Source
status_t flexspi_update_clock_source(uint32_t instance, uint32_t source)
{
    status_t status = kStatus_InvalidArgument;

    do
    {
        if ((instance < kFlexspiInstance_1) || (instance > kFlexspiInstance_2) || (source > kFlexSpiClkSrc_PLL_480))
        {
            break;
        }

        volatile uint32_t *clkRootRegister = NULL;
        if (instance == kFlexspiInstance_1)
        {
            clkRootRegister = &CCM->CLOCK_ROOT[kCLOCK_Root_Flexspi1].CONTROL;
        }
        else if (instance == kFlexspiInstance_2)
        {
            clkRootRegister = &CCM->CLOCK_ROOT[kCLOCK_Root_Flexspi2].CONTROL;
        }

        uint32_t expectedRegValue =
            ((*clkRootRegister) & ~CCM_CLOCK_ROOT_CONTROL_MUX_MASK) | CCM_CLOCK_ROOT_CONTROL_MUX(source);
        FLEXSPI_Type *g_flexspiInstances[] = FLEXSPI_BASE_PTRS;
        if (expectedRegValue != *clkRootRegister)
        {
            // Disable FlexSPI before clock switch
            bool needRestoreClock = false;
            if (is_flexspi_clock_enabled(instance))
            {
                g_flexspiInstances[instance]->MCR0 |= FLEXSPI_MCR0_MDIS_MASK;
                flexspi_clock_gate_disable(instance);

                needRestoreClock = true;
            }
            *clkRootRegister = expectedRegValue;

            if (needRestoreClock)
            {
                // Restore FlexSPI after clock switch
                flexspi_clock_gate_enable(instance);
                g_flexspiInstances[instance]->MCR0 &= ~FLEXSPI_MCR0_MDIS_MASK;
            }
        }

        //debug_printf("%s, addr=%x, value=%x\n", __func__, (uint32_t)clkRootRegister, *clkRootRegister);

        status = kStatus_Success;

    } while (0);

    return status;
}

//!@brief Configure clock for FlexSPI peripheral
void flexspi_clock_config(uint32_t instance, uint32_t freq, uint32_t sampleClkMode)
{
    //debug_printf("%s, instance=%d, freq=%d, sampleClkMode=%d\n", __func__, instance, freq, sampleClkMode);

#if !defined(BL_TARGET_FPGA)
    /*
    According to The clock root definition in RT1170, The PFD clock frequency is fixed.
    To support 30MHz, 50MHz, 60MHz, 80MHz, 100MHz, 120MHz, 133MHz, 166MHz and 200MHz,
    the typical clock configurations is as below:
    PLL528_PFD2 = 396MHz
    PLL480 = 480MHz
    PLL480PFD0 = 664MHz
    RC400M = 400MHz
    PLL528 = 528MHz

    FLEXSPI   30MHz	    50MHz   	   60MHz   	80MHz    100MHz   	    120MHz   133MHz  	166MHz   	 200MHz
    SDR	      PLL480/16 PLL528_PFD2/8  PLL480/6	PLL480/6 PLL528_PFD2/4	PLL480/4 PLL528/4   PLL480PFD0/4 PLL528_PFD2/2
    DDR   	  PLL480/8	PLL528_PFD2/4  PLL480/3	PLL480/3 PLL528_PFD2/2	PLL480/2 PLL528/2	PLL480PFD0/2 PLL528_PFD2/1
    */

    do
    {
        if ((sampleClkMode > kFlexSpiClk_DDR) || (freq < 1))
        {
            break;
        }

        if ((instance != kFlexspiInstance_1) && (instance != kFlexspiInstance_2))
        {
            break;
        }

        volatile uint32_t *clkRootRegister = NULL;
        if (instance == kFlexspiInstance_1)
        {
            clkRootRegister = &CCM->CLOCK_ROOT[kCLOCK_Root_Flexspi1].CONTROL;
        }
        else if (instance == kFlexspiInstance_2)
        {
            clkRootRegister = &CCM->CLOCK_ROOT[kCLOCK_Root_Flexspi2].CONTROL;
        }

        uint32_t freqSrc = ((*clkRootRegister) & CCM_CLOCK_ROOT_CONTROL_MUX_MASK) >> CCM_CLOCK_ROOT_CONTROL_MUX_SHIFT;
        //debug_printf("%s, addr=%x, value=%x\n", __func__, (uint32_t)clkRootRegister, *clkRootRegister);
        typedef struct
        {
            uint8_t src;
            uint8_t div;
        } flexspi_clk_freq_list_t;
        // Formula: Clock(src) / div
        const flexspi_clk_freq_list_t k_flexspi_clk_list_sdr[2][kFlexSpiSerialClk_200MHz + 1] = {
            {
                { 0, 0 },                           // Reserved
                { kFlexSpiClkSrc_PLL_480, 16 },     // 480MHz / 16       30MHz
                { kFlexSpiClkSrc_PLL_528_PFD2, 8 }, // 396MHz / 8    49.5MHz
                { kFlexSpiClkSrc_PLL_528, 8 },      // 528MHz / 6         66MHz
                { kFlexSpiClkSrc_PLL_480, 6 },      // 480MHz / 6         80MHz
                { kFlexSpiClkSrc_PLL_528_PFD2, 4 }, // 396MHz / 4    99MHz
                { kFlexSpiClkSrc_PLL_480, 4 },      // 480MHz / 4         120MHz
                { kFlexSpiClkSrc_PLL_528, 4 },      // 528MHz / 4         132MHz
                { kFlexSpiClkSrc_PLL_480_PFD0, 4 }, // 666MHz / 4    166MHz
                { kFlexSpiClkSrc_PLL_528_PFD2, 2 }  // 396MHz / 2    198MHz
            },
            {
                { 0, 0 },                      // Reserved
                { kFlexSpiClkSrc_RC400M, 13 }, // 400MHz / 13       30MHz
                { kFlexSpiClkSrc_RC400M, 8 },  // 400MHz / 8         50MHz
                { kFlexSpiClkSrc_RC400M, 6 },  // 400MHz / 6         66MHz
                { kFlexSpiClkSrc_RC400M, 5 },  // 400MHz / 5         80MHz
                { kFlexSpiClkSrc_RC400M, 4 },  // 400MHz / 4         100MHz
                { kFlexSpiClkSrc_RC400M, 3 },  // 400MHz / 3         133MHz
                { kFlexSpiClkSrc_RC400M, 3 },  // 400MHz / 3         133MHz
                { kFlexSpiClkSrc_RC400M, 3 },  // 400MHz / 3         133MHz
                { kFlexSpiClkSrc_RC400M, 2 }   // 400MHz / 2         200MHz
            }
        };
        // Formula: Clock(src) / div / 2
        const flexspi_clk_freq_list_t k_flexspi_clk_list_ddr[2][kFlexSpiSerialClk_200MHz + 1] = {
            {
                { 0, 0 },                           // Reserved
                { kFlexSpiClkSrc_PLL_480, 8 },      // 480MHz / 8 / 2        30MHz
                { kFlexSpiClkSrc_PLL_528_PFD2, 4 }, // 396MHz / 4 / 2   49.5MHz
                { kFlexSpiClkSrc_PLL_528, 4 },      // 528MHz / 4 / 2        66MHz
                { kFlexSpiClkSrc_PLL_480, 3 },      // 480MHz / 3 / 2        80MHz
                { kFlexSpiClkSrc_PLL_528_PFD2, 2 }, // 396MHz / 2 / 2   99MHz
                { kFlexSpiClkSrc_PLL_480, 2 },      // 480MHz / 2 / 2        120MHz
                { kFlexSpiClkSrc_PLL_528, 2 },      // 528MHz / 2 / 2        132MHz
                { kFlexSpiClkSrc_PLL_480_PFD0, 2 }, // 666MHz / 2 / 2   166MHz
                { kFlexSpiClkSrc_PLL_528_PFD2, 1 }  // 396MHz / 2       198MHz
            },
            {
                { 0, 0 },                     // Reserved
                { kFlexSpiClkSrc_RC400M, 6 }, // 400MHz / 6 / 2     33MHz
                { kFlexSpiClkSrc_RC400M, 4 }, // 400MHz / 4 / 2     50MHz
                { kFlexSpiClkSrc_RC400M, 3 }, // 400MHz / 3 / 2     66MHz
                { kFlexSpiClkSrc_RC400M, 3 }, // 400MHz / 3 / 2     66MHz
                { kFlexSpiClkSrc_RC400M, 2 }, // 400MHz / 2 / 2     100MHz
                { kFlexSpiClkSrc_RC400M, 2 }, // 400MHz / 2 / 2     100MHz
                { kFlexSpiClkSrc_RC400M, 2 }, // 400MHz / 2 / 2     100MHz
                { kFlexSpiClkSrc_RC400M, 2 }, // 400MHz / 2 / 2     100MHz
                { kFlexSpiClkSrc_RC400M, 1 }  // 400MHz / 1 / 2     200MHz
            }
        };

        bool isHighSpeedSupported = false;
        uint32_t clkDiv = 1;
        uint32_t clkSrc = 0;
        switch (freqSrc)
        {
            default:
                isHighSpeedSupported = true;
                freqSrc = 0;
                break;
            case kFlexSpiClkSrc_RC400M:
                isHighSpeedSupported = true;
                freqSrc = 1;
                break;
            case kFlexSpiClkSrc_RC48M_Div2:
                clkDiv = 1;
                clkSrc = kFlexSpiClkSrc_RC48M_Div2;
                break;
            case kFlexSpiClkSrc_OSC:
                clkDiv = 1;
                clkSrc = kFlexSpiClkSrc_OSC;
                break;
            case kFlexSpiClkSrc_RC4M_16M:
                clkDiv = 1;
                clkSrc = kFlexSpiClkSrc_RC4M_16M;
                break;
        }

        if (isHighSpeedSupported)
        {
            const flexspi_clk_freq_list_t *freq_list = k_flexspi_clk_list_sdr[freqSrc];
            if (sampleClkMode == kFlexSpiClk_DDR)
            {
                freq_list = k_flexspi_clk_list_ddr[freqSrc];
            }

            if (freq > kFlexSpiSerialClk_200MHz)
            {
                freq = kFlexSpiSerialClk_SafeFreq;
            }
            clkDiv = freq_list[freq].div;
            clkSrc = freq_list[freq].src;
        }

        uint32_t tmpClkRootRegValue = CCM_CLOCK_ROOT_CONTROL_DIV(clkDiv - 1) | CCM_CLOCK_ROOT_CONTROL_MUX(clkSrc);

        FLEXSPI_Type *g_flexspiInstances[] = FLEXSPI_BASE_PTRS;

        // Disable FlexSPI before clock switch
        if (is_flexspi_clock_enabled(instance))
        {
            g_flexspiInstances[instance]->MCR0 |= FLEXSPI_MCR0_MDIS_MASK;
            flexspi_clock_gate_disable(instance);
        }

        // Update FLEXSPI clock root setting as needed
        if (tmpClkRootRegValue != *clkRootRegister)
        {
            *clkRootRegister = tmpClkRootRegValue;
        }

        //debug_printf("%s, address=%x, value=%x\n", __func__, (uint32_t)clkRootRegister, *clkRootRegister);

        // Restore FlexSPI after clock switch
        flexspi_clock_gate_enable(instance);
        g_flexspiInstances[instance]->MCR0 &= ~FLEXSPI_MCR0_MDIS_MASK;

        uint32_t serialRootClk = 0;

        flexspi_get_clock(instance, kFlexSpiClock_SerialRootClock, &serialRootClk);
        //debug_printf("BootROM: FlexSPI Clock = %d\n", serialRootClk);

    } while (0);
#endif
}

//!@brief Gate on the clock for the FlexSPI peripheral
void flexspi_clock_gate_enable(uint32_t instance)
{
    volatile uint32_t *reg = NULL;
    if (instance == kFlexspiInstance_1)
    {
        reg = &CCM->LPCG[kCLOCK_Flexspi1].DIRECT;
    }
    else if (instance == kFlexspiInstance_2)
    {
        reg = &CCM->LPCG[kCLOCK_Flexspi2].DIRECT;
    }

    if (reg != NULL)
    {
        *reg = 1;

        //debug_printf("%s, address=%x, value=%x\n", __func__, (uint32_t)reg, *reg);
        __DSB();
        __ISB();
        uint32_t retryCnt = 10;
        while ((!(*reg & 1)) && retryCnt)
        {
            retryCnt--;
        }
    }
}

//!@brief Gate off the clock the FlexSPI peripheral
void flexspi_clock_gate_disable(uint32_t instance)
{
    volatile uint32_t *reg = NULL;
    if (instance == kFlexspiInstance_1)
    {
        reg = &CCM->LPCG[kCLOCK_Flexspi1].DIRECT;
    }
    else if (instance == kFlexspiInstance_2)
    {
        reg = &CCM->LPCG[kCLOCK_Flexspi2].DIRECT;
    }

    if (reg != NULL)
    {
        *reg = 0;

        //debug_printf("%s, address=%x, value=%x\n", __func__, (uint32_t)reg, *reg);
        __DSB();
        __ISB();
        uint32_t retryCnt = 10;
        while ((*reg & 1) && retryCnt)
        {
            retryCnt--;
        }
    }
}

bool is_flexspi_clock_enabled(uint32_t instance)
{
    volatile uint32_t *reg = NULL;
    bool isEnabled = false;
    if (instance == kFlexspiInstance_1)
    {
        reg = &CCM->LPCG[kCLOCK_Flexspi1].DIRECT;
    }
    else if (instance == kFlexspiInstance_2)
    {
        reg = &CCM->LPCG[kCLOCK_Flexspi2].DIRECT;
    }
    if (reg != NULL)
    {
        isEnabled = *reg ? true : false;
    }

    return isEnabled;
}

void flexspi_update_padsetting(uint32_t instance, flexspi_mem_config_t *config, uint32_t driveStrength)
{
    if (driveStrength < 1)
    {
        return;
    }

    if (instance == kFlexspiInstance_1)
    {
        config->csPadSettingOverride = driveStrength;
        config->sclkPadSettingOverride = driveStrength;
        config->dataPadSettingOverride = driveStrength;

        if (config->controllerMiscOption & FLEXSPI_BITMASK(kFlexSpiMiscOffset_SecondPinMux))
        {
            uint32_t highDriveEnable = (driveStrength & IOMUXC_SW_PAD_CTL_PAD_DSE_MASK) ? 1 : 0;

            // DQS pad must have pull-down resistor enabled
            config->dqsPadSettingOverride = IOMUXC_SW_PAD_CTL_PAD_DSE(highDriveEnable) | IOMUXC_SW_PAD_CTL_PAD_PUE(1);
        }
        else
        {
            uint32_t normalDriveEnable = (driveStrength & IOMUXC_SW_PAD_CTL_PAD_PDRV_MASK) ? 1 : 0;
            // DQS pad must have pull-down resistor enabled
            config->dqsPadSettingOverride =
                IOMUXC_SW_PAD_CTL_PAD_PDRV(normalDriveEnable) | IOMUXC_SW_PAD_CTL_PAD_PULL(2);
        }

        if (config->controllerMiscOption & FLEXSPI_BITMASK(kFlexSpiMiscOffset_SecondDqsPinMux))
        {
            uint32_t normalDriveEnable = (driveStrength & IOMUXC_SW_PAD_CTL_PAD_PDRV_MASK) ? 1 : 0;
            // DQS pad must have pull-down resistor enabled
            config->dqsPadSettingOverride =
                IOMUXC_SW_PAD_CTL_PAD_PDRV(normalDriveEnable) | IOMUXC_SW_PAD_CTL_PAD_PULL(2);
        }
    }
    else if (instance == kFlexspiInstance_2)
    {
        config->csPadSettingOverride = driveStrength;
        config->sclkPadSettingOverride = driveStrength;
        config->dataPadSettingOverride = driveStrength;

        uint32_t normalDriveEnable = (driveStrength & IOMUXC_SW_PAD_CTL_PAD_PDRV_MASK) ? 1 : 0;
        // DQS pad must have pull-down resistor enabled
        config->dqsPadSettingOverride = IOMUXC_SW_PAD_CTL_PAD_PDRV(normalDriveEnable) | IOMUXC_SW_PAD_CTL_PAD_PULL(2);
    }
    else
    {
        // Do nothing here
    }
}

uint32_t get_flexspinor_instance(void)
{
    uint32_t instance;
    if (FUSE_SIP_PKG_EN_VALUE > 0)
    {
        instance = kFlexspiInstance_2;
    }
    else
    {
        instance = 1 + FUSE_FLEXSPI_INSTANCE_VALUE;
    }
    return instance;
}

uint32_t get_flash_type(void)
{
    // Read Flash type from Shadow register
    uint32_t flash_type;

    if (FUSE_SIP_PKG_EN_VALUE > 0)
    {
        flash_type = 0;
    }
    else
    {
        flash_type = FUSE_FLASH_TYPE_VALUE;
        if (flash_type > 7)
        {
            flash_type = 0;
        }
    }

    return flash_type;
}

uint32_t get_flexspinand_instance(void)
{
    uint32_t instance = 1 + FUSE_FLEXSPI_INSTANCE_VALUE;
    return instance;
}

uint32_t flexspinor_get_phy_addr(uint32_t offset)
{
    uint32_t instance = get_flexspinor_instance();
    uint32_t baseAmbaAddr = (instance == kFlexspiInstance_1) ? FlexSPI1_AMBA_BASE : FlexSPI2_AMBA_BASE;
    return (baseAmbaAddr + offset);
}

status_t flexspi_ram_ahb_buffer_config(uint32_t instance, flexspi_mem_config_t *memCfg)
{
    status_t status = kStatus_Fail;

    FLEXSPI_Type *g_flexSpiInstances[] = FLEXSPI_BASE_PTRS;

    do
    {
        FLEXSPI_Type *base = NULL;

        if (instance < sizeof(g_flexSpiInstances) / sizeof(g_flexSpiInstances[0]))
        {
            base = g_flexSpiInstances[instance];
        }
        if (base == NULL)
        {
            break;
        }

        base->MCR0 |= FLEXSPI_MCR0_MDIS_MASK;

        // Note: All the existing Memory device has the maximum 4us Chip-select signal low period, converting to the
        //       buffer size, see the below calculation
        //      Here assumes that the RAM device works at OPI_DDR mode;
        uint32_t maxClkCyclesInUs = 30;
        uint32_t flexspiSerialRootClk = 0;
        status = flexspi_get_clock(instance, kFlexSpiClock_SerialRootClock, &flexspiSerialRootClk);
        if (status != kStatus_Success)
        {
            break;
        }
        maxClkCyclesInUs = flexspiSerialRootClk / 2 / FREQ_1MHz;
        // Through the max Chip-select low interval is 4us, here, ROM reserves 0.1us buffer
        uint32_t maxClkCyclesDuringCsLow = maxClkCyclesInUs * 39 / 10;

        /* Because each Read sequence consists of (CS_SETUP+CS_HOLD+READ_CMD_ADDR+DUMMY+DATA)
          CS_SETUP = 3 cycles
          CS_HOLD  = 3 cycles
          READ_CMD_ADDR = up to 6 cycles (HyperRAM)
          DUMMY = up to 7 cycles (APMemory)
          DATA =  maxClkCyclesDuringCsLow - CS_SETUP - CS_HOLD - READ_CMD_ADDR - DUMMY
          The FLEXSPI reads data from RAM via 8-bit wide data pad using DDR mode, so:
          maxBufferSize = 2 bytes * (maxClkCyclesDuringCsLow - CS_SETUP - CS_HOLD - READ_CMD_ADDR - DUMMY);
        */
        uint32_t maxBufferSize = 2 * (maxClkCyclesDuringCsLow - 3 - 3 - 6 - 7);

        /* By default, all the masters share buffer0 .
        ** Others - All other masters share the remaining AHB buffer spaces
        */
        if (maxBufferSize > (FLEXSPI_AHB_RX_BUFFER_SIZE / 2))
        {
            maxBufferSize = (FLEXSPI_AHB_RX_BUFFER_SIZE / 2);
        }
        //debug_printf("%s: maxBufferSize=%d\n", __func__, maxBufferSize);

        for (uint32_t index = 0; index < ARRAY_SIZE(base->AHBRXBUFCR0) - 1; index++)
        {
            base->AHBRXBUFCR0[index] &=
                ~(FLEXSPI_AHBRXBUFCR0_BUFSZ_MASK | FLEXSPI_AHBRXBUFCR0_MSTRID_MASK | FLEXSPI_AHBRXBUFCR0_PRIORITY_MASK);
        }

        base->AHBRXBUFCR0[0] = FLEXSPI_AHBRXBUFCR0_BUFSZ(maxBufferSize / 8) | FLEXSPI_AHBRXBUFCR0_MSTRID(0) |
                               FLEXSPI_AHBRXBUFCR0_PREFETCHEN(1);
        // The other masters share the remaining AHB buffer spaces

        // Initialize the LUT with known value
        base->MCR0 &= ~FLEXSPI_MCR0_MDIS_MASK;

        flexspi_clear_cache(instance);

        for (uint32_t delayCnt = 200; delayCnt > 0; delayCnt--)
        {
            __NOP();
        }

        status = kStatus_Success;

    } while (0);

    return status;
}
