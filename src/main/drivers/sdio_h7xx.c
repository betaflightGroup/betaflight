/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * Original author: Alain (https://github.com/aroyer-qc)
 * Modified for BF source: Chris Hockuba (https://github.com/conkerkh)
 */

/* Include(s) -------------------------------------------------------------------------------------------------------*/

#include "stdbool.h"
#include <string.h>

#include "platform.h"

#ifdef USE_SDCARD_SDIO

#include "sdmmc_sdio.h"

#include "pg/sdio.h"

#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/nvic.h"
#include "drivers/sdio.h"

typedef struct SD_Handle_s
{
    uint32_t          CSD[4];           // SD card specific data table
    uint32_t          CID[4];           // SD card identification number table
    volatile uint32_t RXCplt;          // SD RX Complete is equal 0 when no transfer
    volatile uint32_t TXCplt;          // SD TX Complete is equal 0 when no transfer
} SD_Handle_t;

SD_HandleTypeDef hsd1;

SD_CardInfo_t                      SD_CardInfo;
SD_CardType_t                      SD_CardType;

static SD_Handle_t                 SD_Handle;

typedef struct sdioPin_s {
    ioTag_t pin;
    uint8_t af;
} sdioPin_t;

#define SDIO_PIN_D0  0
#define SDIO_PIN_D1  1
#define SDIO_PIN_D2  2
#define SDIO_PIN_D3  3
#define SDIO_PIN_CK  4
#define SDIO_PIN_CMD 5
#define SDIO_PIN_COUNT  6

#define SDIO_MAX_PINDEFS 2

typedef struct sdioHardware_s {
    SDMMC_TypeDef *instance;
    IRQn_Type irqn;
    sdioPin_t sdioPinCK[SDIO_MAX_PINDEFS];
    sdioPin_t sdioPinCMD[SDIO_MAX_PINDEFS];
    sdioPin_t sdioPinD0[SDIO_MAX_PINDEFS];
    sdioPin_t sdioPinD1[SDIO_MAX_PINDEFS];
    sdioPin_t sdioPinD2[SDIO_MAX_PINDEFS];
    sdioPin_t sdioPinD3[SDIO_MAX_PINDEFS];
} sdioHardware_t;

// Possible pin assignments

#define PINDEF(device, pin, afnum) { DEFIO_TAG_E(pin), GPIO_AF ## afnum ## _SDMMC ## device }

static const sdioHardware_t sdioPinHardware[SDIODEV_COUNT] = {
    {
        .instance = SDMMC1,
        .irqn = SDMMC1_IRQn,
        .sdioPinCK  = { PINDEF(1, PC12, 12) },
        .sdioPinCMD = { PINDEF(1, PD2,  12) },
        .sdioPinD0  = { PINDEF(1, PC8,  12) },
        .sdioPinD1  = { PINDEF(1, PC9,  12) },
        .sdioPinD2  = { PINDEF(1, PC10, 12) },
        .sdioPinD3  = { PINDEF(1, PC11, 12) },
    },
    {
        .instance = SDMMC2,
        .irqn = SDMMC2_IRQn,
        .sdioPinCK  = { PINDEF(2, PC1,   9), PINDEF(2, PD6, 11) },
        .sdioPinCMD = { PINDEF(2, PA0,   9), PINDEF(2, PD7, 11) },
        .sdioPinD0  = { PINDEF(2, PB14,  9) },
        .sdioPinD1  = { PINDEF(2, PB15,  9) },
        .sdioPinD2  = { PINDEF(2, PB3,   9) },
        .sdioPinD3  = { PINDEF(2, PB4,   9) },
    }
};

#undef PINDEF

// Active configuration
static const sdioHardware_t *sdioHardware;
static sdioPin_t sdioPin[SDIO_PIN_COUNT];

static const sdioPin_t *sdioFindPinDef(const sdioPin_t *pindefs, ioTag_t pin)
{
    for (unsigned index = 0; index < SDIO_MAX_PINDEFS; index++) {
        if (pindefs[index].pin == pin) {
            return &pindefs[index];
        }
    }

    return NULL;
}

#define SDIOFINDPIN(pinname)  { \
        const sdioPin_t *pindef;                                                                     \
        pindef = sdioFindPinDef(sdioHardware->sdioPin ## pinname, sdioPinConfig()->pinname ## Pin);  \
        if (pindef) {                                                                                \
            sdioPin[SDIO_PIN_ ## pinname] = *pindef;                                           \
        }                                                                                            \
    } struct dummy

void sdioPinConfigure(void)
{
    SDIODevice device = SDIO_CFG_TO_DEV(sdioConfig()->device);

    if (device == SDIOINVALID) {
        return;
    }

    sdioHardware = &sdioPinHardware[device];

    SDIOFINDPIN(CK);
    SDIOFINDPIN(CMD);
    SDIOFINDPIN(D0);

    if (sdioConfig()->use4BitWidth) {
        SDIOFINDPIN(D1);
        SDIOFINDPIN(D2);
        SDIOFINDPIN(D3);
    }
}

#undef SDIOFINDPIN

#define IOCFG_SDMMC       IO_CONFIG(GPIO_MODE_AF_PP, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_NOPULL)

void HAL_SD_MspInit(SD_HandleTypeDef* hsd)
{
    UNUSED(hsd);

    if (!sdioHardware) {
        return;
    }

    if (sdioHardware->instance == SDMMC1) {
        __HAL_RCC_SDMMC1_CLK_DISABLE();
        __HAL_RCC_SDMMC1_FORCE_RESET();
        __HAL_RCC_SDMMC1_RELEASE_RESET();
        __HAL_RCC_SDMMC1_CLK_ENABLE();
    } else if (sdioHardware->instance == SDMMC2) {
        __HAL_RCC_SDMMC2_CLK_DISABLE();
        __HAL_RCC_SDMMC2_FORCE_RESET();
        __HAL_RCC_SDMMC2_RELEASE_RESET();
        __HAL_RCC_SDMMC2_CLK_ENABLE();
    }

    uint8_t is4BitWidth = sdioConfig()->use4BitWidth;

    const IO_t clk = IOGetByTag(sdioPin[SDIO_PIN_CK].pin);
    const IO_t cmd = IOGetByTag(sdioPin[SDIO_PIN_CMD].pin);
    const IO_t d0 = IOGetByTag(sdioPin[SDIO_PIN_D0].pin);
    const IO_t d1 = IOGetByTag(sdioPin[SDIO_PIN_D1].pin);
    const IO_t d2 = IOGetByTag(sdioPin[SDIO_PIN_D2].pin);
    const IO_t d3 = IOGetByTag(sdioPin[SDIO_PIN_D3].pin);

    IOConfigGPIOAF(clk, IOCFG_SDMMC, sdioPin[SDIO_PIN_CK].af);
    IOConfigGPIOAF(cmd, IOCFG_SDMMC, sdioPin[SDIO_PIN_CMD].af);
    IOConfigGPIOAF(d0, IOCFG_SDMMC, sdioPin[SDIO_PIN_D0].af);

    if(is4BitWidth) {
        IOConfigGPIOAF(d1, IOCFG_SDMMC, sdioPin[SDIO_PIN_D1].af);
        IOConfigGPIOAF(d2, IOCFG_SDMMC, sdioPin[SDIO_PIN_D2].af);
        IOConfigGPIOAF(d3, IOCFG_SDMMC, sdioPin[SDIO_PIN_D3].af);
    }

    HAL_NVIC_SetPriority(sdioHardware->irqn, NVIC_PRIORITY_BASE(NVIC_PRIO_SDIO_DMA), NVIC_PRIORITY_SUB(NVIC_PRIO_SDIO_DMA));
    HAL_NVIC_EnableIRQ(sdioHardware->irqn);
}

void SDIO_GPIO_Init(void)
{
    if (!sdioHardware) {
        return;
    }

    uint8_t is4BitWidth = sdioConfig()->use4BitWidth;

    const IO_t clk = IOGetByTag(sdioPin[SDIO_PIN_CK].pin);
    const IO_t cmd = IOGetByTag(sdioPin[SDIO_PIN_CMD].pin);
    const IO_t d0 = IOGetByTag(sdioPin[SDIO_PIN_D0].pin);
    const IO_t d1 = IOGetByTag(sdioPin[SDIO_PIN_D1].pin);
    const IO_t d2 = IOGetByTag(sdioPin[SDIO_PIN_D2].pin);
    const IO_t d3 = IOGetByTag(sdioPin[SDIO_PIN_D3].pin);

    IOInit(clk, OWNER_SDIO_CK, 0);
    IOInit(cmd, OWNER_SDIO_CMD, 0);
    IOInit(d0, OWNER_SDIO_D0, 0);

    if (is4BitWidth) {
        IOInit(d1, OWNER_SDIO_D1, 0);
        IOInit(d2, OWNER_SDIO_D2, 0);
        IOInit(d3, OWNER_SDIO_D3, 0);
    }

    //
    // Setting all the SDIO pins to high for a short time results in more robust initialisation.
    //
    IOHi(d0);
    IOConfigGPIO(d0, IOCFG_OUT_PP);

    if(is4BitWidth) {
        IOHi(d1);
        IOHi(d2);
        IOHi(d3);
        IOConfigGPIO(d1, IOCFG_OUT_PP);
        IOConfigGPIO(d2, IOCFG_OUT_PP);
        IOConfigGPIO(d3, IOCFG_OUT_PP);
    }

    IOHi(clk);
    IOHi(cmd);
    IOConfigGPIO(clk, IOCFG_OUT_PP);
    IOConfigGPIO(cmd, IOCFG_OUT_PP);
}

void SD_Initialize_LL(DMA_Stream_TypeDef *dma)
{
    UNUSED(dma);
}

bool SD_GetState(void)
{
    HAL_SD_CardStateTypedef cardState = HAL_SD_GetCardState(&hsd1);

    return (cardState == HAL_SD_CARD_TRANSFER);
}

SD_Error_t SD_Init(void)
{
    HAL_StatusTypeDef status;

    memset(&hsd1, 0, sizeof(hsd1));

    hsd1.Instance = sdioHardware->instance;

    hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
    hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_ENABLE;
    if (sdioConfig()->use4BitWidth) {
        hsd1.Init.BusWide = SDMMC_BUS_WIDE_4B;
    } else {
        hsd1.Init.BusWide = SDMMC_BUS_WIDE_1B; // FIXME untested
    }
    hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_ENABLE;
    hsd1.Init.ClockDiv = 1; // 200Mhz / (2 * 1 ) = 100Mhz, used for "UltraHigh speed SD card" only, see   HAL_SD_ConfigWideBusOperation, SDMMC_HSpeed_CLK_DIV, SDMMC_NSpeed_CLK_DIV

    status = HAL_SD_Init(&hsd1); // Will call HAL_SD_MspInit

    if (status != HAL_OK) {
        return SD_ERROR;
    }

    switch(hsd1.SdCard.CardType) {
    case CARD_SDSC:
        switch (hsd1.SdCard.CardVersion) {
        case CARD_V1_X:
            SD_CardType = SD_STD_CAPACITY_V1_1;
            break;
        case CARD_V2_X:
            SD_CardType = SD_STD_CAPACITY_V2_0;
            break;
        default:
            return SD_ERROR;
        }
        break;

    case CARD_SDHC_SDXC:
        SD_CardType = SD_HIGH_CAPACITY;
        break;

    default:
        return SD_ERROR;
    }

    STATIC_ASSERT(sizeof(SD_Handle.CSD) == sizeof(hsd1.CSD), hal-csd-size-error);
    memcpy(&SD_Handle.CSD, &hsd1.CSD, sizeof(SD_Handle.CSD));

    STATIC_ASSERT(sizeof(SD_Handle.CID) == sizeof(hsd1.CID), hal-cid-size-error);
    memcpy(&SD_Handle.CID, &hsd1.CID, sizeof(SD_Handle.CID));

    return SD_OK;
}

SD_Error_t SD_GetCardInfo(void)
{
    SD_Error_t ErrorState = SD_OK;

    // fill in SD_CardInfo

    uint32_t Temp = 0;

    // Byte 0
    Temp = (SD_Handle.CSD[0] & 0xFF000000) >> 24;
    SD_CardInfo.SD_csd.CSDStruct      = (uint8_t)((Temp & 0xC0) >> 6);
    SD_CardInfo.SD_csd.SysSpecVersion = (uint8_t)((Temp & 0x3C) >> 2);
    SD_CardInfo.SD_csd.Reserved1      = Temp & 0x03;

    // Byte 1
    Temp = (SD_Handle.CSD[0] & 0x00FF0000) >> 16;
    SD_CardInfo.SD_csd.TAAC = (uint8_t)Temp;

    // Byte 2
    Temp = (SD_Handle.CSD[0] & 0x0000FF00) >> 8;
    SD_CardInfo.SD_csd.NSAC = (uint8_t)Temp;

    // Byte 3
    Temp = SD_Handle.CSD[0] & 0x000000FF;
    SD_CardInfo.SD_csd.MaxBusClkFrec = (uint8_t)Temp;

    // Byte 4
    Temp = (SD_Handle.CSD[1] & 0xFF000000) >> 24;
    SD_CardInfo.SD_csd.CardComdClasses = (uint16_t)(Temp << 4);

    // Byte 5
    Temp = (SD_Handle.CSD[1] & 0x00FF0000) >> 16;
    SD_CardInfo.SD_csd.CardComdClasses |= (uint16_t)((Temp & 0xF0) >> 4);
    SD_CardInfo.SD_csd.RdBlockLen       = (uint8_t)(Temp & 0x0F);

    // Byte 6
    Temp = (SD_Handle.CSD[1] & 0x0000FF00) >> 8;
    SD_CardInfo.SD_csd.PartBlockRead   = (uint8_t)((Temp & 0x80) >> 7);
    SD_CardInfo.SD_csd.WrBlockMisalign = (uint8_t)((Temp & 0x40) >> 6);
    SD_CardInfo.SD_csd.RdBlockMisalign = (uint8_t)((Temp & 0x20) >> 5);
    SD_CardInfo.SD_csd.DSRImpl         = (uint8_t)((Temp & 0x10) >> 4);
    SD_CardInfo.SD_csd.Reserved2       = 0; /*!< Reserved */

    if((SD_CardType == SD_STD_CAPACITY_V1_1) || (SD_CardType == SD_STD_CAPACITY_V2_0)) {
        SD_CardInfo.SD_csd.DeviceSize = (Temp & 0x03) << 10;

        // Byte 7
        Temp = (uint8_t)(SD_Handle.CSD[1] & 0x000000FF);
        SD_CardInfo.SD_csd.DeviceSize |= (Temp) << 2;

        // Byte 8
        Temp = (uint8_t)((SD_Handle.CSD[2] & 0xFF000000) >> 24);
        SD_CardInfo.SD_csd.DeviceSize |= (Temp & 0xC0) >> 6;

        SD_CardInfo.SD_csd.MaxRdCurrentVDDMin = (Temp & 0x38) >> 3;
        SD_CardInfo.SD_csd.MaxRdCurrentVDDMax = (Temp & 0x07);

        // Byte 9
        Temp = (uint8_t)((SD_Handle.CSD[2] & 0x00FF0000) >> 16);
        SD_CardInfo.SD_csd.MaxWrCurrentVDDMin = (Temp & 0xE0) >> 5;
        SD_CardInfo.SD_csd.MaxWrCurrentVDDMax = (Temp & 0x1C) >> 2;
        SD_CardInfo.SD_csd.DeviceSizeMul      = (Temp & 0x03) << 1;

        // Byte 10
        Temp = (uint8_t)((SD_Handle.CSD[2] & 0x0000FF00) >> 8);
        SD_CardInfo.SD_csd.DeviceSizeMul |= (Temp & 0x80) >> 7;

        SD_CardInfo.CardCapacity  = (SD_CardInfo.SD_csd.DeviceSize + 1) ;
        SD_CardInfo.CardCapacity *= (1 << (SD_CardInfo.SD_csd.DeviceSizeMul + 2));
        SD_CardInfo.CardBlockSize = 1 << (SD_CardInfo.SD_csd.RdBlockLen);
        SD_CardInfo.CardCapacity = SD_CardInfo.CardCapacity * SD_CardInfo.CardBlockSize / 512; // In 512 byte blocks
    } else if(SD_CardType == SD_HIGH_CAPACITY) {
        // Byte 7
        Temp = (uint8_t)(SD_Handle.CSD[1] & 0x000000FF);
        SD_CardInfo.SD_csd.DeviceSize = (Temp & 0x3F) << 16;

        // Byte 8
        Temp = (uint8_t)((SD_Handle.CSD[2] & 0xFF000000) >> 24);

        SD_CardInfo.SD_csd.DeviceSize |= (Temp << 8);

        // Byte 9
        Temp = (uint8_t)((SD_Handle.CSD[2] & 0x00FF0000) >> 16);

        SD_CardInfo.SD_csd.DeviceSize |= (Temp);

        // Byte 10
        Temp = (uint8_t)((SD_Handle.CSD[2] & 0x0000FF00) >> 8);

        SD_CardInfo.CardCapacity  = ((uint64_t)SD_CardInfo.SD_csd.DeviceSize + 1) * 1024;
        SD_CardInfo.CardBlockSize = 512;
    } else {
        // Not supported card type
        ErrorState = SD_ERROR;
    }

    SD_CardInfo.SD_csd.EraseGrSize = (Temp & 0x40) >> 6;
    SD_CardInfo.SD_csd.EraseGrMul  = (Temp & 0x3F) << 1;

    // Byte 11
    Temp = (uint8_t)(SD_Handle.CSD[2] & 0x000000FF);
    SD_CardInfo.SD_csd.EraseGrMul     |= (Temp & 0x80) >> 7;
    SD_CardInfo.SD_csd.WrProtectGrSize = (Temp & 0x7F);

    // Byte 12
    Temp = (uint8_t)((SD_Handle.CSD[3] & 0xFF000000) >> 24);
    SD_CardInfo.SD_csd.WrProtectGrEnable = (Temp & 0x80) >> 7;
    SD_CardInfo.SD_csd.ManDeflECC        = (Temp & 0x60) >> 5;
    SD_CardInfo.SD_csd.WrSpeedFact       = (Temp & 0x1C) >> 2;
    SD_CardInfo.SD_csd.MaxWrBlockLen     = (Temp & 0x03) << 2;

    // Byte 13
    Temp = (uint8_t)((SD_Handle.CSD[3] & 0x00FF0000) >> 16);
    SD_CardInfo.SD_csd.MaxWrBlockLen      |= (Temp & 0xC0) >> 6;
    SD_CardInfo.SD_csd.WriteBlockPaPartial = (Temp & 0x20) >> 5;
    SD_CardInfo.SD_csd.Reserved3           = 0;
    SD_CardInfo.SD_csd.ContentProtectAppli = (Temp & 0x01);

    // Byte 14
    Temp = (uint8_t)((SD_Handle.CSD[3] & 0x0000FF00) >> 8);
    SD_CardInfo.SD_csd.FileFormatGrouop = (Temp & 0x80) >> 7;
    SD_CardInfo.SD_csd.CopyFlag         = (Temp & 0x40) >> 6;
    SD_CardInfo.SD_csd.PermWrProtect    = (Temp & 0x20) >> 5;
    SD_CardInfo.SD_csd.TempWrProtect    = (Temp & 0x10) >> 4;
    SD_CardInfo.SD_csd.FileFormat       = (Temp & 0x0C) >> 2;
    SD_CardInfo.SD_csd.ECC              = (Temp & 0x03);

    // Byte 15
    Temp = (uint8_t)(SD_Handle.CSD[3] & 0x000000FF);
    SD_CardInfo.SD_csd.CSD_CRC   = (Temp & 0xFE) >> 1;
    SD_CardInfo.SD_csd.Reserved4 = 1;

    // Byte 0
    Temp = (uint8_t)((SD_Handle.CID[0] & 0xFF000000) >> 24);
    SD_CardInfo.SD_cid.ManufacturerID = Temp;

    // Byte 1
    Temp = (uint8_t)((SD_Handle.CID[0] & 0x00FF0000) >> 16);
    SD_CardInfo.SD_cid.OEM_AppliID = Temp << 8;

    // Byte 2
    Temp = (uint8_t)((SD_Handle.CID[0] & 0x000000FF00) >> 8);
    SD_CardInfo.SD_cid.OEM_AppliID |= Temp;

    // Byte 3
    Temp = (uint8_t)(SD_Handle.CID[0] & 0x000000FF);
    SD_CardInfo.SD_cid.ProdName1 = Temp << 24;

    // Byte 4
    Temp = (uint8_t)((SD_Handle.CID[1] & 0xFF000000) >> 24);
    SD_CardInfo.SD_cid.ProdName1 |= Temp << 16;

    // Byte 5
    Temp = (uint8_t)((SD_Handle.CID[1] & 0x00FF0000) >> 16);
    SD_CardInfo.SD_cid.ProdName1 |= Temp << 8;

    // Byte 6
    Temp = (uint8_t)((SD_Handle.CID[1] & 0x0000FF00) >> 8);
    SD_CardInfo.SD_cid.ProdName1 |= Temp;

    // Byte 7
    Temp = (uint8_t)(SD_Handle.CID[1] & 0x000000FF);
    SD_CardInfo.SD_cid.ProdName2 = Temp;

    // Byte 8
    Temp = (uint8_t)((SD_Handle.CID[2] & 0xFF000000) >> 24);
    SD_CardInfo.SD_cid.ProdRev = Temp;

    // Byte 9
    Temp = (uint8_t)((SD_Handle.CID[2] & 0x00FF0000) >> 16);
    SD_CardInfo.SD_cid.ProdSN = Temp << 24;

    // Byte 10
    Temp = (uint8_t)((SD_Handle.CID[2] & 0x0000FF00) >> 8);
    SD_CardInfo.SD_cid.ProdSN |= Temp << 16;

    // Byte 11
    Temp = (uint8_t)(SD_Handle.CID[2] & 0x000000FF);
    SD_CardInfo.SD_cid.ProdSN |= Temp << 8;

    // Byte 12
    Temp = (uint8_t)((SD_Handle.CID[3] & 0xFF000000) >> 24);
    SD_CardInfo.SD_cid.ProdSN |= Temp;

    // Byte 13
    Temp = (uint8_t)((SD_Handle.CID[3] & 0x00FF0000) >> 16);
    SD_CardInfo.SD_cid.Reserved1   |= (Temp & 0xF0) >> 4;
    SD_CardInfo.SD_cid.ManufactDate = (Temp & 0x0F) << 8;

    // Byte 14
    Temp = (uint8_t)((SD_Handle.CID[3] & 0x0000FF00) >> 8);
    SD_CardInfo.SD_cid.ManufactDate |= Temp;

    // Byte 15
    Temp = (uint8_t)(SD_Handle.CID[3] & 0x000000FF);
    SD_CardInfo.SD_cid.CID_CRC   = (Temp & 0xFE) >> 1;
    SD_CardInfo.SD_cid.Reserved2 = 1;

    return ErrorState;
}

SD_Error_t SD_CheckWrite(void) {
    if (SD_Handle.TXCplt != 0) return SD_BUSY;
    return SD_OK;
}

SD_Error_t SD_CheckRead(void) {
    if (SD_Handle.RXCplt != 0) return SD_BUSY;
    return SD_OK;
}

SD_Error_t SD_WriteBlocks_DMA(uint64_t WriteAddress, uint32_t *buffer, uint32_t BlockSize, uint32_t NumberOfBlocks)
{
    SD_Error_t ErrorState = SD_OK;
    SD_Handle.TXCplt = 1;

    if (BlockSize != 512) {
        return SD_ERROR; // unsupported.
    }

    if ((uint32_t)buffer & 0x1f) {
        return SD_ADDR_MISALIGNED;
    }

    // Ensure the data is flushed to main memory
    SCB_CleanDCache_by_Addr(buffer, NumberOfBlocks * BlockSize);

    HAL_StatusTypeDef status;
    if ((status = HAL_SD_WriteBlocks_DMA(&hsd1, (uint8_t *)buffer, WriteAddress, NumberOfBlocks)) != HAL_OK) {
        return SD_ERROR;
    }

    return ErrorState;
}

typedef struct {
    uint32_t *buffer;
    uint32_t BlockSize;
    uint32_t NumberOfBlocks;
} sdReadParameters_t;

sdReadParameters_t sdReadParameters;

SD_Error_t SD_ReadBlocks_DMA(uint64_t ReadAddress, uint32_t *buffer, uint32_t BlockSize, uint32_t NumberOfBlocks)
{
    SD_Error_t ErrorState = SD_OK;

    if (BlockSize != 512) {
        return SD_ERROR; // unsupported.
    }

    if ((uint32_t)buffer & 0x1f) {
        return SD_ADDR_MISALIGNED;
    }

    SD_Handle.RXCplt = 1;

    sdReadParameters.buffer = buffer;
    sdReadParameters.BlockSize = BlockSize;
    sdReadParameters.NumberOfBlocks = NumberOfBlocks;

    HAL_StatusTypeDef status;
    if ((status = HAL_SD_ReadBlocks_DMA(&hsd1, (uint8_t *)buffer, ReadAddress, NumberOfBlocks)) != HAL_OK) {
        return SD_ERROR;
    }

    return ErrorState;
}

/**
  * @brief Tx Transfer completed callback
  * @param hsd: SD handle
  * @retval None
  */
void HAL_SD_TxCpltCallback(SD_HandleTypeDef *hsd)
{
    UNUSED(hsd);

    SD_Handle.TXCplt = 0;
}

/**
  * @brief Rx Transfer completed callback
  * @param hsd: SD handle
  * @retval None
  */
void HAL_SD_RxCpltCallback(SD_HandleTypeDef *hsd)
{
    UNUSED(hsd);

    SD_Handle.RXCplt = 0;

    /*
       the SCB_InvalidateDCache_by_Addr() requires a 32-Byte aligned address,
       adjust the address and the D-Cache size to invalidate accordingly.
     */
    uint32_t alignedAddr = (uint32_t)sdReadParameters.buffer &  ~0x1F;
    SCB_InvalidateDCache_by_Addr((uint32_t*)alignedAddr, sdReadParameters.NumberOfBlocks * sdReadParameters.BlockSize + ((uint32_t)sdReadParameters.buffer - alignedAddr));
}

void HAL_SD_AbortCallback(SD_HandleTypeDef *hsd)
{
    UNUSED(hsd);

    SD_Handle.TXCplt = 0;
    SD_Handle.RXCplt = 0;
}

void SDMMC1_IRQHandler(void)
{
    HAL_SD_IRQHandler(&hsd1);
}

void SDMMC2_IRQHandler(void)
{
    HAL_SD_IRQHandler(&hsd1);
}

#endif
