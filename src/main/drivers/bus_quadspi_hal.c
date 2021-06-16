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
 *
 * Author: Dominic Clifton
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_QUADSPI

#include "bus_quadspi.h"
#include "bus_quadspi_impl.h"
#include "dma.h"
#include "io.h"
#include "io_impl.h"
#include "nvic.h"
#include "rcc.h"

#include "pg/bus_quadspi.h"

static void Error_Handler(void) { while (1) { } }

void quadSpiInitDevice(QUADSPIDevice device)
{
    quadSpiDevice_t *quadSpi = &(quadSpiDevice[device]);

    // Enable QUADSPI clock
    RCC_ClockCmd(quadSpi->rcc, ENABLE);
    RCC_ResetCmd(quadSpi->rcc, ENABLE);

    IOInit(IOGetByTag(quadSpi->clk),  OWNER_QUADSPI_CLK,  RESOURCE_INDEX(device));
    IOInit(IOGetByTag(quadSpi->bk1IO0), OWNER_QUADSPI_BK1IO0, RESOURCE_INDEX(device));
    IOInit(IOGetByTag(quadSpi->bk1IO1), OWNER_QUADSPI_BK1IO1, RESOURCE_INDEX(device));
    IOInit(IOGetByTag(quadSpi->bk1IO2), OWNER_QUADSPI_BK1IO2, RESOURCE_INDEX(device));
    IOInit(IOGetByTag(quadSpi->bk1IO3), OWNER_QUADSPI_BK1IO3, RESOURCE_INDEX(device));
    IOInit(IOGetByTag(quadSpi->bk1CS), OWNER_QUADSPI_BK1CS, RESOURCE_INDEX(device));

    IOInit(IOGetByTag(quadSpi->bk2IO0), OWNER_QUADSPI_BK2IO0, RESOURCE_INDEX(device));
    IOInit(IOGetByTag(quadSpi->bk2IO1), OWNER_QUADSPI_BK2IO1, RESOURCE_INDEX(device));
    IOInit(IOGetByTag(quadSpi->bk2IO2), OWNER_QUADSPI_BK2IO2, RESOURCE_INDEX(device));
    IOInit(IOGetByTag(quadSpi->bk2IO3), OWNER_QUADSPI_BK2IO3, RESOURCE_INDEX(device));
    IOInit(IOGetByTag(quadSpi->bk2CS), OWNER_QUADSPI_BK2CS, RESOURCE_INDEX(device));

#if defined(STM32H7)
    // clock is only on AF9
    // IO and CS lines are on AF9 and AF10
    IOConfigGPIOAF(IOGetByTag(quadSpi->clk), QUADSPI_IO_AF_CLK_CFG, GPIO_AF9_QUADSPI);
    IOConfigGPIOAF(IOGetByTag(quadSpi->bk1IO0), QUADSPI_IO_AF_BK_IO_CFG, quadSpi->bk1IO0AF);
    IOConfigGPIOAF(IOGetByTag(quadSpi->bk1IO1), QUADSPI_IO_AF_BK_IO_CFG, quadSpi->bk1IO1AF);
    IOConfigGPIOAF(IOGetByTag(quadSpi->bk1IO2), QUADSPI_IO_AF_BK_IO_CFG, quadSpi->bk1IO2AF);
    IOConfigGPIOAF(IOGetByTag(quadSpi->bk1IO3), QUADSPI_IO_AF_BK_IO_CFG, quadSpi->bk1IO3AF);
    IOConfigGPIOAF(IOGetByTag(quadSpi->bk2IO0), QUADSPI_IO_AF_BK_IO_CFG, quadSpi->bk2IO0AF);
    IOConfigGPIOAF(IOGetByTag(quadSpi->bk2IO1), QUADSPI_IO_AF_BK_IO_CFG, quadSpi->bk2IO1AF);
    IOConfigGPIOAF(IOGetByTag(quadSpi->bk2IO2), QUADSPI_IO_AF_BK_IO_CFG, quadSpi->bk2IO2AF);
    IOConfigGPIOAF(IOGetByTag(quadSpi->bk2IO3), QUADSPI_IO_AF_BK_IO_CFG, quadSpi->bk2IO3AF);

    if ((quadSpiConfig(device)->csFlags & QUADSPI_BK1_CS_MASK) == QUADSPI_BK1_CS_HARDWARE) {
        IOConfigGPIOAF(IOGetByTag(quadSpi->bk1CS), QUADSPI_IO_AF_BK_CS_CFG, quadSpi->bk1CSAF);
    } else {
        IOConfigGPIO(IOGetByTag(quadSpi->bk1CS), QUADSPI_IO_BK_CS_CFG);
    }

    if ((quadSpiConfig(device)->csFlags & QUADSPI_BK2_CS_MASK) == QUADSPI_BK2_CS_HARDWARE) {
        IOConfigGPIOAF(IOGetByTag(quadSpi->bk2CS), QUADSPI_IO_AF_BK_CS_CFG, quadSpi->bk2CSAF);
    } else {
        IOConfigGPIO(IOGetByTag(quadSpi->bk2CS), QUADSPI_IO_BK_CS_CFG);
    }
#endif

    quadSpi->hquadSpi.Instance = quadSpi->dev;
    // DeInit QUADSPI hardware
    HAL_QSPI_DeInit(&quadSpi->hquadSpi);

    quadSpi->hquadSpi.Init.ClockPrescaler = QUADSPI_CLOCK_INITIALISATION;
    quadSpi->hquadSpi.Init.FifoThreshold = 1;
    quadSpi->hquadSpi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_NONE;
    quadSpi->hquadSpi.Init.FlashSize = 23; // address bits + 1
    quadSpi->hquadSpi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
    quadSpi->hquadSpi.Init.ClockMode = QSPI_CLOCK_MODE_0;

    switch (quadSpiConfig(device)->mode) {
    case QUADSPI_MODE_BK1_ONLY:
        quadSpi->hquadSpi.Init.FlashID = QSPI_FLASH_ID_1;
        quadSpi->hquadSpi.Init.DualFlash = QSPI_DUALFLASH_DISABLE;
        break;
    case QUADSPI_MODE_BK2_ONLY:
        quadSpi->hquadSpi.Init.FlashID = QSPI_FLASH_ID_2;
        quadSpi->hquadSpi.Init.DualFlash = QSPI_DUALFLASH_DISABLE;
        break;
    case QUADSPI_MODE_DUAL_FLASH:
        quadSpi->hquadSpi.Init.DualFlash = QSPI_DUALFLASH_ENABLE;
        break;
    }

    // Init QUADSPI hardware
    if (HAL_QSPI_Init(&quadSpi->hquadSpi) != HAL_OK)
    {
      Error_Handler();
    }
}

static const uint32_t quadSpi_addressSizeMap[] = {
    QSPI_ADDRESS_8_BITS,
    QSPI_ADDRESS_16_BITS,
    QSPI_ADDRESS_24_BITS,
    QSPI_ADDRESS_32_BITS
};

static uint32_t quadSpi_addressSizeFromValue(uint8_t addressSize)
{
    return quadSpi_addressSizeMap[((addressSize + 1) / 8) - 1]; // rounds to nearest QSPI_ADDRESS_* value that will hold the address.
}

/**
 * Return true if the bus is currently in the middle of a transmission.
 */
bool quadSpiIsBusBusy(QUADSPI_TypeDef *instance)
{
    QUADSPIDevice device = quadSpiDeviceByInstance(instance);
    if(quadSpiDevice[device].hquadSpi.State == HAL_QSPI_STATE_BUSY)
        return true;
    else
        return false;
}

#define QUADSPI_DEFAULT_TIMEOUT 10

void quadSpiSelectDevice(QUADSPI_TypeDef *instance)
{
    QUADSPIDevice device = quadSpiDeviceByInstance(instance);

    IO_t bk1CS = IOGetByTag(quadSpiDevice[device].bk1CS);
    IO_t bk2CS = IOGetByTag(quadSpiDevice[device].bk2CS);

    switch(quadSpiConfig(device)->mode) {
    case QUADSPI_MODE_DUAL_FLASH:
        if ((quadSpiConfig(device)->csFlags & QUADSPI_BK1_CS_MASK) == QUADSPI_BK1_CS_SOFTWARE) {
            IOLo(bk1CS);
        }
        if ((quadSpiConfig(device)->csFlags & QUADSPI_BK2_CS_MASK) == QUADSPI_BK2_CS_SOFTWARE) {
            IOLo(bk2CS);
        }
        break;
    case QUADSPI_MODE_BK1_ONLY:
        if ((quadSpiConfig(device)->csFlags & QUADSPI_BK1_CS_MASK) == QUADSPI_BK1_CS_SOFTWARE) {
            IOLo(bk1CS);
        }
        break;
    case QUADSPI_MODE_BK2_ONLY:
        if ((quadSpiConfig(device)->csFlags & QUADSPI_BK2_CS_MASK) == QUADSPI_BK2_CS_SOFTWARE) {
            IOLo(bk2CS);
        }
        break;
    }
}

void quadSpiDeselectDevice(QUADSPI_TypeDef *instance)
{
    QUADSPIDevice device = quadSpiDeviceByInstance(instance);

    IO_t bk1CS = IOGetByTag(quadSpiDevice[device].bk1CS);
    IO_t bk2CS = IOGetByTag(quadSpiDevice[device].bk2CS);

    switch(quadSpiConfig(device)->mode) {
    case QUADSPI_MODE_DUAL_FLASH:
        if ((quadSpiConfig(device)->csFlags & QUADSPI_BK1_CS_MASK) == QUADSPI_BK1_CS_SOFTWARE) {
            IOHi(bk1CS);
        }
        if ((quadSpiConfig(device)->csFlags & QUADSPI_BK2_CS_MASK) == QUADSPI_BK2_CS_SOFTWARE) {
            IOHi(bk2CS);
        }
        break;
    case QUADSPI_MODE_BK1_ONLY:
        if ((quadSpiConfig(device)->csFlags & QUADSPI_BK1_CS_MASK) == QUADSPI_BK1_CS_SOFTWARE) {
            IOHi(bk1CS);
        }
        break;
    case QUADSPI_MODE_BK2_ONLY:
        if ((quadSpiConfig(device)->csFlags & QUADSPI_BK2_CS_MASK) == QUADSPI_BK2_CS_SOFTWARE) {
            IOHi(bk2CS);
        }
        break;
    }
}



bool quadSpiTransmit1LINE(QUADSPI_TypeDef *instance, uint8_t instruction, uint8_t dummyCycles, const uint8_t *out, int length)
{
    QUADSPIDevice device = quadSpiDeviceByInstance(instance);
    HAL_StatusTypeDef status;


    QSPI_CommandTypeDef cmd;
    cmd.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
    cmd.AddressMode       = QSPI_ADDRESS_NONE;
    cmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    cmd.DataMode          = QSPI_DATA_NONE;
    cmd.DummyCycles       = dummyCycles;
    cmd.DdrMode           = QSPI_DDR_MODE_DISABLE;
    cmd.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
    cmd.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

    cmd.Instruction       = instruction;
    cmd.NbData            = length;

    if (out) {
        cmd.DataMode          = QSPI_DATA_1_LINE;
    }

    quadSpiSelectDevice(instance);

    status = HAL_QSPI_Command(&quadSpiDevice[device].hquadSpi, &cmd, QUADSPI_DEFAULT_TIMEOUT);
    bool timeout = (status != HAL_OK);
    if (!timeout) {
        if (out && length > 0) {
            status = HAL_QSPI_Transmit(&quadSpiDevice[device].hquadSpi, (uint8_t *)out, QUADSPI_DEFAULT_TIMEOUT);
            timeout = (status != HAL_OK);
        }
    }

    quadSpiDeselectDevice(instance);

    if (timeout) {
        quadSpiTimeoutUserCallback(instance);
        return false;
    }

    return true;
}

bool quadSpiReceive1LINE(QUADSPI_TypeDef *instance, uint8_t instruction, uint8_t dummyCycles, uint8_t *in, int length)
{
    QUADSPIDevice device = quadSpiDeviceByInstance(instance);
    HAL_StatusTypeDef status;

    QSPI_CommandTypeDef cmd;
    cmd.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
    cmd.AddressMode       = QSPI_ADDRESS_NONE;
    cmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    cmd.DataMode          = QSPI_DATA_1_LINE;
    cmd.DummyCycles       = dummyCycles;
    cmd.DdrMode           = QSPI_DDR_MODE_DISABLE;
    cmd.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
    cmd.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

    cmd.Instruction       = instruction;
    cmd.NbData            = length;

    quadSpiSelectDevice(instance);

    status = HAL_QSPI_Command(&quadSpiDevice[device].hquadSpi, &cmd, QUADSPI_DEFAULT_TIMEOUT);
    bool timeout = (status != HAL_OK);
    if (!timeout) {
        status = HAL_QSPI_Receive(&quadSpiDevice[device].hquadSpi, in, QUADSPI_DEFAULT_TIMEOUT);

        timeout = (status != HAL_OK);
    }

    quadSpiDeselectDevice(instance);

    if (timeout) {
        quadSpiTimeoutUserCallback(instance);
        return false;
    }

    return true;
}

bool quadSpiReceive4LINES(QUADSPI_TypeDef *instance, uint8_t instruction, uint8_t dummyCycles, uint8_t *in, int length)
{
    QUADSPIDevice device = quadSpiDeviceByInstance(instance);
    HAL_StatusTypeDef status;

    QSPI_CommandTypeDef cmd;
    cmd.InstructionMode   = QSPI_INSTRUCTION_4_LINES;
    cmd.AddressMode       = QSPI_ADDRESS_NONE;
    cmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    cmd.DataMode          = QSPI_DATA_4_LINES;
    cmd.DummyCycles       = dummyCycles;
    cmd.DdrMode           = QSPI_DDR_MODE_DISABLE;
    cmd.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
    cmd.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

    cmd.Instruction       = instruction;
    cmd.NbData            = length;

    quadSpiSelectDevice(instance);

    status = HAL_QSPI_Command(&quadSpiDevice[device].hquadSpi, &cmd, QUADSPI_DEFAULT_TIMEOUT);
    bool timeout = (status != HAL_OK);
    if (!timeout) {
        status = HAL_QSPI_Receive(&quadSpiDevice[device].hquadSpi, in, QUADSPI_DEFAULT_TIMEOUT);

        timeout = (status != HAL_OK);
    }

    quadSpiDeselectDevice(instance);

    if (timeout) {
        quadSpiTimeoutUserCallback(instance);
        return false;
    }

    return true;
}

bool quadSpiReceiveWithAddress1LINE(QUADSPI_TypeDef *instance, uint8_t instruction, uint8_t dummyCycles, uint32_t address, uint8_t addressSize, uint8_t *in, int length)
{
    QUADSPIDevice device = quadSpiDeviceByInstance(instance);
    HAL_StatusTypeDef status;

    QSPI_CommandTypeDef cmd;
    cmd.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
    cmd.AddressMode       = QSPI_ADDRESS_1_LINE;
    cmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    cmd.DataMode          = QSPI_DATA_1_LINE;
    cmd.DummyCycles       = dummyCycles;
    cmd.DdrMode           = QSPI_DDR_MODE_DISABLE;
    cmd.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
    cmd.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

    cmd.Instruction       = instruction;
    cmd.Address           = address;
    cmd.AddressSize       = quadSpi_addressSizeFromValue(addressSize);
    cmd.NbData            = length;

    quadSpiSelectDevice(instance);

    status = HAL_QSPI_Command(&quadSpiDevice[device].hquadSpi, &cmd, QUADSPI_DEFAULT_TIMEOUT);
    bool timeout = (status != HAL_OK);
    if (!timeout) {
        status = HAL_QSPI_Receive(&quadSpiDevice[device].hquadSpi, in, QUADSPI_DEFAULT_TIMEOUT);
        timeout = (status != HAL_OK);
    }

    quadSpiDeselectDevice(instance);

    if (timeout) {
        quadSpiTimeoutUserCallback(instance);
        return false;
    }

    return true;
}


bool quadSpiReceiveWithAddress4LINES(QUADSPI_TypeDef *instance, uint8_t instruction, uint8_t dummyCycles, uint32_t address, uint8_t addressSize, uint8_t *in, int length)
{
    QUADSPIDevice device = quadSpiDeviceByInstance(instance);
    HAL_StatusTypeDef status;

    QSPI_CommandTypeDef cmd;
    cmd.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
    cmd.AddressMode       = QSPI_ADDRESS_1_LINE;
    cmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    cmd.DataMode          = QSPI_DATA_4_LINES;
    cmd.DummyCycles       = dummyCycles;
    cmd.DdrMode           = QSPI_DDR_MODE_DISABLE;
    cmd.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
    cmd.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

    cmd.Instruction       = instruction;
    cmd.Address           = address;
    cmd.AddressSize       = quadSpi_addressSizeFromValue(addressSize);
    cmd.NbData            = length;

    quadSpiSelectDevice(instance);

    status = HAL_QSPI_Command(&quadSpiDevice[device].hquadSpi, &cmd, QUADSPI_DEFAULT_TIMEOUT);
    bool timeout = (status != HAL_OK);
    if (!timeout) {
        status = HAL_QSPI_Receive(&quadSpiDevice[device].hquadSpi, in, QUADSPI_DEFAULT_TIMEOUT);
        timeout = (status != HAL_OK);
    }

    quadSpiDeselectDevice(instance);

    if (timeout) {
        quadSpiTimeoutUserCallback(instance);
        return false;
    }

    return true;
}
bool quadSpiTransmitWithAddress1LINE(QUADSPI_TypeDef *instance, uint8_t instruction, uint8_t dummyCycles, uint32_t address, uint8_t addressSize, const uint8_t *out, int length)
{
    QUADSPIDevice device = quadSpiDeviceByInstance(instance);
    HAL_StatusTypeDef status;

    QSPI_CommandTypeDef cmd;
    cmd.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
    cmd.AddressMode       = QSPI_ADDRESS_1_LINE;
    cmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    cmd.DataMode          = QSPI_DATA_1_LINE;
    cmd.DummyCycles       = dummyCycles;
    cmd.DdrMode           = QSPI_DDR_MODE_DISABLE;
    cmd.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
    cmd.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

    cmd.Instruction       = instruction;
    cmd.Address           = address;
    cmd.AddressSize       = quadSpi_addressSizeFromValue(addressSize);
    cmd.NbData            = length;

    quadSpiSelectDevice(instance);

    status = HAL_QSPI_Command(&quadSpiDevice[device].hquadSpi, &cmd, QUADSPI_DEFAULT_TIMEOUT);
    bool timeout = (status != HAL_OK);

    if (!timeout) {
        status = HAL_QSPI_Transmit(&quadSpiDevice[device].hquadSpi, (uint8_t *)out, QUADSPI_DEFAULT_TIMEOUT);
        timeout = (status != HAL_OK);
    }

    quadSpiDeselectDevice(instance);

    if (timeout) {
        quadSpiTimeoutUserCallback(instance);
        return false;
    }

    return true;
}

bool quadSpiTransmitWithAddress4LINES(QUADSPI_TypeDef *instance, uint8_t instruction, uint8_t dummyCycles, uint32_t address, uint8_t addressSize, const uint8_t *out, int length)
{
    QUADSPIDevice device = quadSpiDeviceByInstance(instance);
    HAL_StatusTypeDef status;

    QSPI_CommandTypeDef cmd;
    cmd.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
    cmd.AddressMode       = QSPI_ADDRESS_1_LINE;
    cmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    cmd.DataMode          = QSPI_DATA_4_LINES;
    cmd.DummyCycles       = dummyCycles;
    cmd.DdrMode           = QSPI_DDR_MODE_DISABLE;
    cmd.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
    cmd.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

    cmd.Instruction       = instruction;
    cmd.Address           = address;
    cmd.AddressSize       = quadSpi_addressSizeFromValue(addressSize);
    cmd.NbData            = length;

    quadSpiSelectDevice(instance);

    status = HAL_QSPI_Command(&quadSpiDevice[device].hquadSpi, &cmd, QUADSPI_DEFAULT_TIMEOUT);
    bool timeout = (status != HAL_OK);

    if (!timeout) {
        status = HAL_QSPI_Transmit(&quadSpiDevice[device].hquadSpi, (uint8_t *)out, QUADSPI_DEFAULT_TIMEOUT);
        timeout = (status != HAL_OK);
    }

    quadSpiDeselectDevice(instance);

    if (timeout) {
        quadSpiTimeoutUserCallback(instance);
        return false;
    }

    return true;
}

bool quadSpiInstructionWithAddress1LINE(QUADSPI_TypeDef *instance, uint8_t instruction, uint8_t dummyCycles, uint32_t address, uint8_t addressSize)
{
    QUADSPIDevice device = quadSpiDeviceByInstance(instance);
    HAL_StatusTypeDef status;

    QSPI_CommandTypeDef cmd;
    cmd.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
    cmd.AddressMode       = QSPI_ADDRESS_1_LINE;
    cmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    cmd.DataMode          = QSPI_DATA_NONE;
    cmd.DummyCycles       = dummyCycles;
    cmd.DdrMode           = QSPI_DDR_MODE_DISABLE;
    cmd.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
    cmd.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

    cmd.Instruction       = instruction;
    cmd.Address           = address;
    cmd.AddressSize       = quadSpi_addressSizeFromValue(addressSize);
    cmd.NbData            = 0;

    quadSpiSelectDevice(instance);

    status = HAL_QSPI_Command(&quadSpiDevice[device].hquadSpi, &cmd, QUADSPI_DEFAULT_TIMEOUT);
    bool timeout = (status != HAL_OK);

    quadSpiDeselectDevice(instance);

    if (timeout) {
        quadSpiTimeoutUserCallback(instance);
        return false;
    }


    return true;
}

bool quadSpiInstructionWithData1LINE(QUADSPI_TypeDef *instance, uint8_t instruction, uint8_t dummyCycles, const uint8_t *out, int length)
{
    QUADSPIDevice device = quadSpiDeviceByInstance(instance);
    HAL_StatusTypeDef status;

    QSPI_CommandTypeDef cmd;
    cmd.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
    cmd.AddressMode       = QSPI_ADDRESS_NONE;
    cmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    cmd.DataMode          = QSPI_DATA_1_LINE;
    cmd.DummyCycles       = dummyCycles;
    cmd.DdrMode           = QSPI_DDR_MODE_DISABLE;
    cmd.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
    cmd.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

    cmd.Instruction       = instruction;
    cmd.NbData            = length;

    quadSpiSelectDevice(instance);

    status = HAL_QSPI_Command(&quadSpiDevice[device].hquadSpi, &cmd, QUADSPI_DEFAULT_TIMEOUT);
    bool timeout =(status != HAL_OK);

    if (!timeout) {
        status = HAL_QSPI_Transmit(&quadSpiDevice[device].hquadSpi, (uint8_t *)out, QUADSPI_DEFAULT_TIMEOUT);
        timeout = (status != HAL_OK);
    }

    quadSpiDeselectDevice(instance);

    if (timeout) {
        quadSpiTimeoutUserCallback(instance);
        return false;
    }

    return true;
}

void quadSpiSetDivisor(QUADSPI_TypeDef *instance, uint16_t divisor)
{
    QUADSPIDevice device = quadSpiDeviceByInstance(instance);
    if (HAL_QSPI_DeInit(&quadSpiDevice[device].hquadSpi) != HAL_OK)
    {
        Error_Handler();
    }

    quadSpiDevice_t *quadSpi = &(quadSpiDevice[device]);

    quadSpi->hquadSpi.Init.ClockPrescaler = divisor;

    HAL_QSPI_Init(&quadSpi->hquadSpi);
}
#endif
