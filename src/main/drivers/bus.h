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

#pragma once

#include "platform.h"

#include "drivers/bus_i2c.h"
#include "drivers/io_types.h"
#include "drivers/dma.h"

typedef enum {
    BUS_TYPE_NONE = 0,
    BUS_TYPE_I2C,
    BUS_TYPE_SPI,
    BUS_TYPE_MPU_SLAVE, // Slave I2C on SPI master
    BUS_TYPE_GYRO_AUTO,  // Only used by acc/gyro bus auto detection code
} busType_e;

struct spiDevice_s;

typedef enum {
    BUS_READY,
    BUS_BUSY,
    BUS_ABORT
} busStatus_e;


// Bus interface, independent of connected device
typedef struct busDevice_s {
    busType_e busType;
    union {
        struct busSpi_s {
            SPI_TypeDef *instance;
            uint16_t speed;
            bool leadingEdge;
        } spi;
        struct busI2C_s {
            I2CDevice device;
        } i2c;
        struct busMpuSlave_s {
            struct extDevice_s *master;
        } mpuSlave;
    } busType_u;
    bool useDMA;
    bool useAtomicWait;
    uint8_t deviceCount;
    resourceOwner_e owner; // owner of first device to use this bus
    dmaChannelDescriptor_t *dmaTx;
    dmaChannelDescriptor_t *dmaRx;
    uint32_t dmaTxChannel;
    uint32_t dmaRxChannel;
#ifndef UNIT_TEST
    // Use a reference here as this saves RAM for unused descriptors
#if defined(USE_FULL_LL_DRIVER)
    LL_DMA_InitTypeDef          *initTx;
    LL_DMA_InitTypeDef          *initRx;
#else
    DMA_InitTypeDef             *initTx;
    DMA_InitTypeDef             *initRx;
#endif
#endif // UNIT_TEST
    struct busSegment_s* volatile curSegment;
    bool initSegment;
} busDevice_t;

/* Each SPI access may comprise multiple parts, for example, wait/write enable/write/data each of which
 * is defined by a segment, with optional callback after each is completed
 */
typedef struct busSegment_s {
    uint8_t *txData;
    uint8_t *rxData;
    int len;
    bool negateCS; // Should CS be negated at the end of this segment
    busStatus_e (*callback)(uint32_t arg);
} busSegment_t;

// External device has an associated bus and bus dependent address
typedef struct extDevice_s {
    busDevice_t *bus;
    union {
        struct extSpi_s {
            uint16_t speed;
            IO_t csnPin;
            bool leadingEdge;
        } spi;
        struct extI2C_s {
            uint8_t address;
        } i2c;
        struct extMpuSlave_s {
            uint8_t address;
        } mpuSlave;
    } busType_u;
#ifndef UNIT_TEST
    // Cache the init structure for the next DMA transfer to reduce inter-segment delay
#if defined(USE_HAL_DRIVER)
    LL_DMA_InitTypeDef          initTx;
    LL_DMA_InitTypeDef          initRx;
#else
    DMA_InitTypeDef             initTx;
    DMA_InitTypeDef             initRx;
#endif
#endif // UNIT_TEST
    // Per device buffer reference if needed
    uint8_t *txBuf, *rxBuf;
    // Connected devices on the same bus may support different speeds
    uint32_t callbackArg;
} extDevice_t;


#ifdef TARGET_BUS_INIT
void targetBusInit(void);
#endif

// Access routines where the register is accessed directly
bool busRawWriteRegister(const extDevice_t *dev, uint8_t reg, uint8_t data);
bool busRawWriteRegisterStart(const extDevice_t *dev, uint8_t reg, uint8_t data);
bool busRawReadRegisterBuffer(const extDevice_t *dev, uint8_t reg, uint8_t *data, uint8_t length);
bool busRawReadRegisterBufferStart(const extDevice_t *dev, uint8_t reg, uint8_t *data, uint8_t length);
// Write routines where the register is masked with 0x7f
bool busWriteRegister(const extDevice_t *dev, uint8_t reg, uint8_t data);
bool busWriteRegisterStart(const extDevice_t *dev, uint8_t reg, uint8_t data);
// Read routines where the register is ORed with 0x80
bool busReadRegisterBuffer(const extDevice_t *dev, uint8_t reg, uint8_t *data, uint8_t length);
bool busReadRegisterBufferStart(const extDevice_t *dev, uint8_t reg, uint8_t *data, uint8_t length);
uint8_t busReadRegister(const extDevice_t *dev, uint8_t reg);

bool busBusy(const extDevice_t *dev, bool *error);
void busDeviceRegister(const extDevice_t *dev);
