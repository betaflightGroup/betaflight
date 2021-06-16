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

#define SPI_TIMEOUT_US  10000

#if defined(STM32F1) || defined(STM32F3) || defined(STM32F4) || defined(STM32G4)
#define MAX_SPI_PIN_SEL 2
#elif defined(STM32F7)
#define MAX_SPI_PIN_SEL 4
#elif defined(STM32H7)
#define MAX_SPI_PIN_SEL 5
#else
#error Unknown MCU family
#endif

typedef struct spiPinDef_s {
    ioTag_t pin;
#if defined(STM32F7) || defined(STM32H7) || defined(STM32G4)
    uint8_t af;
#endif
} spiPinDef_t;

typedef struct spiHardware_s {
    SPIDevice device;
    SPI_TypeDef *reg;
    spiPinDef_t sckPins[MAX_SPI_PIN_SEL];
    spiPinDef_t misoPins[MAX_SPI_PIN_SEL];
    spiPinDef_t mosiPins[MAX_SPI_PIN_SEL];
#ifndef STM32F7
    uint8_t af;
#endif
    rccPeriphTag_t rcc;
#ifdef USE_DMA
    uint8_t dmaIrqHandler;
#endif
} spiHardware_t;

extern const spiHardware_t spiHardware[];

typedef struct SPIDevice_s {
    SPI_TypeDef *dev;
    ioTag_t sck;
    ioTag_t miso;
    ioTag_t mosi;
#if defined(STM32F7) || defined(STM32H7) || defined(STM32G4)
    uint8_t sckAF;
    uint8_t misoAF;
    uint8_t mosiAF;
#else
    uint8_t af;
#endif
#if defined(HAL_SPI_MODULE_ENABLED)
    SPI_HandleTypeDef hspi;
#endif
    rccPeriphTag_t rcc;
    volatile uint16_t errorCount;
    bool leadingEdge;
#ifdef USE_DMA
    uint8_t dmaIrqHandler;
#endif
} spiDevice_t;

extern spiDevice_t spiDevice[SPIDEV_COUNT];

void spiInitDevice(SPIDevice device);
void spiInternalInitStream(const extDevice_t *dev, bool preInit);
void spiInternalStartDMA(const extDevice_t *dev);
void spiInternalStopDMA (const extDevice_t *dev);
void spiInternalResetStream(dmaChannelDescriptor_t *descriptor);
void spiInternalResetDescriptors(busDevice_t *bus);

