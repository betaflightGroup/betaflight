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

#include "platform.h"

#ifdef USE_SPI

#include "drivers/dma_reqmap.h"
#include "drivers/io.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "bus_spi.h"

typedef struct spiDefaultConfig_s {
    SPIDevice device;
    ioTag_t sck;
    ioTag_t miso;
    ioTag_t mosi;
    dmaoptValue_t txDmaopt;
    dmaoptValue_t rxDmaopt;
} spiDefaultConfig_t;

const spiDefaultConfig_t spiDefaultConfig[] = {
#ifdef USE_SPI_DEVICE_1
    { SPIDEV_1, IO_TAG(SPI1_SCK_PIN), IO_TAG(SPI1_MISO_PIN), IO_TAG(SPI1_MOSI_PIN), SPI1_TX_DMA_OPT, SPI1_RX_DMA_OPT },
#endif
#ifdef USE_SPI_DEVICE_2
    { SPIDEV_2, IO_TAG(SPI2_SCK_PIN), IO_TAG(SPI2_MISO_PIN), IO_TAG(SPI2_MOSI_PIN), SPI2_TX_DMA_OPT, SPI2_RX_DMA_OPT },
#endif
#ifdef USE_SPI_DEVICE_3
    { SPIDEV_3, IO_TAG(SPI3_SCK_PIN), IO_TAG(SPI3_MISO_PIN), IO_TAG(SPI3_MOSI_PIN), SPI3_TX_DMA_OPT, SPI3_RX_DMA_OPT },
#endif
#ifdef USE_SPI_DEVICE_4
    { SPIDEV_4, IO_TAG(SPI4_SCK_PIN), IO_TAG(SPI4_MISO_PIN), IO_TAG(SPI4_MOSI_PIN), SPI4_TX_DMA_OPT, SPI4_RX_DMA_OPT },
#endif
#ifdef USE_SPI_DEVICE_5
    { SPIDEV_5, IO_TAG(SPI5_SCK_PIN), IO_TAG(SPI5_MISO_PIN), IO_TAG(SPI5_MOSI_PIN), SPI5_TX_DMA_OPT, SPI5_RX_DMA_OPT },
#endif
#ifdef USE_SPI_DEVICE_6
    { SPIDEV_6, IO_TAG(SPI6_SCK_PIN), IO_TAG(SPI6_MISO_PIN), IO_TAG(SPI6_MOSI_PIN), SPI6_TX_DMA_OPT, SPI6_RX_DMA_OPT },
#endif
};

PG_REGISTER_ARRAY_WITH_RESET_FN(spiPinConfig_t, SPIDEV_COUNT, spiPinConfig, PG_SPI_PIN_CONFIG, 1);

void pgResetFn_spiPinConfig(spiPinConfig_t *spiPinConfig)
{
    for (size_t i = 0; i < SPIDEV_COUNT; i++) {
        spiPinConfig[i].txDmaopt = -1;
        spiPinConfig[i].rxDmaopt = -1;
    }

    for (size_t i = 0 ; i < ARRAYLEN(spiDefaultConfig) ; i++) {
        const spiDefaultConfig_t *defconf = &spiDefaultConfig[i];
        spiPinConfig[defconf->device].ioTagSck = defconf->sck;
        spiPinConfig[defconf->device].ioTagMiso = defconf->miso;
        spiPinConfig[defconf->device].ioTagMosi = defconf->mosi;
        spiPinConfig[defconf->device].txDmaopt = defconf->txDmaopt;
        spiPinConfig[defconf->device].rxDmaopt = defconf->rxDmaopt;
    }
}
#endif
