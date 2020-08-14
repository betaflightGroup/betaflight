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

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_SDCARD

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "sdcard.h"
#include "drivers/bus_spi.h"
#include "drivers/io.h"
#include "drivers/dma.h"
#include "drivers/dma_reqmap.h"

PG_REGISTER_WITH_RESET_FN(sdcardConfig_t, sdcardConfig, PG_SDCARD_CONFIG, 1);

void pgResetFn_sdcardConfig(sdcardConfig_t *config)
{
    config->cardDetectTag = IO_TAG(SDCARD_DETECT_PIN);
    config->cardDetectInverted = SDCARD_DETECT_IS_INVERTED;

    // We can safely handle SPI and SDIO cases separately on custom targets, as these are exclusive per target.
    // On generic targets, SPI has precedence over SDIO; SDIO must be post-flash configured.
    config->useDma = false;
    config->device = SPI_DEV_TO_CFG(SPIINVALID);
    config->mode = SDCARD_MODE_NONE;

#if defined(STM32H7) && defined(USE_SDCARD_SDIO) // H7 only for now, likely should be applied to F4/F7 too
    config->mode = SDCARD_MODE_SDIO;
    config->useDma = true;
#endif

#ifdef USE_SDCARD_SPI
    // These settings do not work for Unified Targets
    // They are only left in place to support legacy targets
    SPIDevice spidevice = spiDeviceByInstance(SDCARD_SPI_INSTANCE);
    config->device = SPI_DEV_TO_CFG(spidevice);
    config->chipSelectTag = IO_TAG(SDCARD_SPI_CS_PIN);

    if (spidevice != SPIINVALID && config->chipSelectTag) {
        config->mode = SDCARD_MODE_SPI;
    }
#endif

#ifndef USE_DMA_SPEC
#ifdef USE_SDCARD_SPI
#if defined(SDCARD_DMA_STREAM_TX_FULL)
    config->dmaIdentifier = (uint8_t)dmaGetIdentifier(SDCARD_DMA_STREAM_TX_FULL);
#elif defined(SDCARD_DMA_CHANNEL_TX)
    config->dmaIdentifier = (uint8_t)dmaGetIdentifier(SDCARD_DMA_CHANNEL_TX);
#endif
#endif
#endif // !USE_DMA_SPEC
}
#endif
