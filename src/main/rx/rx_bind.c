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

#if defined(USE_RX_BIND)

#include "rx/rx_spi_common.h"
#include "rx/srxl2.h"

#include "rx_bind.h"

static bool doRxBind(bool doBind)
{
    switch (rxRuntimeState.rxProvider) {
    default:
        return false;
    case RX_PROVIDER_SERIAL:
        switch (rxRuntimeState.serialrxProvider) {
        default:
            return false;
#if defined(USE_SERIALRX_SRXL2)
        case SERIALRX_SRXL2:
            if (doBind) {
                srxl2Bind();
            }

            break;
#endif
        }

        break;
#if defined(USE_RX_SPI)
    case RX_PROVIDER_SPI:
        switch (rxSpiConfig()->rx_spi_protocol) {
        default:
            return false;
#if defined(USE_RX_FRSKY_SPI)
#if defined(USE_RX_FRSKY_SPI_D)
        case RX_SPI_FRSKY_D:
#endif
#if defined(USE_RX_FRSKY_SPI_X)
        case RX_SPI_FRSKY_X:
        case RX_SPI_FRSKY_X_LBT:
        case RX_SPI_FRSKY_X_V2:
        case RX_SPI_FRSKY_X_LBT_V2:
#endif
#if defined(USE_RX_REDPINE_SPI)
        case RX_SPI_REDPINE:
#endif
#endif // USE_RX_FRSKY_SPI
#ifdef USE_RX_SFHSS_SPI
        case RX_SPI_SFHSS:
#endif
#ifdef USE_RX_FLYSKY
        case RX_SPI_A7105_FLYSKY:
        case RX_SPI_A7105_FLYSKY_2A:
#endif
#ifdef USE_RX_SPEKTRUM
        case RX_SPI_CYRF6936_DSM:
#endif
#ifdef USE_RX_EXPRESSLRS
        case RX_SPI_EXPRESSLRS:
#endif
#if defined(USE_RX_FRSKY_SPI) || defined(USE_RX_SFHSS_SPI) || defined(USE_RX_FLYSKY) || defined(USE_RX_SPEKTRUM) || defined(USE_RX_EXPRESSLRS)
            if (doBind) {
                rxSpiBind();
            }

            break;
#endif
        }

        break;
#endif
    }

    return true;
}

bool startRxBind(void)
{
    return doRxBind(true);
}

bool getRxBindSupported(void)
{
    return doRxBind(false);
}
#endif
