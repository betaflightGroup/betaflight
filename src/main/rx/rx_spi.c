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

#ifdef USE_RX_SPI

#include "build/build_config.h"

#include "common/utils.h"

#include "config/feature.h"

#include "drivers/io.h"
#include "drivers/rx/rx_spi.h"
#include "drivers/rx/rx_nrf24l01.h"

#include "config/config.h"

#include "pg/rx_spi.h"

#include "rx/rx_spi.h"
#include "rx/cc2500_frsky_common.h"
#include "rx/cc2500_redpine.h"
#include "rx/nrf24_cx10.h"
#include "rx/nrf24_syma.h"
#include "rx/nrf24_v202.h"
#include "rx/nrf24_h8_3d.h"
#include "rx/nrf24_inav.h"
#include "rx/nrf24_kn.h"
#include "rx/a7105_flysky.h"
#include "rx/cc2500_sfhss.h"
#include "rx/cyrf6936_spektrum.h"


uint16_t rxSpiRcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];
STATIC_UNIT_TESTED uint8_t rxSpiPayload[RX_SPI_MAX_PAYLOAD_SIZE];
STATIC_UNIT_TESTED uint8_t rxSpiNewPacketAvailable; // set true when a new packet is received

typedef bool (*protocolInitFnPtr)(const rxSpiConfig_t *rxSpiConfig, rxRuntimeState_t *rxRuntimeState, rxSpiExtiConfig_t *extiConfig);
typedef rx_spi_received_e (*protocolDataReceivedFnPtr)(uint8_t *payload);
typedef rx_spi_received_e (*protocolProcessFrameFnPtr)(uint8_t *payload);
typedef void (*protocolSetRcDataFromPayloadFnPtr)(uint16_t *rcData, const uint8_t *payload);

static protocolInitFnPtr protocolInit;
static protocolDataReceivedFnPtr protocolDataReceived;
static protocolProcessFrameFnPtr protocolProcessFrame;
static protocolSetRcDataFromPayloadFnPtr protocolSetRcDataFromPayload;

STATIC_UNIT_TESTED float rxSpiReadRawRC(const rxRuntimeState_t *rxRuntimeState, uint8_t channel)
{
    STATIC_ASSERT(NRF24L01_MAX_PAYLOAD_SIZE <= RX_SPI_MAX_PAYLOAD_SIZE, NRF24L01_MAX_PAYLOAD_SIZE_larger_than_RX_SPI_MAX_PAYLOAD_SIZE);

    if (channel >= rxRuntimeState->channelCount) {
        return 0;
    }
    if (rxSpiNewPacketAvailable) {
        protocolSetRcDataFromPayload(rxSpiRcData, rxSpiPayload);
        rxSpiNewPacketAvailable = false;
    }
    return rxSpiRcData[channel];
}

STATIC_UNIT_TESTED bool rxSpiSetProtocol(rx_spi_protocol_e protocol)
{
    switch (protocol) {
#ifdef USE_RX_V202
    case RX_SPI_NRF24_V202_250K:
    case RX_SPI_NRF24_V202_1M:
        protocolInit = v202Nrf24Init;
        protocolDataReceived = v202Nrf24DataReceived;
        protocolSetRcDataFromPayload = v202Nrf24SetRcDataFromPayload;
        break;
#endif
#ifdef USE_RX_SYMA
    case RX_SPI_NRF24_SYMA_X:
    case RX_SPI_NRF24_SYMA_X5C:
        protocolInit = symaNrf24Init;
        protocolDataReceived = symaNrf24DataReceived;
        protocolSetRcDataFromPayload = symaNrf24SetRcDataFromPayload;
        break;
#endif
#ifdef USE_RX_CX10
    case RX_SPI_NRF24_CX10:
    case RX_SPI_NRF24_CX10A:
        protocolInit = cx10Nrf24Init;
        protocolDataReceived = cx10Nrf24DataReceived;
        protocolSetRcDataFromPayload = cx10Nrf24SetRcDataFromPayload;
        break;
#endif
#ifdef USE_RX_H8_3D
    case RX_SPI_NRF24_H8_3D:
        protocolInit = h8_3dNrf24Init;
        protocolDataReceived = h8_3dNrf24DataReceived;
        protocolSetRcDataFromPayload = h8_3dNrf24SetRcDataFromPayload;
        break;
#endif
#ifdef USE_RX_KN
    case RX_SPI_NRF24_KN:
        protocolInit = knNrf24Init;
        protocolDataReceived = knNrf24DataReceived;
        protocolSetRcDataFromPayload = knNrf24SetRcDataFromPayload;
        break;
#endif
#ifdef USE_RX_INAV
    case RX_SPI_NRF24_INAV:
        protocolInit = inavNrf24Init;
        protocolDataReceived = inavNrf24DataReceived;
        protocolSetRcDataFromPayload = inavNrf24SetRcDataFromPayload;
        break;
#endif
#if defined(USE_RX_FRSKY_SPI)
#if defined(USE_RX_FRSKY_SPI_D)
    case RX_SPI_FRSKY_D:
#endif
#if defined(USE_RX_FRSKY_SPI_X)
    case RX_SPI_FRSKY_X:
    case RX_SPI_FRSKY_X_LBT:
    case RX_SPI_FRSKY_X_V2:
    case RX_SPI_FRSKY_X_LBT_V2:
        protocolInit = frSkySpiInit;
        protocolDataReceived = frSkySpiDataReceived;
        protocolSetRcDataFromPayload = frSkySpiSetRcData;
        protocolProcessFrame = frSkySpiProcessFrame;
        break;
#endif
#if defined(USE_RX_REDPINE_SPI)
    case RX_SPI_REDPINE:
        protocolInit = redpineSpiInit;
        protocolDataReceived = redpineSpiDataReceived;
        protocolSetRcDataFromPayload = redpineSetRcData;
        break;
#endif

#endif // USE_RX_FRSKY_SPI
#ifdef USE_RX_FLYSKY
    case RX_SPI_A7105_FLYSKY:
    case RX_SPI_A7105_FLYSKY_2A:
        protocolInit = flySkyInit;
        protocolDataReceived = flySkyDataReceived;
        protocolSetRcDataFromPayload = flySkySetRcDataFromPayload;
        break;
#endif
#ifdef USE_RX_SFHSS_SPI
    case RX_SPI_SFHSS:
        protocolInit = sfhssSpiInit;
        protocolDataReceived = sfhssSpiDataReceived;
        protocolSetRcDataFromPayload = sfhssSpiSetRcData;
        break;
#endif
#ifdef USE_RX_SPEKTRUM
    case RX_SPI_CYRF6936_DSM:
        protocolInit = spektrumSpiInit;
        protocolDataReceived = spektrumSpiDataReceived;
        protocolSetRcDataFromPayload = spektrumSpiSetRcDataFromPayload;
        break;
#endif
    default:
        return false;
    }

    return true;
}

/*
 * Returns true if the RX has received new data.
 * Called from updateRx in rx.c, updateRx called from taskUpdateRxCheck.
 * If taskUpdateRxCheck returns true, then taskUpdateRxMain will shortly be called.
 */
static uint8_t rxSpiFrameStatus(rxRuntimeState_t *rxRuntimeState)
{
    UNUSED(rxRuntimeState);

    uint8_t status = RX_FRAME_PENDING;

    rx_spi_received_e result = protocolDataReceived(rxSpiPayload);

    if (result & RX_SPI_RECEIVED_DATA) {
        rxSpiNewPacketAvailable = true;
        status = RX_FRAME_COMPLETE;
    }

    if (result & RX_SPI_ROCESSING_REQUIRED) {
        status |= RX_FRAME_PROCESSING_REQUIRED;
    }

    return status;
}

static bool rxSpiProcessFrame(const rxRuntimeState_t *rxRuntimeState)
{
    UNUSED(rxRuntimeState);

    if (protocolProcessFrame) {
        rx_spi_received_e result = protocolProcessFrame(rxSpiPayload);

        if (result & RX_SPI_RECEIVED_DATA) {
            rxSpiNewPacketAvailable = true;
        }

        if (result & RX_SPI_ROCESSING_REQUIRED) {
            return false;
        }
    }

    return true;
}

/*
 * Set and initialize the RX protocol
 */
bool rxSpiInit(const rxSpiConfig_t *rxSpiConfig, rxRuntimeState_t *rxRuntimeState)
{
    bool ret = false;

    if (!rxSpiDeviceInit(rxSpiConfig)) {
        return false;
    }

    if (!rxSpiSetProtocol(rxSpiConfig->rx_spi_protocol)) {
        return false;
    }

    rxSpiExtiConfig_t extiConfig = {
        .ioConfig = IOCFG_IN_FLOATING,
        .trigger = BETAFLIGHT_EXTI_TRIGGER_RISING,
    };

    ret = protocolInit(rxSpiConfig, rxRuntimeState, &extiConfig);

    if (rxSpiExtiConfigured()) {
        rxSpiExtiInit(extiConfig.ioConfig, extiConfig.trigger);

        rxRuntimeState->rcFrameTimeUsFn = rxSpiGetLastExtiTimeUs;
    }

    rxSpiNewPacketAvailable = false;
    rxRuntimeState->rxRefreshRate = 20000;

    rxRuntimeState->rcReadRawFn = rxSpiReadRawRC;
    rxRuntimeState->rcFrameStatusFn = rxSpiFrameStatus;
    rxRuntimeState->rcProcessFrameFn = rxSpiProcessFrame;

    return ret;
}
#endif
