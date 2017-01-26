/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "build/build_config.h"

#include "common/utils.h"

#include "config/config_master.h"

#include "drivers/system.h"
#include "drivers/serial.h"
#if defined(USE_SOFTSERIAL1) || defined(USE_SOFTSERIAL2)
#include "drivers/serial_softserial.h"
#endif

#if defined(USE_UART1) || defined(USE_UART2) || defined(USE_UART3) || defined(USE_UART4) || defined(USE_UART5) || defined(USE_UART6)
#include "drivers/serial_uart.h"
#endif

#include "drivers/light_led.h"

#if defined(USE_VCP)
#include "drivers/serial_usb_vcp.h"
#endif

#include "io/serial.h"
#include "fc/cli.h" // for cliEnter()

#include "msp/msp_serial.h"

#ifdef TELEMETRY
#include "telemetry/telemetry.h"
#endif

static serialConfig_t *serialConfig;
static serialPortUsage_t serialPortUsageList[SERIAL_PORT_COUNT];

const serialPortIdentifier_e serialPortIdentifiers[SERIAL_PORT_COUNT] = {
#ifdef USE_VCP
    SERIAL_PORT_USB_VCP,
#endif
#ifdef USE_UART1
    SERIAL_PORT_USART1,
#endif
#ifdef USE_UART2
    SERIAL_PORT_USART2,
#endif
#ifdef USE_UART3
    SERIAL_PORT_USART3,
#endif
#ifdef USE_UART4
    SERIAL_PORT_USART4,
#endif
#ifdef USE_UART5
    SERIAL_PORT_USART5,
#endif
#ifdef USE_UART6
    SERIAL_PORT_USART6,
#endif
#ifdef USE_UART7
    SERIAL_PORT_USART7,
#endif
#ifdef USE_UART8
    SERIAL_PORT_USART8,
#endif
#ifdef USE_SOFTSERIAL1
    SERIAL_PORT_SOFTSERIAL1,
#endif
#ifdef USE_SOFTSERIAL2
    SERIAL_PORT_SOFTSERIAL2,
#endif
};

static uint8_t serialPortCount;

// Default pins (NONE).
// XXX Does this mess belong here??? serial_default.h ???
#ifdef USE_UART1
# if !defined(UART1_RX_PIN)
#  define UART1_RX_PIN NONE
# endif
# if !defined(UART1_TX_PIN)
#  define UART1_TX_PIN NONE
# endif
#endif

#ifdef USE_UART2
# if !defined(UART2_RX_PIN)
#  define UART2_RX_PIN NONE
# endif
# if !defined(UART2_TX_PIN)
#  define UART2_TX_PIN NONE
# endif
#endif

#ifdef USE_UART3
# if !defined(UART3_RX_PIN)
#  define UART3_RX_PIN NONE
# endif
# if !defined(UART3_TX_PIN)
#  define UART3_TX_PIN NONE
# endif
#endif

#ifdef USE_UART4
# if !defined(UART4_RX_PIN)
#  define UART4_RX_PIN NONE
# endif
# if !defined(UART4_TX_PIN)
#  define UART4_TX_PIN NONE
# endif
#endif

#ifdef USE_UART5
# if !defined(UART5_RX_PIN)
#  define UART5_RX_PIN NONE
# endif
# if !defined(UART5_TX_PIN)
#  define UART5_TX_PIN NONE
# endif
#endif

#ifdef USE_UART6
# if !defined(UART6_RX_PIN)
#  define UART6_RX_PIN NONE
# endif
# if !defined(UART6_TX_PIN)
#  define UART6_TX_PIN NONE
# endif
#endif

#ifdef USE_UART7
# if !defined(UART7_RX_PIN)
#  define UART7_RX_PIN NONE
# endif
# if !defined(UART7_TX_PIN)
#  define UART7_TX_PIN NONE
# endif
#endif

#ifdef USE_UART8
# if !defined(UART8_RX_PIN)
#  define UART8_RX_PIN NONE
# endif
# if !defined(UART8_TX_PIN)
#  define UART8_TX_PIN NONE
# endif
#endif

#ifdef USE_SOFTSERIAL1
# if !defined(SOFTSERIAL1_RX_PIN)
#  define SOFTSERIAL1_RX_PIN NONE
# endif
# if !defined(SOFTSERIAL1_TX_PIN)
#  define SOFTSERIAL1_TX_PIN NONE
# endif
#endif

#ifdef USE_SOFTSERIAL2
# if !defined(SOFTSERIAL2_RX_PIN)
#  define SOFTSERIAL2_RX_PIN NONE
# endif
# if !defined(SOFTSERIAL2_TX_PIN)
#  define SOFTSERIAL2_TX_PIN NONE
# endif
#endif

// XXX For compatibility; reset pin configuration as specified in target.h.
// XXX Does this belong here? Driver layer???

typedef struct serialPinDefault_s {
    serialPortIdentifier_e portId;
    ioTag_t rx, tx;
} serialPinDefault_t;

static serialPinDefault_t serialPinDefault[] = {
#ifdef USE_UART1
    { SERIAL_PORT_USART1, IO_TAG(UART1_RX_PIN), IO_TAG(UART1_TX_PIN) },
#endif
#ifdef USE_UART2
    { SERIAL_PORT_USART2, IO_TAG(UART2_RX_PIN), IO_TAG(UART2_TX_PIN) },
#endif
#ifdef USE_UART3
    { SERIAL_PORT_USART3, IO_TAG(UART3_RX_PIN), IO_TAG(UART3_TX_PIN) },
#endif
#ifdef USE_UART4
    { SERIAL_PORT_USART4, IO_TAG(UART4_RX_PIN), IO_TAG(UART4_TX_PIN) },
#endif
#ifdef USE_UART5
    { SERIAL_PORT_USART5, IO_TAG(UART5_RX_PIN), IO_TAG(UART5_TX_PIN) },
#endif
#ifdef USE_UART6
    { SERIAL_PORT_USART6, IO_TAG(UART6_RX_PIN), IO_TAG(UART6_TX_PIN) },
#endif
#ifdef USE_UART7
    { SERIAL_PORT_USART7, IO_TAG(UART7_RX_PIN), IO_TAG(UART7_TX_PIN) },
#endif
#ifdef USE_UART8
    { SERIAL_PORT_USART8, IO_TAG(UART8_RX_PIN), IO_TAG(UART8_TX_PIN) },
#endif
#ifdef USE_SOFTSERIAL1
    { SERIAL_PORT_SOFTSERIAL1, IO_TAG(SOFTSERIAL1_RX_PIN), IO_TAG(SOFTSERIAL1_TX_PIN) },
#endif
#ifdef USE_SOFTSERIAL2
    { SERIAL_PORT_SOFTSERIAL2, IO_TAG(SOFTSERIAL2_RX_PIN), IO_TAG(SOFTSERIAL2_TX_PIN) },
#endif
};

void serialPinConfigReset(serialPinConfig_t *pSerialPinConfig)
{

    for (int port = 0 ; port < SERIAL_PORT_MAX_COUNT ; port++) {
        pSerialPinConfig->ioTagRx[port] = IO_TAG(NONE);
        pSerialPinConfig->ioTagTx[port] = IO_TAG(NONE);
    }

    for (size_t port = 0 ; port < ARRAYLEN(serialPinDefault) ; port++) {
        serialPinDefault_t *pDefault = &serialPinDefault[port];
            pSerialPinConfig->ioTagRx[SERIAL_PORT_IDENTIFIER_TO_RESOURCE_INDEX(pDefault->portId)] = pDefault->rx;
            pSerialPinConfig->ioTagTx[SERIAL_PORT_IDENTIFIER_TO_RESOURCE_INDEX(pDefault->portId)] = pDefault->tx;
    }
}

const uint32_t baudRates[] = {0, 9600, 19200, 38400, 57600, 115200, 230400, 250000,
        400000, 460800, 500000, 921600, 1000000, 1500000, 2000000, 2470000}; // see baudRate_e

#define BAUD_RATE_COUNT (sizeof(baudRates) / sizeof(baudRates[0]))

baudRate_e lookupBaudRateIndex(uint32_t baudRate)
{
    uint8_t index;

    for (index = 0; index < BAUD_RATE_COUNT; index++) {
        if (baudRates[index] == baudRate) {
            return index;
        }
    }
    return BAUD_AUTO;
}

int findSerialPortIndexByIdentifier(serialPortIdentifier_e identifier)
{
    for (int index = 0; index < SERIAL_PORT_COUNT; index++) {
        if (serialPortIdentifiers[index] == identifier) {
            return index;
        }
    }
    return -1;
}

serialPortUsage_t *findSerialPortUsageByIdentifier(serialPortIdentifier_e identifier)
{
    uint8_t index;
    for (index = 0; index < SERIAL_PORT_COUNT; index++) {
        serialPortUsage_t *candidate = &serialPortUsageList[index];
        if (candidate->identifier == identifier) {
            return candidate;
        }
    }
    return NULL;
}

serialPortUsage_t *findSerialPortUsageByPort(serialPort_t *serialPort) {
    uint8_t index;
    for (index = 0; index < SERIAL_PORT_COUNT; index++) {
        serialPortUsage_t *candidate = &serialPortUsageList[index];
        if (candidate->serialPort == serialPort) {
            return candidate;
        }
    }
    return NULL;
}

typedef struct findSerialPortConfigState_s {
    uint8_t lastIndex;
} findSerialPortConfigState_t;

static findSerialPortConfigState_t findSerialPortConfigState;

serialPortConfig_t *findSerialPortConfig(serialPortFunction_e function)
{
    memset(&findSerialPortConfigState, 0, sizeof(findSerialPortConfigState));

    return findNextSerialPortConfig(function);
}

serialPortConfig_t *findNextSerialPortConfig(serialPortFunction_e function)
{
    while (findSerialPortConfigState.lastIndex < SERIAL_PORT_COUNT) {
        serialPortConfig_t *candidate = &serialConfig->portConfigs[findSerialPortConfigState.lastIndex++];

        if (candidate->functionMask & function) {
            return candidate;
        }
    }
    return NULL;
}

typedef struct findSharedSerialPortState_s {
    uint8_t lastIndex;
} findSharedSerialPortState_t;

portSharing_e determinePortSharing(serialPortConfig_t *portConfig, serialPortFunction_e function)
{
    if (!portConfig || (portConfig->functionMask & function) == 0) {
        return PORTSHARING_UNUSED;
    }
    return portConfig->functionMask == function ? PORTSHARING_NOT_SHARED : PORTSHARING_SHARED;
}

bool isSerialPortShared(serialPortConfig_t *portConfig, uint16_t functionMask, serialPortFunction_e sharedWithFunction)
{
    return (portConfig) && (portConfig->functionMask & sharedWithFunction) && (portConfig->functionMask & functionMask);
}

static findSharedSerialPortState_t findSharedSerialPortState;

serialPort_t *findSharedSerialPort(uint16_t functionMask, serialPortFunction_e sharedWithFunction)
{
    memset(&findSharedSerialPortState, 0, sizeof(findSharedSerialPortState));

    return findNextSharedSerialPort(functionMask, sharedWithFunction);
}

serialPort_t *findNextSharedSerialPort(uint16_t functionMask, serialPortFunction_e sharedWithFunction)
{
    while (findSharedSerialPortState.lastIndex < SERIAL_PORT_COUNT) {
        serialPortConfig_t *candidate = &serialConfig->portConfigs[findSharedSerialPortState.lastIndex++];

        if (isSerialPortShared(candidate, functionMask, sharedWithFunction)) {
            serialPortUsage_t *serialPortUsage = findSerialPortUsageByIdentifier(candidate->identifier);
            if (!serialPortUsage) {
                continue;
            }
            return serialPortUsage->serialPort;
        }
    }
    return NULL;
}

#ifdef TELEMETRY
#define ALL_TELEMETRY_FUNCTIONS_MASK (TELEMETRY_SHAREABLE_PORT_FUNCTIONS_MASK | FUNCTION_TELEMETRY_HOTT | FUNCTION_TELEMETRY_SMARTPORT)
#define ALL_FUNCTIONS_SHARABLE_WITH_MSP (FUNCTION_BLACKBOX | ALL_TELEMETRY_FUNCTIONS_MASK)
#else
#define ALL_FUNCTIONS_SHARABLE_WITH_MSP (FUNCTION_BLACKBOX)
#endif

bool isSerialConfigValid(serialConfig_t *serialConfigToCheck)
{
    UNUSED(serialConfigToCheck);
    /*
     * rules:
     * - 1 MSP port minimum, max MSP ports is defined and must be adhered to.
     * - MSP is allowed to be shared with EITHER any telemetry OR blackbox.
     *   (using either / or, switching based on armed / disarmed or an AUX channel if 'telemetry_switch' is true
     * - serial RX and FrSky / LTM / MAVLink telemetry can be shared
     *   (serial RX using RX line, telemetry using TX line)
     * - No other sharing combinations are valid.
     */
    uint8_t mspPortCount = 0;

    uint8_t index;
    for (index = 0; index < SERIAL_PORT_COUNT; index++) {
        serialPortConfig_t *portConfig = &serialConfigToCheck->portConfigs[index];

        if (portConfig->functionMask & FUNCTION_MSP) {
            mspPortCount++;
        }

        uint8_t bitCount = BITCOUNT(portConfig->functionMask);
        if (bitCount > 1) {
            // shared
            if (bitCount > 2) {
                return false;
            }

            if ((portConfig->functionMask & FUNCTION_MSP) && (portConfig->functionMask & ALL_FUNCTIONS_SHARABLE_WITH_MSP)) {
                // MSP & telemetry
#ifdef TELEMETRY
            } else if (telemetryCheckRxPortShared(portConfig)) {
                // serial RX & telemetry
#endif
            } else {
                // some other combination
                return false;
            }
        }
    }

    if (mspPortCount == 0 || mspPortCount > MAX_MSP_PORT_COUNT) {
        return false;
    }
    return true;
}

serialPortConfig_t *serialFindPortConfiguration(serialPortIdentifier_e identifier)
{
    uint8_t index;
    for (index = 0; index < SERIAL_PORT_COUNT; index++) {
        serialPortConfig_t *candidate = &serialConfig->portConfigs[index];
        if (candidate->identifier == identifier) {
            return candidate;
        }
    }
    return NULL;
}

bool doesConfigurationUsePort(serialPortIdentifier_e identifier)
{
    serialPortConfig_t *candidate = serialFindPortConfiguration(identifier);
    return candidate != NULL && candidate->functionMask;
}

serialPort_t *openSerialPort(
    serialPortIdentifier_e identifier,
    serialPortFunction_e function,
    serialReceiveCallbackPtr rxCallback,
    uint32_t baudRate,
    portMode_t mode,
    portOptions_t options)
{
#if (!defined(USE_UART1) && !defined(USE_UART2) && !defined(USE_UART3) && !defined(USE_UART4) && !defined(USE_UART5) && !defined(USE_UART6) && !defined(USE_SOFTSERIAL1) && !defined(USE_SOFTSERIAL2))
    UNUSED(rxCallback);
    UNUSED(baudRate);
    UNUSED(mode);
    UNUSED(options);
#endif

    serialPortUsage_t *serialPortUsage = findSerialPortUsageByIdentifier(identifier);
    if (!serialPortUsage || serialPortUsage->function != FUNCTION_NONE) {
        // not available / already in use
        return NULL;
    }

    serialPort_t *serialPort = NULL;

    switch(identifier) {
#ifdef USE_VCP
        case SERIAL_PORT_USB_VCP:
            serialPort = usbVcpOpen();
            break;
#endif
#ifdef USE_UART1
        case SERIAL_PORT_USART1:
            serialPort = uartOpen(USART1, rxCallback, baudRate, mode, options);
            break;
#endif
#ifdef USE_UART2
        case SERIAL_PORT_USART2:
            serialPort = uartOpen(USART2, rxCallback, baudRate, mode, options);
            break;
#endif
#ifdef USE_UART3
        case SERIAL_PORT_USART3:
            serialPort = uartOpen(USART3, rxCallback, baudRate, mode, options);
            break;
#endif
#ifdef USE_UART4
        case SERIAL_PORT_USART4:
            serialPort = uartOpen(UART4, rxCallback, baudRate, mode, options);
            break;
#endif
#ifdef USE_UART5
        case SERIAL_PORT_USART5:
            serialPort = uartOpen(UART5, rxCallback, baudRate, mode, options);
            break;
#endif
#ifdef USE_UART6
        case SERIAL_PORT_USART6:
            serialPort = uartOpen(USART6, rxCallback, baudRate, mode, options);
            break;
#endif
#ifdef USE_UART7
        case SERIAL_PORT_USART7:
            serialPort = uartOpen(UART7, rxCallback, baudRate, mode, options);
            break;
#endif
#ifdef USE_UART8
        case SERIAL_PORT_USART8:
            serialPort = uartOpen(UART8, rxCallback, baudRate, mode, options);
            break;
#endif
#ifdef USE_SOFTSERIAL1
        case SERIAL_PORT_SOFTSERIAL1:
            serialPort = openSoftSerial(SOFTSERIAL1, rxCallback, baudRate, mode, options);
            break;
#endif
#ifdef USE_SOFTSERIAL2
        case SERIAL_PORT_SOFTSERIAL2:
            serialPort = openSoftSerial(SOFTSERIAL2, rxCallback, baudRate, mode, options);
            break;
#endif
        default:
            break;
    }

    if (!serialPort) {
        return NULL;
    }

    serialPort->identifier = identifier;

    serialPortUsage->function = function;
    serialPortUsage->serialPort = serialPort;

    return serialPort;
}

void closeSerialPort(serialPort_t *serialPort)
{
    serialPortUsage_t *serialPortUsage = findSerialPortUsageByPort(serialPort);
    if (!serialPortUsage) {
        // already closed
        return;
    }

    // TODO wait until data has been transmitted.

    serialPort->rxCallback = NULL;

    serialPortUsage->function = FUNCTION_NONE;
    serialPortUsage->serialPort = NULL;
}

void serialInit(serialConfig_t *initialSerialConfig, bool softserialEnabled, serialPortIdentifier_e serialPortToDisable)
{
    uint8_t index;

    serialConfig = initialSerialConfig;

    serialPortCount = SERIAL_PORT_COUNT;
    memset(&serialPortUsageList, 0, sizeof(serialPortUsageList));

    for (index = 0; index < SERIAL_PORT_COUNT; index++) {
        serialPortUsageList[index].identifier = serialPortIdentifiers[index];

        if (serialPortToDisable != SERIAL_PORT_NONE) {
            if (serialPortUsageList[index].identifier == serialPortToDisable) {
                serialPortUsageList[index].identifier = SERIAL_PORT_NONE;
                serialPortCount--;
            }
        }
        if (!softserialEnabled) {
            if (0
#ifdef USE_SOFTSERIAL1
                || serialPortUsageList[index].identifier == SERIAL_PORT_SOFTSERIAL1
#endif
#ifdef USE_SOFTSERIAL2
                || serialPortUsageList[index].identifier == SERIAL_PORT_SOFTSERIAL2
#endif
            ) {
                serialPortUsageList[index].identifier = SERIAL_PORT_NONE;
                serialPortCount--;
            }
        }
    }
}

void serialRemovePort(serialPortIdentifier_e identifier)
{
    for (uint8_t index = 0; index < SERIAL_PORT_COUNT; index++) {
        if (serialPortUsageList[index].identifier == identifier) {
            serialPortUsageList[index].identifier = SERIAL_PORT_NONE;
            serialPortCount--;
        }
    }
}

uint8_t serialGetAvailablePortCount(void)
{
    return serialPortCount;
}

bool serialIsPortAvailable(serialPortIdentifier_e identifier)
{
    for (uint8_t index = 0; index < SERIAL_PORT_COUNT; index++) {
        if (serialPortUsageList[index].identifier == identifier) {
            return true;
        }
    }
    return false;
}

void waitForSerialPortToFinishTransmitting(serialPort_t *serialPort)
{
    while (!isSerialTransmitBufferEmpty(serialPort)) {
        delay(10);
    };
}

void serialEvaluateNonMspData(serialPort_t *serialPort, uint8_t receivedChar)
{
#ifndef USE_CLI
    UNUSED(serialPort);
#else
    if (receivedChar == '#') {
        cliEnter(serialPort);
    }
#endif
    if (receivedChar == serialConfig->reboot_character) {
        systemResetToBootloader();
    }
}

#if defined(GPS) || ! defined(SKIP_SERIAL_PASSTHROUGH)
// Default data consumer for serialPassThrough.
static void nopConsumer(uint8_t data)
{
    UNUSED(data);
}

/*
 A high-level serial passthrough implementation. Used by cli to start an
 arbitrary serial passthrough "proxy". Optional callbacks can be given to allow
 for specialized data processing.
 */
void serialPassthrough(serialPort_t *left, serialPort_t *right, serialConsumer
                       *leftC, serialConsumer *rightC)
{
    waitForSerialPortToFinishTransmitting(left);
    waitForSerialPortToFinishTransmitting(right);

    if (!leftC)
        leftC = &nopConsumer;
    if (!rightC)
        rightC = &nopConsumer;

    LED0_OFF;
    LED1_OFF;

    // Either port might be open in a mode other than MODE_RXTX. We rely on
    // serialRxBytesWaiting() to do the right thing for a TX only port. No
    // special handling is necessary OR performed.
    while(1) {
        // TODO: maintain a timestamp of last data received. Use this to
        // implement a guard interval and check for `+++` as an escape sequence
        // to return to CLI command mode.
        // https://en.wikipedia.org/wiki/Escape_sequence#Modem_control
        if (serialRxBytesWaiting(left)) {
            LED0_ON;
            uint8_t c = serialRead(left);
            serialWrite(right, c);
            leftC(c);
            LED0_OFF;
         }
         if (serialRxBytesWaiting(right)) {
             LED0_ON;
             uint8_t c = serialRead(right);
             serialWrite(left, c);
             rightC(c);
             LED0_OFF;
         }
     }
 }
 #endif
