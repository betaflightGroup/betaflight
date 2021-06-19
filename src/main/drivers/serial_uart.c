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
 * Authors:
 * jflyper - Refactoring, cleanup and made pin-configurable
 * Dominic Clifton - Serial port abstraction, Separation of common STM32 code for cleanflight, various cleanups.
 * Hamasaki/Timecop - Initial baseflight code
*/

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_UART

#include "build/build_config.h"

#include "common/utils.h"

#include "drivers/dma.h"
#include "drivers/dma_reqmap.h"
#include "drivers/rcc.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/serial_uart_impl.h"

#include "pg/serial_uart.h"

#if defined(STM32H7)
#define UART_TX_BUFFER_ATTRIBUTE DMA_RAM            // D2 SRAM
#define UART_RX_BUFFER_ATTRIBUTE DMA_RAM            // D2 SRAM
#elif defined(STM32G4)
#define UART_TX_BUFFER_ATTRIBUTE DMA_RAM_W          // SRAM MPU NOT_BUFFERABLE
#define UART_RX_BUFFER_ATTRIBUTE DMA_RAM_R          // SRAM MPU NOT CACHABLE
#elif defined(STM32F7)
#define UART_TX_BUFFER_ATTRIBUTE FAST_DATA_ZERO_INIT // DTCM RAM
#define UART_RX_BUFFER_ATTRIBUTE FAST_DATA_ZERO_INIT // DTCM RAM
#elif defined(STM32F4) || defined(STM32F3) || defined(STM32F1)
#define UART_TX_BUFFER_ATTRIBUTE                    // NONE
#define UART_RX_BUFFER_ATTRIBUTE                    // NONE
#else
#error Undefined UART_{TX,RX}_BUFFER_ATTRIBUTE for this MCU
#endif

#define UART_BUFFERS(n) \
    UART_BUFFER(UART_TX_BUFFER_ATTRIBUTE, n, T); \
    UART_BUFFER(UART_RX_BUFFER_ATTRIBUTE, n, R); struct dummy_s

#define LPUART_BUFFERS(n) \
    LPUART_BUFFER(UART_TX_BUFFER_ATTRIBUTE, n, T); \
    LPUART_BUFFER(UART_RX_BUFFER_ATTRIBUTE, n, R); struct dummy_s

#ifdef USE_UART1
UART_BUFFERS(1);
#endif

#ifdef USE_UART2
UART_BUFFERS(2);
#endif

#ifdef USE_UART3
UART_BUFFERS(3);
#endif

#ifdef USE_UART4
UART_BUFFERS(4);
#endif

#ifdef USE_UART5
UART_BUFFERS(5);
#endif

#ifdef USE_UART6
UART_BUFFERS(6);
#endif

#ifdef USE_UART7
UART_BUFFERS(7);
#endif

#ifdef USE_UART8
UART_BUFFERS(8);
#endif

#ifdef USE_UART9
UART_BUFFERS(9);
#endif

#ifdef USE_UART10
UART_BUFFERS(10);
#endif

#ifdef USE_LPUART1
LPUART_BUFFERS(1);
#endif

#undef UART_BUFFERS

serialPort_t *uartOpen(UARTDevice_e device, serialReceiveCallbackPtr rxCallback, void *rxCallbackData, uint32_t baudRate, portMode_e mode, portOptions_e options)
{
    uartPort_t *s = serialUART(device, baudRate, mode, options);

    if (!s)
        return (serialPort_t *)s;

#ifdef USE_DMA
    s->txDMAEmpty = true;
#endif

    // common serial initialisation code should move to serialPort::init()
    s->port.rxBufferHead = s->port.rxBufferTail = 0;
    s->port.txBufferHead = s->port.txBufferTail = 0;
    // callback works for IRQ-based RX ONLY
    s->port.rxCallback = rxCallback;
    s->port.rxCallbackData = rxCallbackData;
    s->port.mode = mode;
    s->port.baudRate = baudRate;
    s->port.options = options;

    uartReconfigure(s);

    return (serialPort_t *)s;
}

static void uartSetBaudRate(serialPort_t *instance, uint32_t baudRate)
{
    uartPort_t *uartPort = (uartPort_t *)instance;
    uartPort->port.baudRate = baudRate;
    uartReconfigure(uartPort);
}

static void uartSetMode(serialPort_t *instance, portMode_e mode)
{
    uartPort_t *uartPort = (uartPort_t *)instance;
    uartPort->port.mode = mode;
    uartReconfigure(uartPort);
}

static uint32_t uartTotalRxBytesWaiting(const serialPort_t *instance)
{
    const uartPort_t *s = (const uartPort_t*)instance;

#ifdef USE_DMA
    if (s->rxDMAResource) {
        // XXX Could be consolidated
#ifdef USE_HAL_DRIVER
        uint32_t rxDMAHead = __HAL_DMA_GET_COUNTER(s->Handle.hdmarx);
#else
        uint32_t rxDMAHead = xDMA_GetCurrDataCounter(s->rxDMAResource);
#endif

        // s->rxDMAPos and rxDMAHead represent distances from the end
        // of the buffer.  They count DOWN as they advance.
        if (s->rxDMAPos >= rxDMAHead) {
            return s->rxDMAPos - rxDMAHead;
        } else {
            return s->port.rxBufferSize + s->rxDMAPos - rxDMAHead;
        }
    }
#endif

    if (s->port.rxBufferHead >= s->port.rxBufferTail) {
        return s->port.rxBufferHead - s->port.rxBufferTail;
    } else {
        return s->port.rxBufferSize + s->port.rxBufferHead - s->port.rxBufferTail;
    }
}

static uint32_t uartTotalTxBytesFree(const serialPort_t *instance)
{
    const uartPort_t *s = (const uartPort_t*)instance;

    uint32_t bytesUsed;

    if (s->port.txBufferHead >= s->port.txBufferTail) {
        bytesUsed = s->port.txBufferHead - s->port.txBufferTail;
    } else {
        bytesUsed = s->port.txBufferSize + s->port.txBufferHead - s->port.txBufferTail;
    }

#ifdef USE_DMA
    if (s->txDMAResource) {
        /*
         * When we queue up a DMA request, we advance the Tx buffer tail before the transfer finishes, so we must add
         * the remaining size of that in-progress transfer here instead:
         */
#ifdef USE_HAL_DRIVER
        bytesUsed += __HAL_DMA_GET_COUNTER(s->Handle.hdmatx);
#else
        bytesUsed += xDMA_GetCurrDataCounter(s->txDMAResource);
#endif

        /*
         * If the Tx buffer is being written to very quickly, we might have advanced the head into the buffer
         * space occupied by the current DMA transfer. In that case the "bytesUsed" total will actually end up larger
         * than the total Tx buffer size, because we'll end up transmitting the same buffer region twice. (So we'll be
         * transmitting a garbage mixture of old and new bytes).
         *
         * Be kind to callers and pretend like our buffer can only ever be 100% full.
         */
        if (bytesUsed >= s->port.txBufferSize - 1) {
            return 0;
        }
    }
#endif

    return (s->port.txBufferSize - 1) - bytesUsed;
}

static bool isUartTransmitBufferEmpty(const serialPort_t *instance)
{
    const uartPort_t *s = (const uartPort_t *)instance;
#ifdef USE_DMA
    if (s->txDMAResource) {
        return s->txDMAEmpty;
    } else
#endif
    {
        return s->port.txBufferTail == s->port.txBufferHead;
    }
}

static uint8_t uartRead(serialPort_t *instance)
{
    uint8_t ch;
    uartPort_t *s = (uartPort_t *)instance;

#ifdef USE_DMA
    if (s->rxDMAResource) {
        ch = s->port.rxBuffer[s->port.rxBufferSize - s->rxDMAPos];
        if (--s->rxDMAPos == 0)
            s->rxDMAPos = s->port.rxBufferSize;
    } else
#endif
    {
        ch = s->port.rxBuffer[s->port.rxBufferTail];
        if (s->port.rxBufferTail + 1 >= s->port.rxBufferSize) {
            s->port.rxBufferTail = 0;
        } else {
            s->port.rxBufferTail++;
        }
    }

    return ch;
}

static void uartWrite(serialPort_t *instance, uint8_t ch)
{
    uartPort_t *s = (uartPort_t *)instance;

    s->port.txBuffer[s->port.txBufferHead] = ch;

    if (s->port.txBufferHead + 1 >= s->port.txBufferSize) {
        s->port.txBufferHead = 0;
    } else {
        s->port.txBufferHead++;
    }

#ifdef USE_DMA
    if (s->txDMAResource) {
        uartTryStartTxDMA(s);
    } else
#endif
    {
#ifdef USE_HAL_DRIVER
        __HAL_UART_ENABLE_IT(&s->Handle, UART_IT_TXE);
#else
        USART_ITConfig(s->USARTx, USART_IT_TXE, ENABLE);
#endif
    }
}

const struct serialPortVTable uartVTable[] = {
    {
        .serialWrite = uartWrite,
        .serialTotalRxWaiting = uartTotalRxBytesWaiting,
        .serialTotalTxFree = uartTotalTxBytesFree,
        .serialRead = uartRead,
        .serialSetBaudRate = uartSetBaudRate,
        .isSerialTransmitBufferEmpty = isUartTransmitBufferEmpty,
        .setMode = uartSetMode,
        .setCtrlLineStateCb = NULL,
        .setBaudRateCb = NULL,
        .writeBuf = NULL,
        .beginWrite = NULL,
        .endWrite = NULL,
    }
};

#ifdef USE_DMA
void uartConfigureDma(uartDevice_t *uartdev)
{
    uartPort_t *s = &(uartdev->port);
    const uartHardware_t *hardware = uartdev->hardware;

#ifdef USE_DMA_SPEC
    UARTDevice_e device = hardware->device;
    const dmaChannelSpec_t *dmaChannelSpec;

    if (serialUartConfig(device)->txDmaopt != DMA_OPT_UNUSED) {
        dmaChannelSpec = dmaGetChannelSpecByPeripheral(DMA_PERIPH_UART_TX, device, serialUartConfig(device)->txDmaopt);
        if (dmaChannelSpec) {
            s->txDMAResource = dmaChannelSpec->ref;
            s->txDMAChannel = dmaChannelSpec->channel;
        }
    }

    if (serialUartConfig(device)->rxDmaopt != DMA_OPT_UNUSED) {
        dmaChannelSpec = dmaGetChannelSpecByPeripheral(DMA_PERIPH_UART_RX, device, serialUartConfig(device)->txDmaopt);
        if (dmaChannelSpec) {
            s->rxDMAResource = dmaChannelSpec->ref;
            s->rxDMAChannel = dmaChannelSpec->channel;
        }
    }
#else
    // Non USE_DMA_SPEC does not support configurable ON/OFF of UART DMA

    if (hardware->rxDMAResource) {
        s->rxDMAResource = hardware->rxDMAResource;
        s->rxDMAChannel = hardware->rxDMAChannel;
    }

    if (hardware->txDMAResource) {
        s->txDMAResource = hardware->txDMAResource;
        s->txDMAChannel = hardware->txDMAChannel;
    }
#endif

    if (s->txDMAResource) {
        dmaIdentifier_e identifier = dmaGetIdentifier(s->txDMAResource);
        dmaInit(identifier, OWNER_SERIAL_TX, RESOURCE_INDEX(hardware->device));
        dmaSetHandler(identifier, uartDmaIrqHandler, hardware->txPriority, (uint32_t)uartdev);
        s->txDMAPeripheralBaseAddr = (uint32_t)&UART_REG_TXD(hardware->reg);
    }

    if (s->rxDMAResource) {
        dmaIdentifier_e identifier = dmaGetIdentifier(s->rxDMAResource);
        dmaInit(identifier, OWNER_SERIAL_RX, RESOURCE_INDEX(hardware->device));
        s->rxDMAPeripheralBaseAddr = (uint32_t)&UART_REG_RXD(hardware->reg);
    }
}
#endif

#define UART_IRQHandler(type, number, dev)                    \
    void type ## number ## _IRQHandler(void)                  \
    {                                                         \
        uartPort_t *s = &(uartDevmap[UARTDEV_ ## dev]->port); \
        uartIrqHandler(s);                                    \
    }

#ifdef USE_UART1
UART_IRQHandler(USART, 1, 1) // USART1 Rx/Tx IRQ Handler
#endif

#ifdef USE_UART2
UART_IRQHandler(USART, 2, 2) // USART2 Rx/Tx IRQ Handler
#endif

#ifdef USE_UART3
UART_IRQHandler(USART, 3, 3) // USART3 Rx/Tx IRQ Handler
#endif

#ifdef USE_UART4
UART_IRQHandler(UART, 4, 4)  // UART4 Rx/Tx IRQ Handler
#endif

#ifdef USE_UART5
UART_IRQHandler(UART, 5, 5)  // UART5 Rx/Tx IRQ Handler
#endif

#ifdef USE_UART6
UART_IRQHandler(USART, 6, 6) // USART6 Rx/Tx IRQ Handler
#endif

#ifdef USE_UART7
UART_IRQHandler(UART, 7, 7)  // UART7 Rx/Tx IRQ Handler
#endif

#ifdef USE_UART8
UART_IRQHandler(UART, 8, 8)  // UART8 Rx/Tx IRQ Handler
#endif

#ifdef USE_UART9
UART_IRQHandler(LPUART, 1, 9) // UART9 (implemented with LPUART1) Rx/Tx IRQ Handler
#endif

#endif // USE_UART
