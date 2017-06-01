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

/*
 * Authors:
 * Dominic Clifton/Hydra - Various cleanups for Cleanflight
 * Bill Nesbitt - Code from AutoQuad
 * Hamasaki/Timecop - Initial baseflight code
*/

#include <stdbool.h>
#include <stdint.h>

#include <platform.h>

#include "drivers/system.h"
#include "drivers/io.h"
#include "drivers/nvic.h"
#include "dma.h"
#include "rcc.h"

#include "serial.h"
#include "serial_uart.h"
#include "serial_uart_impl.h"

#ifdef USE_UART1
static uartPort_t uartPort1;
#endif

#ifdef USE_UART2
static uartPort_t uartPort2;
#endif

#ifdef USE_UART3
static uartPort_t uartPort3;
#endif

static void uartConfigIO(IO_t txIO, IO_t rxIO, portMode_t mode, portOptions_t options)
{
    if (options & SERIAL_BIDIR) {
        IOInit(txIO, OWNER_SERIAL_TX, 1);
        IOConfigGPIO(txIO, (options & SERIAL_BIDIR_PP) ? IOCFG_AF_PP : IOCFG_AF_OD);
    } else {
        if (mode & MODE_TX) {
            IOInit(txIO, OWNER_SERIAL_TX, 1);
            IOConfigGPIO(txIO, IOCFG_AF_PP);
        }

        if (mode & MODE_RX) {
            IOInit(rxIO, OWNER_SERIAL_RX, 1);
            IOConfigGPIO(rxIO, IOCFG_IPU);
        }
    }
}

static void uartConfigInterrupt(uint8_t irqn, int priority)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = irqn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(priority);
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(priority);
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

static void uartIrqCallback(uartPort_t *s)
{
    uint16_t SR = s->USARTx->SR;

    if (SR & USART_FLAG_RXNE && !s->rxDMAChannel) {
        // If we registered a callback, pass crap there
        if (s->port.rxCallback) {
            s->port.rxCallback(s->USARTx->DR);
        } else {
            s->port.rxBuffer[s->port.rxBufferHead++] = s->USARTx->DR;
            if (s->port.rxBufferHead >= s->port.rxBufferSize) {
                s->port.rxBufferHead = 0;
            }
        }
    }
    if (SR & USART_FLAG_TXE) {
        if (s->port.txBufferTail != s->port.txBufferHead) {
            s->USARTx->DR = s->port.txBuffer[s->port.txBufferTail++];
            if (s->port.txBufferTail >= s->port.txBufferSize) {
                s->port.txBufferTail = 0;
            }
        } else {
            USART_ITConfig(s->USARTx, USART_IT_TXE, DISABLE);
        }
    }
}

#ifdef USE_UART1
// USART1 Tx DMA Handler
void uart_tx_dma_IRQHandler(dmaChannelDescriptor_t* descriptor)
{
    uartPort_t *s = (uartPort_t*)(descriptor->userParam);
    DMA_CLEAR_FLAG(descriptor, DMA_IT_TCIF);
    DMA_Cmd(descriptor->ref, DISABLE);

    if (s->port.txBufferHead != s->port.txBufferTail)
        uartStartTxDMA(s);
    else
        s->txDMAEmpty = true;
}

// USART1 - Telemetry (RX/TX by DMA)
uartPort_t *serialUART1(uint32_t baudRate, portMode_t mode, portOptions_t options)
{
    uartPort_t *s;
    static volatile uint8_t rx1Buffer[UART1_RX_BUFFER_SIZE];
    static volatile uint8_t tx1Buffer[UART1_TX_BUFFER_SIZE];

    s = &uartPort1;
    s->port.vTable = uartVTable;

    s->port.baudRate = baudRate;

    s->port.rxBuffer = rx1Buffer;
    s->port.txBuffer = tx1Buffer;
    s->port.rxBufferSize = UART1_RX_BUFFER_SIZE;
    s->port.txBufferSize = UART1_TX_BUFFER_SIZE;

    s->USARTx = USART1;

#ifdef USE_UART1_RX_DMA
    dmaInit(DMA1_CH5_HANDLER, OWNER_SERIAL_RX, 1);
    s->rxDMAChannel = DMA1_Channel5;
    s->rxDMAPeripheralBaseAddr = (uint32_t)&s->USARTx->DR;
#endif
    s->txDMAChannel = DMA1_Channel4;
    s->txDMAPeripheralBaseAddr = (uint32_t)&s->USARTx->DR;

    RCC_ClockCmd(RCC_APB2(USART1), ENABLE);

    // UART1_TX    PA9
    // UART1_RX    PA10
    uartConfigIO(IOGetByTag(IO_TAG(PA9)), IOGetByTag(IO_TAG(PA10)), mode, options);

    // DMA TX Interrupt
    dmaInit(DMA1_CH4_HANDLER, OWNER_SERIAL_TX, 1);
    dmaSetHandler(DMA1_CH4_HANDLER, uart_tx_dma_IRQHandler, NVIC_PRIO_SERIALUART1_TXDMA, (uint32_t)&uartPort1);

#ifndef USE_UART1_RX_DMA
    // RX/TX Interrupt
    uartConfigInterrupt(USART1_IRQn, NVIC_PRIO_SERIALUART1);
#endif

    return s;
}

// USART1 Rx/Tx IRQ Handler
void USART1_IRQHandler(void)
{
    uartPort_t *s = &uartPort1;
    uartIrqCallback(s);
}

#endif

#ifdef USE_UART2
// USART2 - GPS or Spektrum or ?? (RX + TX by IRQ)
uartPort_t *serialUART2(uint32_t baudRate, portMode_t mode, portOptions_t options)
{
    uartPort_t *s;
    static volatile uint8_t rx2Buffer[UART2_RX_BUFFER_SIZE];
    static volatile uint8_t tx2Buffer[UART2_TX_BUFFER_SIZE];

    s = &uartPort2;
    s->port.vTable = uartVTable;

    s->port.baudRate = baudRate;

    s->port.rxBufferSize = UART2_RX_BUFFER_SIZE;
    s->port.txBufferSize = UART2_TX_BUFFER_SIZE;
    s->port.rxBuffer = rx2Buffer;
    s->port.txBuffer = tx2Buffer;

    s->USARTx = USART2;

    s->txDMAPeripheralBaseAddr = (uint32_t)&s->USARTx->DR;
    s->rxDMAPeripheralBaseAddr = (uint32_t)&s->USARTx->DR;

    RCC_ClockCmd(RCC_APB1(USART2), ENABLE);

    // UART2_TX    PA2
    // UART2_RX    PA3
    uartConfigIO(IOGetByTag(IO_TAG(PA2)), IOGetByTag(IO_TAG(PA3)), mode, options);

    // RX/TX Interrupt
    uartConfigInterrupt(USART2_IRQn, NVIC_PRIO_SERIALUART2);

    return s;
}


// USART2 Rx/Tx IRQ Handler
void USART2_IRQHandler(void)
{
    uartPort_t *s = &uartPort2;
    uartIrqCallback(s);
}

#endif

#ifdef USE_UART3
// USART3
uartPort_t *serialUART3(uint32_t baudRate, portMode_t mode, portOptions_t options)
{
    uartPort_t *s;
    static volatile uint8_t rx3Buffer[UART3_RX_BUFFER_SIZE];
    static volatile uint8_t tx3Buffer[UART3_TX_BUFFER_SIZE];

    s = &uartPort3;
    s->port.vTable = uartVTable;

    s->port.baudRate = baudRate;

    s->port.rxBuffer = rx3Buffer;
    s->port.txBuffer = tx3Buffer;
    s->port.rxBufferSize = UART3_RX_BUFFER_SIZE;
    s->port.txBufferSize = UART3_TX_BUFFER_SIZE;

    s->USARTx = USART3;

    s->txDMAPeripheralBaseAddr = (uint32_t)&s->USARTx->DR;
    s->rxDMAPeripheralBaseAddr = (uint32_t)&s->USARTx->DR;

    RCC_ClockCmd(RCC_APB1(USART3), ENABLE);

    uartConfigIO(IOGetByTag(IO_TAG(UART3_TX_PIN)), IOGetByTag(IO_TAG(UART3_RX_PIN)), mode, options);

    // RX/TX Interrupt
    uartConfigInterrupt(USART3_IRQn, NVIC_PRIO_SERIALUART3);

    return s;
}

// USART2 Rx/Tx IRQ Handler
void USART3_IRQHandler(void)
{
    uartPort_t *s = &uartPort3;
    uartIrqCallback(s);
}
#endif

// Temporary solution until serialUARTx() are refactored/consolidated

uartPort_t *serialUART(UARTDevice device, uint32_t baudRate, portMode_t mode, portOptions_t options)
{
    switch (device) {
#ifdef USE_UART1
    case UARTDEV_1:
        return serialUART1(baudRate, mode, options);
#endif
#ifdef USE_UART2
    case UARTDEV_2:
        return serialUART2(baudRate, mode, options);
#endif
#ifdef USE_UART3
    case UARTDEV_3:
        return serialUART3(baudRate, mode, options);
#endif
    default:
        return NULL;
    }
}
