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
#include <stdlib.h>

#include "platform.h"

#ifdef USE_UART

#include "build/build_config.h"
#include "build/atomic.h"

#include "common/utils.h"
#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/inverter.h"
#include "drivers/dma.h"
#include "drivers/rcc.h"

#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/serial_uart_impl.h"

static void usartConfigurePinInversion(uartPort_t *uartPort) {
    bool inverted = uartPort->port.options & SERIAL_INVERTED;

    if (inverted)
    {
        if (uartPort->port.mode & MODE_RX)
        {
            uartPort->Handle.AdvancedInit.AdvFeatureInit |= UART_ADVFEATURE_RXINVERT_INIT;
            uartPort->Handle.AdvancedInit.RxPinLevelInvert = UART_ADVFEATURE_RXINV_ENABLE;
        }
        if (uartPort->port.mode & MODE_TX)
        {
            uartPort->Handle.AdvancedInit.AdvFeatureInit |= UART_ADVFEATURE_TXINVERT_INIT;
            uartPort->Handle.AdvancedInit.TxPinLevelInvert = UART_ADVFEATURE_TXINV_ENABLE;
        }
    }
}

// XXX uartReconfigure does not handle resource management properly.

void uartReconfigure(uartPort_t *uartPort)
{
    HAL_UART_DeInit(&uartPort->Handle);
    uartPort->Handle.Init.BaudRate = uartPort->port.baudRate;
    // according to the stm32 documentation wordlen has to be 9 for parity bits
    // this does not seem to matter for rx but will give bad data on tx!
    uartPort->Handle.Init.WordLength = (uartPort->port.options & SERIAL_PARITY_EVEN) ? UART_WORDLENGTH_9B : UART_WORDLENGTH_8B;
    uartPort->Handle.Init.StopBits = (uartPort->port.options & SERIAL_STOPBITS_2) ? USART_STOPBITS_2 : USART_STOPBITS_1;
    uartPort->Handle.Init.Parity = (uartPort->port.options & SERIAL_PARITY_EVEN) ? USART_PARITY_EVEN : USART_PARITY_NONE;
    uartPort->Handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    uartPort->Handle.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    uartPort->Handle.Init.Mode = 0;
#if defined(STM32G4) || defined(STM32H7)
    if (uartPort->Handle.Instance == LPUART1) {
        uartPort->Handle.Init.ClockPrescaler = UART_PRESCALER_DIV8;
    }
#endif

    if (uartPort->port.mode & MODE_RX)
        uartPort->Handle.Init.Mode |= UART_MODE_RX;
    if (uartPort->port.mode & MODE_TX)
        uartPort->Handle.Init.Mode |= UART_MODE_TX;


    usartConfigurePinInversion(uartPort);

#ifdef TARGET_USART_CONFIG
    void usartTargetConfigure(uartPort_t *);
    usartTargetConfigure(uartPort);
#endif

    if (uartPort->port.options & SERIAL_BIDIR)
    {
        HAL_HalfDuplex_Init(&uartPort->Handle);
    }
    else
    {
        HAL_UART_Init(&uartPort->Handle);
    }

    // Receive DMA or IRQ
    if (uartPort->port.mode & MODE_RX)
    {
#ifdef USE_DMA
        if (uartPort->rxDMAResource)
        {
            uartPort->rxDMAHandle.Instance = (DMA_ARCH_TYPE *)uartPort->rxDMAResource;
#if !(defined(STM32H7) || defined(STM32G4))
            uartPort->rxDMAHandle.Init.Channel = uartPort->rxDMAChannel;
#else
            uartPort->txDMAHandle.Init.Request = uartPort->rxDMAChannel;
#endif
            uartPort->rxDMAHandle.Init.Direction = DMA_PERIPH_TO_MEMORY;
            uartPort->rxDMAHandle.Init.PeriphInc = DMA_PINC_DISABLE;
            uartPort->rxDMAHandle.Init.MemInc = DMA_MINC_ENABLE;
            uartPort->rxDMAHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
            uartPort->rxDMAHandle.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
            uartPort->rxDMAHandle.Init.Mode = DMA_CIRCULAR;
#if defined(STM32F4) || defined(STM32F7) || defined(STM32H7)
            uartPort->rxDMAHandle.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
            uartPort->rxDMAHandle.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_1QUARTERFULL;
            uartPort->rxDMAHandle.Init.PeriphBurst = DMA_PBURST_SINGLE;
            uartPort->rxDMAHandle.Init.MemBurst = DMA_MBURST_SINGLE;
#endif
            uartPort->rxDMAHandle.Init.Priority = DMA_PRIORITY_MEDIUM;


            HAL_DMA_DeInit(&uartPort->rxDMAHandle);
            HAL_DMA_Init(&uartPort->rxDMAHandle);
            /* Associate the initialized DMA handle to the UART handle */
            __HAL_LINKDMA(&uartPort->Handle, hdmarx, uartPort->rxDMAHandle);

            HAL_UART_Receive_DMA(&uartPort->Handle, (uint8_t*)uartPort->port.rxBuffer, uartPort->port.rxBufferSize);

            uartPort->rxDMAPos = __HAL_DMA_GET_COUNTER(&uartPort->rxDMAHandle);
        } else
#endif
        {
            /* Enable the UART Parity Error Interrupt */
            SET_BIT(uartPort->USARTx->CR1, USART_CR1_PEIE);

            /* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
            SET_BIT(uartPort->USARTx->CR3, USART_CR3_EIE);

            /* Enable the UART Data Register not empty Interrupt */
            SET_BIT(uartPort->USARTx->CR1, USART_CR1_RXNEIE);

            /* Enable Idle Line detection */
            SET_BIT(uartPort->USARTx->CR1, USART_CR1_IDLEIE);
        }
    }

    // Transmit DMA or IRQ
    if (uartPort->port.mode & MODE_TX) {
#ifdef USE_DMA
        if (uartPort->txDMAResource) {
            uartPort->txDMAHandle.Instance = (DMA_ARCH_TYPE *)uartPort->txDMAResource;
#if !(defined(STM32H7) || defined(STM32G4))
            uartPort->txDMAHandle.Init.Channel = uartPort->txDMAChannel;
#else
            uartPort->txDMAHandle.Init.Request = uartPort->txDMAChannel;
#endif
            uartPort->txDMAHandle.Init.Direction = DMA_MEMORY_TO_PERIPH;
            uartPort->txDMAHandle.Init.PeriphInc = DMA_PINC_DISABLE;
            uartPort->txDMAHandle.Init.MemInc = DMA_MINC_ENABLE;
            uartPort->txDMAHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
            uartPort->txDMAHandle.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
            uartPort->txDMAHandle.Init.Mode = DMA_NORMAL;
#if !defined(STM32G4)
            // G4's DMA is channel based, and does not have FIFO
            uartPort->txDMAHandle.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
            uartPort->txDMAHandle.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_1QUARTERFULL;
            uartPort->txDMAHandle.Init.PeriphBurst = DMA_PBURST_SINGLE;
            uartPort->txDMAHandle.Init.MemBurst = DMA_MBURST_SINGLE;
#endif
            uartPort->txDMAHandle.Init.Priority = DMA_PRIORITY_MEDIUM;


            HAL_DMA_DeInit(&uartPort->txDMAHandle);
            HAL_StatusTypeDef status = HAL_DMA_Init(&uartPort->txDMAHandle);
            if (status != HAL_OK)
            {
                while (1);
            }
            /* Associate the initialized DMA handle to the UART handle */
            __HAL_LINKDMA(&uartPort->Handle, hdmatx, uartPort->txDMAHandle);

            __HAL_DMA_SET_COUNTER(&uartPort->txDMAHandle, 0);
        } else
#endif
        {

            /* Enable the UART Transmit Data Register Empty Interrupt */
            SET_BIT(uartPort->USARTx->CR1, USART_CR1_TXEIE);
        }
    }
    return;
}

#ifdef USE_DMA
void uartTryStartTxDMA(uartPort_t *s)
{
    ATOMIC_BLOCK(NVIC_PRIO_SERIALUART_TXDMA) {
        if (IS_DMA_ENABLED(s->txDMAResource)) {
            // DMA is already in progress
            return;
        }

        HAL_UART_StateTypeDef state = HAL_UART_GetState(&s->Handle);
        if ((state & HAL_UART_STATE_BUSY_TX) == HAL_UART_STATE_BUSY_TX) {
            // UART is still transmitting
            return;
        }

        if (s->port.txBufferHead == s->port.txBufferTail) {
            // No more data to transmit
            s->txDMAEmpty = true;
            return;
        }

        uint16_t size;
        uint32_t fromwhere = s->port.txBufferTail;

        if (s->port.txBufferHead > s->port.txBufferTail) {
            size = s->port.txBufferHead - s->port.txBufferTail;
            s->port.txBufferTail = s->port.txBufferHead;
        } else {
            size = s->port.txBufferSize - s->port.txBufferTail;
            s->port.txBufferTail = 0;
        }
        s->txDMAEmpty = false;

        HAL_UART_Transmit_DMA(&s->Handle, (uint8_t *)&s->port.txBuffer[fromwhere], size);
    }
}

static void handleUsartTxDma(uartPort_t *s)
{
    uartTryStartTxDMA(s);
}

void uartDmaIrqHandler(dmaChannelDescriptor_t* descriptor)
{
    uartPort_t *s = &(((uartDevice_t*)(descriptor->userParam))->port);

    HAL_DMA_IRQHandler(&s->txDMAHandle);

#ifdef STM32G4
    // G4's DMA HAL turns on half transfer interrupt.
    // Only detect the transfer complete interrupt by checking remaining transfer count.
    // XXX TODO Consider using HAL's XferCpltCallback facility to do this.

    if (s->txDMAHandle.Instance->CNDTR == 0) {

        // Unlike other stream based DMA implementations (F4, F7 and H7),
        // G4's DMA implementation does not clear EN bit upon completion of a transfer,
        // and it is neccesary to clear the EN bit explicitly here for IS_DMA_ENABLED macro
        // used in uartTryStartTxDMA() to continue working with G4.

        __HAL_DMA_DISABLE(&s->txDMAHandle);
    }
#endif
}
#endif

FAST_CODE void uartIrqHandler(uartPort_t *s)
{
    UART_HandleTypeDef *huart = &s->Handle;
    /* UART in mode Receiver ---------------------------------------------------*/
    if ((__HAL_UART_GET_IT(huart, UART_IT_RXNE) != RESET)) {
        uint8_t rbyte = (uint8_t)(huart->Instance->RDR & (uint8_t) 0xff);

        if (s->port.rxCallback) {
            s->port.rxCallback(rbyte, s->port.rxCallbackData);
        } else {
            s->port.rxBuffer[s->port.rxBufferHead] = rbyte;
            s->port.rxBufferHead = (s->port.rxBufferHead + 1) % s->port.rxBufferSize;
        }
        CLEAR_BIT(huart->Instance->CR1, (USART_CR1_PEIE));

        /* Disable the UART Error Interrupt: (Frame error, noise error, overrun error) */
        CLEAR_BIT(huart->Instance->CR3, USART_CR3_EIE);

        __HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST);
    }

    /* UART parity error interrupt occurred -------------------------------------*/
    if ((__HAL_UART_GET_IT(huart, UART_IT_PE) != RESET)) {
        __HAL_UART_CLEAR_IT(huart, UART_CLEAR_PEF);
    }

    /* UART frame error interrupt occurred --------------------------------------*/
    if ((__HAL_UART_GET_IT(huart, UART_IT_FE) != RESET)) {
        __HAL_UART_CLEAR_IT(huart, UART_CLEAR_FEF);
    }

    /* UART noise error interrupt occurred --------------------------------------*/
    if ((__HAL_UART_GET_IT(huart, UART_IT_NE) != RESET)) {
        __HAL_UART_CLEAR_IT(huart, UART_CLEAR_NEF);
    }

    /* UART Over-Run interrupt occurred -----------------------------------------*/
    if ((__HAL_UART_GET_IT(huart, UART_IT_ORE) != RESET)) {
        __HAL_UART_CLEAR_IT(huart, UART_CLEAR_OREF);
    }

    // UART transmitter in interrupting mode, tx buffer empty

    if (
#ifdef USE_DMA
        !s->txDMAResource &&
#endif
        (__HAL_UART_GET_IT(huart, UART_IT_TXE) != RESET)) {
        /* Check that a Tx process is ongoing */
        if (huart->gState != HAL_UART_STATE_BUSY_TX) {
            if (s->port.txBufferTail == s->port.txBufferHead) {
                huart->TxXferCount = 0;
                /* Disable the UART Transmit Data Register Empty Interrupt */
                CLEAR_BIT(huart->Instance->CR1, USART_CR1_TXEIE);
            } else {
                if ((huart->Init.WordLength == UART_WORDLENGTH_9B) && (huart->Init.Parity == UART_PARITY_NONE)) {
                    huart->Instance->TDR = (((uint16_t) s->port.txBuffer[s->port.txBufferTail]) & (uint16_t) 0x01FFU);
                } else {
                    huart->Instance->TDR = (uint8_t)(s->port.txBuffer[s->port.txBufferTail]);
                }
                s->port.txBufferTail = (s->port.txBufferTail + 1) % s->port.txBufferSize;
            }
        }
    }

    // UART transmitter in DMA mode, transmission completed

    if ((__HAL_UART_GET_IT(huart, UART_IT_TC) != RESET)) {
        HAL_UART_IRQHandler(huart);
#ifdef USE_DMA
        if (s->txDMAResource) {
            handleUsartTxDma(s);
        }
#endif
    }

    // UART reception idle detected

    if (__HAL_UART_GET_IT(huart, UART_IT_IDLE)) {
        if (s->port.idleCallback) {
            s->port.idleCallback();
        }

        __HAL_UART_CLEAR_IDLEFLAG(huart);
    }

}
#endif // USE_UART
