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

#include <stdint.h>

#include "platform.h"

#ifdef USE_DMA_SPEC

#include "drivers/adc.h"
#include "drivers/bus_spi.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/timer_def.h"

#include "pg/timerio.h"

#include "dma_reqmap.h"

typedef struct dmaPeripheralMapping_s {
    dmaPeripheral_e device;
    uint8_t index;
#if defined(STM32H7) || defined(STM32G4)
    uint8_t dmaRequest;
#else
    dmaChannelSpec_t channelSpec[MAX_PERIPHERAL_DMA_OPTIONS];
#endif
} dmaPeripheralMapping_t;

typedef struct dmaTimerMapping_s {
    TIM_TypeDef *tim;
    uint8_t channel;
#if defined(STM32H7) || defined(STM32G4)
    uint8_t dmaRequest;
#else
    dmaChannelSpec_t channelSpec[MAX_TIMER_DMA_OPTIONS];
#endif
} dmaTimerMapping_t;

#if defined(STM32G4)

#define REQMAP_SGL(periph) { DMA_PERIPH_ ## periph, 0, DMA_REQUEST_ ## periph }
#define REQMAP(periph, device) { DMA_PERIPH_ ## periph, periph ## DEV_ ## device, DMA_REQUEST_ ## periph ## device }
#define REQMAP_DIR(periph, device, dir) { DMA_PERIPH_ ## periph ## _ ## dir, periph ## DEV_ ## device, DMA_REQUEST_ ## periph ## device ## _ ## dir }
#define REQMAP_TIMUP(periph, timno) { DMA_PERIPH_TIMUP, timno - 1, DMA_REQUEST_ ## TIM ## timno ## _UP }

// Resolve UART/USART mess, also map UART6 requests to LPUART1 requests
#define DMA_REQUEST_UART1_RX DMA_REQUEST_USART1_RX
#define DMA_REQUEST_UART1_TX DMA_REQUEST_USART1_TX
#define DMA_REQUEST_UART2_RX DMA_REQUEST_USART2_RX
#define DMA_REQUEST_UART2_TX DMA_REQUEST_USART2_TX
#define DMA_REQUEST_UART3_RX DMA_REQUEST_USART3_RX
#define DMA_REQUEST_UART3_TX DMA_REQUEST_USART3_TX
#define DMA_REQUEST_UART9_RX DMA_REQUEST_LPUART1_RX
#define DMA_REQUEST_UART9_TX DMA_REQUEST_LPUART1_TX

static const dmaPeripheralMapping_t dmaPeripheralMapping[] = {
#ifdef USE_SPI
    REQMAP_DIR(SPI, 1, TX),
    REQMAP_DIR(SPI, 1, RX),
    REQMAP_DIR(SPI, 2, TX),
    REQMAP_DIR(SPI, 2, RX),
    REQMAP_DIR(SPI, 3, TX),
    REQMAP_DIR(SPI, 3, RX),
    REQMAP_DIR(SPI, 4, TX),
    REQMAP_DIR(SPI, 4, RX),
#endif // USE_SPI

#ifdef USE_ADC
    REQMAP(ADC, 1),
    REQMAP(ADC, 2),
    REQMAP(ADC, 3),
    REQMAP(ADC, 4),
    REQMAP(ADC, 5),
#endif

#ifdef USE_UART
    REQMAP_DIR(UART, 1, TX),
    REQMAP_DIR(UART, 1, RX),
    REQMAP_DIR(UART, 2, TX),
    REQMAP_DIR(UART, 2, RX),
    REQMAP_DIR(UART, 3, TX),
    REQMAP_DIR(UART, 3, RX),
    REQMAP_DIR(UART, 4, TX),
    REQMAP_DIR(UART, 4, RX),
    REQMAP_DIR(UART, 5, TX),
    REQMAP_DIR(UART, 5, RX),
    REQMAP_DIR(UART, 9, TX),
    REQMAP_DIR(UART, 9, RX),
#endif

#ifdef USE_TIMER
// Pseudo peripheral for TIMx_UP channel
    REQMAP_TIMUP(TIMUP, 1),
    REQMAP_TIMUP(TIMUP, 2),
    REQMAP_TIMUP(TIMUP, 3),
    REQMAP_TIMUP(TIMUP, 4),
    REQMAP_TIMUP(TIMUP, 5),
    REQMAP_TIMUP(TIMUP, 6),
    REQMAP_TIMUP(TIMUP, 7),
    REQMAP_TIMUP(TIMUP, 8),
    REQMAP_TIMUP(TIMUP, 15),
    REQMAP_TIMUP(TIMUP, 16),
    REQMAP_TIMUP(TIMUP, 17),
    REQMAP_TIMUP(TIMUP, 20),
#endif
};

#undef REQMAP_TIMUP
#undef REQMAP
#undef REQMAP_SGL
#undef REQMAP_DIR

#define TC(chan) DEF_TIM_CHANNEL(CH_ ## chan)

#define REQMAP_TIM(tim, chan) { tim, TC(chan), DMA_REQUEST_ ## tim ## _ ## chan }

static const dmaTimerMapping_t dmaTimerMapping[] = {
    REQMAP_TIM(TIM1, CH1),
    REQMAP_TIM(TIM1, CH2),
    REQMAP_TIM(TIM1, CH3),
    REQMAP_TIM(TIM1, CH4),
    REQMAP_TIM(TIM2, CH1),
    REQMAP_TIM(TIM2, CH2),
    REQMAP_TIM(TIM2, CH3),
    REQMAP_TIM(TIM2, CH4),
    REQMAP_TIM(TIM3, CH1),
    REQMAP_TIM(TIM3, CH2),
    REQMAP_TIM(TIM3, CH3),
    REQMAP_TIM(TIM3, CH4),
    REQMAP_TIM(TIM4, CH1),
    REQMAP_TIM(TIM4, CH2),
    REQMAP_TIM(TIM4, CH3),
    REQMAP_TIM(TIM5, CH1),
    REQMAP_TIM(TIM5, CH2),
    REQMAP_TIM(TIM5, CH3),
    REQMAP_TIM(TIM5, CH4),
    REQMAP_TIM(TIM8, CH1),
    REQMAP_TIM(TIM8, CH2),
    REQMAP_TIM(TIM8, CH3),
    REQMAP_TIM(TIM8, CH4),
    REQMAP_TIM(TIM15, CH1),
    REQMAP_TIM(TIM16, CH1),
    REQMAP_TIM(TIM17, CH1),
    REQMAP_TIM(TIM20, CH1),
    REQMAP_TIM(TIM20, CH2),
    REQMAP_TIM(TIM20, CH3),
    REQMAP_TIM(TIM20, CH4),
    // XXX Check non-CH1 for TIM15,16,17 and 20
};

#undef TC
#undef REQMAP_TIM

#define DMA(d, c) { DMA_CODE(d, c, 0), (dmaResource_t *)DMA ## d ## _Channel ## c, 0 }

static dmaChannelSpec_t dmaChannelSpec[MAX_PERIPHERAL_DMA_OPTIONS] = {
    DMA(1, 1),
    DMA(1, 2),
    DMA(1, 3),
    DMA(1, 4),
    DMA(1, 5),
    DMA(1, 6),
    DMA(1, 7),
    DMA(1, 8),
    DMA(2, 1),
    DMA(2, 2),
    DMA(2, 3),
    DMA(2, 4),
    DMA(2, 5),
    DMA(2, 6),
    DMA(2, 7),
    DMA(2, 8),
};

#undef DMA

#elif defined(STM32H7)

#define REQMAP_SGL(periph) { DMA_PERIPH_ ## periph, 0, DMA_REQUEST_ ## periph }
#define REQMAP(periph, device) { DMA_PERIPH_ ## periph, periph ## DEV_ ## device, DMA_REQUEST_ ## periph ## device }
#define REQMAP_DIR(periph, device, dir) { DMA_PERIPH_ ## periph ## _ ## dir, periph ## DEV_ ## device, DMA_REQUEST_ ## periph ## device ## _ ## dir }
#define REQMAP_TIMUP(periph, timno) { DMA_PERIPH_TIMUP, timno - 1, DMA_REQUEST_ ## TIM ## timno ## _UP }

// Resolve UART/USART mess
#define DMA_REQUEST_UART1_RX DMA_REQUEST_USART1_RX
#define DMA_REQUEST_UART1_TX DMA_REQUEST_USART1_TX
#define DMA_REQUEST_UART2_RX DMA_REQUEST_USART2_RX
#define DMA_REQUEST_UART2_TX DMA_REQUEST_USART2_TX
#define DMA_REQUEST_UART3_RX DMA_REQUEST_USART3_RX
#define DMA_REQUEST_UART3_TX DMA_REQUEST_USART3_TX
#define DMA_REQUEST_UART6_RX DMA_REQUEST_USART6_RX
#define DMA_REQUEST_UART6_TX DMA_REQUEST_USART6_TX

static const dmaPeripheralMapping_t dmaPeripheralMapping[] = {
#ifdef USE_SPI
    REQMAP_DIR(SPI, 1, TX),
    REQMAP_DIR(SPI, 1, RX),
    REQMAP_DIR(SPI, 2, TX),
    REQMAP_DIR(SPI, 2, RX),
    REQMAP_DIR(SPI, 3, TX),
    REQMAP_DIR(SPI, 3, RX),
    REQMAP_DIR(SPI, 4, TX),
    REQMAP_DIR(SPI, 4, RX),
    REQMAP_DIR(SPI, 5, TX), // Not available in smaller packages
    REQMAP_DIR(SPI, 5, TX), // ditto
    // REQMAP_DIR(SPI, 6, TX), // SPI6 is on BDMA (todo)
    // REQMAP_DIR(SPI, 6, TX), // ditto
#endif // USE_SPI

#ifdef USE_ADC
    REQMAP(ADC, 1),
    REQMAP(ADC, 2),
#if defined(STM32H743xx) || defined(STM32H750xx) || defined(STM32H723xx) || defined(STM32H725xx)
    REQMAP(ADC, 3),
#endif
#endif

#ifdef USE_UART
    REQMAP_DIR(UART, 1, TX),
    REQMAP_DIR(UART, 1, RX),
    REQMAP_DIR(UART, 2, TX),
    REQMAP_DIR(UART, 2, RX),
    REQMAP_DIR(UART, 3, TX),
    REQMAP_DIR(UART, 3, RX),
    REQMAP_DIR(UART, 4, TX),
    REQMAP_DIR(UART, 4, RX),
    REQMAP_DIR(UART, 5, TX),
    REQMAP_DIR(UART, 5, RX),
    REQMAP_DIR(UART, 6, TX),
    REQMAP_DIR(UART, 6, RX),
    REQMAP_DIR(UART, 7, TX),
    REQMAP_DIR(UART, 7, RX),
    REQMAP_DIR(UART, 8, TX),
    REQMAP_DIR(UART, 8, RX),
#endif

#ifdef USE_TIMER
// Pseudo peripheral for TIMx_UP channel
    REQMAP_TIMUP(TIMUP, 1),
    REQMAP_TIMUP(TIMUP, 2),
    REQMAP_TIMUP(TIMUP, 3),
    REQMAP_TIMUP(TIMUP, 4),
    REQMAP_TIMUP(TIMUP, 5),
    REQMAP_TIMUP(TIMUP, 6),
    REQMAP_TIMUP(TIMUP, 7),
    REQMAP_TIMUP(TIMUP, 8),
    REQMAP_TIMUP(TIMUP, 15),
    REQMAP_TIMUP(TIMUP, 16),
    REQMAP_TIMUP(TIMUP, 17),
#endif
};

#undef REQMAP_TIMUP
#undef REQMAP
#undef REQMAP_SGL
#undef REQMAP_DIR

#define TC(chan) DEF_TIM_CHANNEL(CH_ ## chan)

#define REQMAP_TIM(tim, chan) { tim, TC(chan), DMA_REQUEST_ ## tim ## _ ## chan }

static const dmaTimerMapping_t dmaTimerMapping[] = {
    REQMAP_TIM(TIM1, CH1),
    REQMAP_TIM(TIM1, CH2),
    REQMAP_TIM(TIM1, CH3),
    REQMAP_TIM(TIM1, CH4),
    REQMAP_TIM(TIM2, CH1),
    REQMAP_TIM(TIM2, CH2),
    REQMAP_TIM(TIM2, CH3),
    REQMAP_TIM(TIM2, CH4),
    REQMAP_TIM(TIM3, CH1),
    REQMAP_TIM(TIM3, CH2),
    REQMAP_TIM(TIM3, CH3),
    REQMAP_TIM(TIM3, CH4),
    REQMAP_TIM(TIM4, CH1),
    REQMAP_TIM(TIM4, CH2),
    REQMAP_TIM(TIM4, CH3),
    REQMAP_TIM(TIM5, CH1),
    REQMAP_TIM(TIM5, CH2),
    REQMAP_TIM(TIM5, CH3),
    REQMAP_TIM(TIM5, CH4),
    REQMAP_TIM(TIM8, CH1),
    REQMAP_TIM(TIM8, CH2),
    REQMAP_TIM(TIM8, CH3),
    REQMAP_TIM(TIM8, CH4),
    REQMAP_TIM(TIM15, CH1),
    REQMAP_TIM(TIM16, CH1),
    REQMAP_TIM(TIM17, CH1),
};

#undef TC
#undef REQMAP_TIM

#define DMA(d, s) { DMA_CODE(d, s, 0), (dmaResource_t *)DMA ## d ## _Stream ## s, 0 }

static dmaChannelSpec_t dmaChannelSpec[MAX_PERIPHERAL_DMA_OPTIONS] = {
    DMA(1, 0),
    DMA(1, 1),
    DMA(1, 2),
    DMA(1, 3),
    DMA(1, 4),
    DMA(1, 5),
    DMA(1, 6),
    DMA(1, 7),
    DMA(2, 0),
    DMA(2, 1),
    DMA(2, 2),
    DMA(2, 3),
    DMA(2, 4),
    DMA(2, 5),
    DMA(2, 6),
    DMA(2, 7),
};

#undef DMA

#elif defined(STM32F4) || defined(STM32F7)

#if defined(STM32F4)
#define DMA(d, s, c) { DMA_CODE(d, s, c), (dmaResource_t *)DMA ## d ## _Stream ## s, DMA_Channel_ ## c }
#elif defined(STM32F7)
#define DMA(d, s, c) { DMA_CODE(d, s, c), (dmaResource_t *)DMA ## d ## _Stream ## s, DMA_CHANNEL_ ## c }
#endif

static const dmaPeripheralMapping_t dmaPeripheralMapping[] = {
#ifdef USE_SPI
    // Everything including F405 and F446
    { DMA_PERIPH_SPI_TX,  SPIDEV_1,  { DMA(2, 3, 3), DMA(2, 5, 3) } },
    { DMA_PERIPH_SPI_RX,  SPIDEV_1,  { DMA(2, 0, 3), DMA(2, 2, 3) } },
    { DMA_PERIPH_SPI_TX,  SPIDEV_2,  { DMA(1, 4, 0) } },
    { DMA_PERIPH_SPI_RX,  SPIDEV_2,  { DMA(1, 3, 0) } },
    { DMA_PERIPH_SPI_TX,  SPIDEV_3,  { DMA(1, 5, 0), DMA(1, 7, 0) } },
    { DMA_PERIPH_SPI_RX,  SPIDEV_3,  { DMA(1, 0, 0), DMA(1, 2, 0) } },

#if defined(STM32F411xE) || defined(STM32F745xx) || defined(STM32F746xx) || defined(STM32F765xx) || defined(STM32F722xx)
    { DMA_PERIPH_SPI_TX,  SPIDEV_4,  { DMA(2, 1, 4) } },
    { DMA_PERIPH_SPI_RX,  SPIDEV_4,  { DMA(2, 0, 4) } },

#ifdef USE_EXTENDED_SPI_DEVICE
    { DMA_PERIPH_SPI_TX,  SPIDEV_5,  { DMA(2, 6, 7) } },
    { DMA_PERIPH_SPI_RX,  SPIDEV_5,  { DMA(2, 5, 7) } },

#if !defined(STM32F722xx)
    { DMA_PERIPH_SPI_TX,  SPIDEV_6,  { DMA(2, 5, 1) } },
    { DMA_PERIPH_SPI_RX,  SPIDEV_6,  { DMA(2, 6, 1) } },
#endif
#endif // USE_EXTENDED_SPI_DEVICE
#endif
#endif // USE_SPI

#ifdef USE_ADC
    { DMA_PERIPH_ADC,     ADCDEV_1,  { DMA(2, 0, 0), DMA(2, 4, 0) } },
    { DMA_PERIPH_ADC,     ADCDEV_2,  { DMA(2, 2, 1), DMA(2, 3, 1) } },
    { DMA_PERIPH_ADC,     ADCDEV_3,  { DMA(2, 0, 2), DMA(2, 1, 2) } },
#endif

#ifdef USE_SDCARD_SDIO
    { DMA_PERIPH_SDIO,    0,         { DMA(2, 3, 4), DMA(2, 6, 4) } },
#endif

#ifdef USE_UART
    { DMA_PERIPH_UART_TX, UARTDEV_1, { DMA(2, 7, 4) } },
    { DMA_PERIPH_UART_RX, UARTDEV_1, { DMA(2, 5, 4), DMA(2, 2, 4) } },
    { DMA_PERIPH_UART_TX, UARTDEV_2, { DMA(1, 6, 4) } },
    { DMA_PERIPH_UART_RX, UARTDEV_2, { DMA(1, 5, 4) } },
    { DMA_PERIPH_UART_TX, UARTDEV_3, { DMA(1, 3, 4) } },
    { DMA_PERIPH_UART_RX, UARTDEV_3, { DMA(1, 1, 4) } },
    { DMA_PERIPH_UART_TX, UARTDEV_4, { DMA(1, 4, 4) } },
    { DMA_PERIPH_UART_RX, UARTDEV_4, { DMA(1, 2, 4) } },
    { DMA_PERIPH_UART_TX, UARTDEV_5, { DMA(1, 7, 4) } },
    { DMA_PERIPH_UART_RX, UARTDEV_5, { DMA(1, 0, 4) } },
    { DMA_PERIPH_UART_TX, UARTDEV_6, { DMA(2, 6, 5), DMA(2, 7, 5) } },
    { DMA_PERIPH_UART_RX, UARTDEV_6, { DMA(2, 1, 5), DMA(2, 2, 5) } },
#endif
};

#define TC(chan) DEF_TIM_CHANNEL(CH_ ## chan)

static const dmaTimerMapping_t dmaTimerMapping[] = {
    // Generated from 'timer_def.h'
    { TIM1, TC(CH1), { DMA(2, 6, 0), DMA(2, 1, 6), DMA(2, 3, 6) } },
    { TIM1, TC(CH2), { DMA(2, 6, 0), DMA(2, 2, 6) } },
    { TIM1, TC(CH3), { DMA(2, 6, 0), DMA(2, 6, 6) } },
    { TIM1, TC(CH4), { DMA(2, 4, 6) } },

    { TIM2, TC(CH1), { DMA(1, 5, 3) } },
    { TIM2, TC(CH2), { DMA(1, 6, 3) } },
    { TIM2, TC(CH3), { DMA(1, 1, 3) } },
    { TIM2, TC(CH4), { DMA(1, 7, 3), DMA(1, 6, 3) } },

    { TIM3, TC(CH1), { DMA(1, 4, 5) } },
    { TIM3, TC(CH2), { DMA(1, 5, 5) } },
    { TIM3, TC(CH3), { DMA(1, 7, 5) } },
    { TIM3, TC(CH4), { DMA(1, 2, 5) } },

    { TIM4, TC(CH1), { DMA(1, 0, 2) } },
    { TIM4, TC(CH2), { DMA(1, 3, 2) } },
    { TIM4, TC(CH3), { DMA(1, 7, 2) } },

    { TIM5, TC(CH1), { DMA(1, 2, 6) } },
    { TIM5, TC(CH2), { DMA(1, 4, 6) } },
    { TIM5, TC(CH3), { DMA(1, 0, 6) } },
    { TIM5, TC(CH4), { DMA(1, 1, 6), DMA(1, 3, 6) } },

    { TIM8, TC(CH1), { DMA(2, 2, 0), DMA(2, 2, 7) } },
    { TIM8, TC(CH2), { DMA(2, 2, 0), DMA(2, 3, 7) } },
    { TIM8, TC(CH3), { DMA(2, 2, 0), DMA(2, 4, 7) } },
    { TIM8, TC(CH4), { DMA(2, 7, 7) } },
};
#undef TC
#undef DMA

#else // STM32F3
    // The embedded ADC24_DMA_REMAP conditional should be removed
    // when (and if) F3 is going generic.
#define DMA(d, c) { DMA_CODE(d, 0, c), (dmaResource_t *)DMA ## d ## _Channel ## c }
static const dmaPeripheralMapping_t dmaPeripheralMapping[18] = {
#ifdef USE_SPI
    { DMA_PERIPH_SPI_TX,  SPIDEV_1, { DMA(1, 3) } },
    { DMA_PERIPH_SPI_RX,  SPIDEV_1, { DMA(1, 2) } },
    { DMA_PERIPH_SPI_TX,  SPIDEV_2, { DMA(1, 5) } },
    { DMA_PERIPH_SPI_RX,  SPIDEV_2, { DMA(1, 4) } },
    { DMA_PERIPH_SPI_TX,  SPIDEV_3, { DMA(2, 2) } },
    { DMA_PERIPH_SPI_RX,  SPIDEV_3, { DMA(2, 1) } },
#endif

#ifdef USE_ADC
    { DMA_PERIPH_ADC,     ADCDEV_1, { DMA(1, 1) } },
#ifdef ADC24_DMA_REMAP
    { DMA_PERIPH_ADC,     ADCDEV_2, { DMA(2, 3) } },
#else
    { DMA_PERIPH_ADC,     ADCDEV_2, { DMA(2, 1) } },
#endif
    { DMA_PERIPH_ADC,     ADCDEV_3, { DMA(2, 5) } },
#ifdef ADC24_DMA_REMAP
    { DMA_PERIPH_ADC,     ADCDEV_4, { DMA(2, 4) } },
#else
    { DMA_PERIPH_ADC,     ADCDEV_4, { DMA(2, 2) } },
#endif
#endif

#ifdef USE_UART
    { DMA_PERIPH_UART_TX, UARTDEV_1, { DMA(1, 4) } },
    { DMA_PERIPH_UART_RX, UARTDEV_1, { DMA(1, 5) } },

    { DMA_PERIPH_UART_TX, UARTDEV_2, { DMA(1, 7) } },
    { DMA_PERIPH_UART_RX, UARTDEV_2, { DMA(1, 6) } },
    { DMA_PERIPH_UART_TX, UARTDEV_3, { DMA(1, 2) } },
    { DMA_PERIPH_UART_RX, UARTDEV_3, { DMA(1, 3) } },
    { DMA_PERIPH_UART_TX, UARTDEV_4, { DMA(2, 5) } },
    { DMA_PERIPH_UART_RX, UARTDEV_4, { DMA(2, 3) } },
};
#endif

#define TC(chan) DEF_TIM_CHANNEL(CH_ ## chan)

static const dmaTimerMapping_t dmaTimerMapping[] = {
    // Generated from 'timer_def.h'
    { TIM1, TC(CH1), { DMA(1, 2) } },
    { TIM1, TC(CH2), { DMA(1, 3) } },
    { TIM1, TC(CH3), { DMA(1, 6) } },
    { TIM1, TC(CH4), { DMA(1, 4) } },

    { TIM2, TC(CH1), { DMA(1, 5) } },
    { TIM2, TC(CH2), { DMA(1, 7) } },
    { TIM2, TC(CH3), { DMA(1, 1) } },
    { TIM2, TC(CH4), { DMA(1, 7) } },

    { TIM3, TC(CH1), { DMA(1, 6) } },
    { TIM3, TC(CH3), { DMA(1, 2) } },
    { TIM3, TC(CH4), { DMA(1, 3) } },

    { TIM4, TC(CH1), { DMA(1, 1) } },
    { TIM4, TC(CH2), { DMA(1, 4) } },
    { TIM4, TC(CH3), { DMA(1, 5) } },

    { TIM8, TC(CH1), { DMA(2, 3) } },
    { TIM8, TC(CH2), { DMA(2, 5) } },
    { TIM8, TC(CH3), { DMA(2, 1) } },
    { TIM8, TC(CH4), { DMA(2, 2) } },

    { TIM15, TC(CH1), { DMA(1, 5) } },

#ifdef REMAP_TIM16_DMA
    { TIM16, TC(CH1), { DMA(1, 6) } },
#else
    { TIM16, TC(CH1), { DMA(1, 3) } },
#endif

#ifdef REMAP_TIM17_DMA
    { TIM17, TC(CH1), { DMA(1, 7) } },
#else
    { TIM17, TC(CH1), { DMA(1, 1) } },
#endif
};

#undef TC
#undef DMA
#endif

#if defined(STM32H7) || defined(STM32G4)
static void dmaSetupRequest(dmaChannelSpec_t *dmaSpec, uint8_t request)
{
    // Setup request as channel
    dmaSpec->channel = request;

    // Insert DMA request into code
    dmaCode_t code = dmaSpec->code;
    dmaSpec->code = DMA_CODE(DMA_CODE_CONTROLLER(code), DMA_CODE_STREAM(code), dmaSpec->channel);
}
#endif

const dmaChannelSpec_t *dmaGetChannelSpecByPeripheral(dmaPeripheral_e device, uint8_t index, int8_t opt)
{
    if (opt < 0 || opt >= MAX_PERIPHERAL_DMA_OPTIONS) {
        return NULL;
    }

    for (unsigned i = 0 ; i < ARRAYLEN(dmaPeripheralMapping) ; i++) {
        const dmaPeripheralMapping_t *periph = &dmaPeripheralMapping[i];
#if defined(STM32H7) || defined(STM32G4)
        if (periph->device == device && periph->index == index) {
            dmaChannelSpec_t *dmaSpec = &dmaChannelSpec[opt];
            dmaSetupRequest(dmaSpec, periph->dmaRequest);
            return dmaSpec;
        }
#else
        if (periph->device == device && periph->index == index && periph->channelSpec[opt].ref) {
            return &periph->channelSpec[opt];
        }
#endif
    }

    return NULL;
}

dmaoptValue_t dmaoptByTag(ioTag_t ioTag)
{
#ifdef USE_TIMER_MGMT
    for (unsigned i = 0; i < MAX_TIMER_PINMAP_COUNT; i++) {
        if (timerIOConfig(i)->ioTag == ioTag) {
            return timerIOConfig(i)->dmaopt;
        }
    }
#else
    UNUSED(ioTag);
#endif

    return DMA_OPT_UNUSED;
}

const dmaChannelSpec_t *dmaGetChannelSpecByTimerValue(TIM_TypeDef *tim, uint8_t channel, dmaoptValue_t dmaopt)
{
    if (dmaopt < 0 || dmaopt >= MAX_TIMER_DMA_OPTIONS) {
        return NULL;
    }

    for (unsigned i = 0 ; i < ARRAYLEN(dmaTimerMapping) ; i++) {
        const dmaTimerMapping_t *timerMapping = &dmaTimerMapping[i];
#if defined(STM32H7) || defined(STM32G4)
        if (timerMapping->tim == tim && timerMapping->channel == channel) {
            dmaChannelSpec_t *dmaSpec = &dmaChannelSpec[dmaopt];
            dmaSetupRequest(dmaSpec, timerMapping->dmaRequest);
            return dmaSpec;
        }
#else
        if (timerMapping->tim == tim && timerMapping->channel == channel && timerMapping->channelSpec[dmaopt].ref) {
            return &timerMapping->channelSpec[dmaopt];
        }
#endif
    }

    return NULL;
}

const dmaChannelSpec_t *dmaGetChannelSpecByTimer(const timerHardware_t *timer)
{
    if (!timer) {
        return NULL;
    }

    dmaoptValue_t dmaopt = dmaoptByTag(timer->tag);
    return dmaGetChannelSpecByTimerValue(timer->tim, timer->channel, dmaopt);
}

// dmaGetOptionByTimer is called by pgResetFn_timerIOConfig to find out dmaopt for pre-configured timer.

dmaoptValue_t dmaGetOptionByTimer(const timerHardware_t *timer)
{
#if defined(STM32H7) || defined(STM32G4)
    for (unsigned opt = 0; opt < ARRAYLEN(dmaChannelSpec); opt++) {
        if (timer->dmaRefConfigured == dmaChannelSpec[opt].ref) {
                return (dmaoptValue_t)opt;
        }
    }
#else
    for (unsigned i = 0 ; i < ARRAYLEN(dmaTimerMapping); i++) {
        const dmaTimerMapping_t *timerMapping = &dmaTimerMapping[i];
        if (timerMapping->tim == timer->tim && timerMapping->channel == timer->channel) {
            for (unsigned j = 0; j < MAX_TIMER_DMA_OPTIONS; j++) {
                const dmaChannelSpec_t *dma = &timerMapping->channelSpec[j];
                if (dma->ref == timer->dmaRefConfigured
#if defined(STM32F4) || defined(STM32F7)
                    && dma->channel == timer->dmaChannelConfigured
#endif
                    ) {
                    return j;
                }
            }
        }
    }
#endif

    return DMA_OPT_UNUSED;
}

#if defined(STM32H7) || defined(STM32G4)
// A variant of dmaGetOptionByTimer that looks for matching dmaTimUPRef
dmaoptValue_t dmaGetUpOptionByTimer(const timerHardware_t *timer)
{
    for (unsigned opt = 0; opt < ARRAYLEN(dmaChannelSpec); opt++) {
        if (timer->dmaTimUPRef == dmaChannelSpec[opt].ref) {
                return (dmaoptValue_t)opt;
        }
    }

    return DMA_OPT_UNUSED;
}
#endif

#endif // USE_DMA_SPEC
