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
#include <string.h>

#include "platform.h"

#ifdef USE_TRANSPONDER

#include "drivers/dma.h"
#include "drivers/dma_reqmap.h"
#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/rcc.h"
#include "drivers/timer.h"
#include "drivers/transponder_ir_arcitimer.h"
#include "drivers/transponder_ir_erlt.h"
#include "drivers/transponder_ir_ilap.h"

#include "transponder_ir.h"

volatile uint8_t transponderIrDataTransferInProgress = 0;

static IO_t transponderIO = IO_NONE;
static TIM_HandleTypeDef TimHandle;
static uint16_t timerChannel = 0;
static uint8_t output;
static uint8_t alternateFunction;

#if !(defined(STM32F7) || defined(STM32H7) || defined(STM32G4))
#error "Transponder (via HAL) not supported on this MCU."
#endif

#if defined(STM32H7)
DMA_RAM transponder_t transponder;
#elif defined(STM32G4)
DMA_RAM_W transponder_t transponder;
#else
transponder_t transponder;
#endif
bool transponderInitialised = false;

FAST_CODE static void TRANSPONDER_DMA_IRQHandler(dmaChannelDescriptor_t* descriptor)
{
    HAL_DMA_IRQHandler(TimHandle.hdma[descriptor->userParam]);
    TIM_DMACmd(&TimHandle, timerChannel, DISABLE);
    transponderIrDataTransferInProgress = 0;
}

void transponderIrHardwareInit(ioTag_t ioTag, transponder_t *transponder)
{
    if (!ioTag) {
        return;
    }

    const timerHardware_t *timerHardware = timerAllocate(ioTag, OWNER_TRANSPONDER, 0);
    TIM_TypeDef *timer = timerHardware->tim;
    timerChannel = timerHardware->channel;
    output = timerHardware->output;
    alternateFunction = timerHardware->alternateFunction;

#if defined(USE_DMA_SPEC)
    const dmaChannelSpec_t *dmaSpec = dmaGetChannelSpecByTimer(timerHardware);

    if (dmaSpec == NULL) {
        return;
    }

    dmaResource_t *dmaRef = dmaSpec->ref;
    uint32_t dmaChannel = dmaSpec->channel;
#else
    dmaResource_t *dmaRef = timerHardware->dmaRef;
    uint32_t dmaChannel = timerHardware->dmaChannel;
#endif

    if (dmaRef == NULL) {
        return;
    }

    /* Time base configuration */

    TimHandle.Instance = timer;

    uint16_t prescaler = timerGetPrescalerByDesiredMhz(timer, transponder->timer_hz);
    uint16_t period = timerGetPeriodByPrescaler(timer, prescaler, transponder->timer_carrier_hz);

    transponder->bitToggleOne = period / 2;

    TimHandle.Init.Prescaler = prescaler;
    TimHandle.Init.Period = period; // 800kHz
    TimHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
    if (HAL_TIM_PWM_Init(&TimHandle) != HAL_OK) {
        /* Initialization Error */
        return;
    }

    /* IO configuration */

    static DMA_HandleTypeDef hdma_tim;

    transponderIO = IOGetByTag(ioTag);
    IOInit(transponderIO, OWNER_TRANSPONDER, 0);
    IOConfigGPIOAF(transponderIO, IO_CONFIG(GPIO_MODE_AF_PP, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_PULLDOWN), timerHardware->alternateFunction);

    __DMA1_CLK_ENABLE();
    __DMA2_CLK_ENABLE();

    /* Set the parameters to be configured */
#if defined(STM32H7) || defined(STM32G4)
    hdma_tim.Init.Request = dmaChannel;
#else
    hdma_tim.Init.Channel = dmaChannel;
#endif
    hdma_tim.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_tim.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tim.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_tim.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_tim.Init.Mode = DMA_NORMAL;
    hdma_tim.Init.Priority = DMA_PRIORITY_HIGH;
#if !defined(STM32G4)
    hdma_tim.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    hdma_tim.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    hdma_tim.Init.MemBurst = DMA_MBURST_SINGLE;
    hdma_tim.Init.PeriphBurst = DMA_PBURST_SINGLE;
#endif

    /* Set hdma_tim instance */
    hdma_tim.Instance = (DMA_ARCH_TYPE *)dmaRef;

    uint16_t dmaIndex = timerDmaIndex(timerChannel);

    /* Link hdma_tim to hdma[x] (channelx) */
    __HAL_LINKDMA(&TimHandle, hdma[dmaIndex], hdma_tim);

    dmaInit(dmaGetIdentifier(dmaRef), OWNER_TRANSPONDER, 0);
    dmaSetHandler(dmaGetIdentifier(dmaRef), TRANSPONDER_DMA_IRQHandler, NVIC_PRIO_TRANSPONDER_DMA, dmaIndex);

    /* Initialize TIMx DMA handle */
    if (HAL_DMA_Init(TimHandle.hdma[dmaIndex]) != HAL_OK) {
        /* Initialization Error */
        return;
    }


    RCC_ClockCmd(timerRCC(timer), ENABLE);

    /* PWM1 Mode configuration: Channel1 */
    TIM_OC_InitTypeDef  TIM_OCInitStructure;

    TIM_OCInitStructure.OCMode = TIM_OCMODE_PWM1;
    TIM_OCInitStructure.OCIdleState = TIM_OCIDLESTATE_RESET;
    TIM_OCInitStructure.OCPolarity = (timerHardware->output & TIMER_OUTPUT_INVERTED) ? TIM_OCPOLARITY_LOW : TIM_OCPOLARITY_HIGH;
    TIM_OCInitStructure.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    TIM_OCInitStructure.OCNPolarity = (timerHardware->output & TIMER_OUTPUT_INVERTED) ? TIM_OCNPOLARITY_LOW : TIM_OCNPOLARITY_HIGH;
    TIM_OCInitStructure.Pulse = 0;
    TIM_OCInitStructure.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &TIM_OCInitStructure, timerChannel) != HAL_OK) {
        /* Configuration Error */
        return;
    }
    if (timerHardware->output & TIMER_OUTPUT_N_CHANNEL) {
        if (HAL_TIMEx_PWMN_Start(&TimHandle, timerChannel) != HAL_OK) {
            /* Starting PWM generation Error */
            return;
        }
    } else {
        if (HAL_TIM_PWM_Start(&TimHandle, timerChannel) != HAL_OK) {
            /* Starting PWM generation Error */
            return;
        }
    }

    transponderInitialised = true;
}

bool transponderIrInit(const ioTag_t ioTag, const transponderProvider_e provider)
{
    if (!ioTag) {
        return false;
    }

    switch (provider) {
        case TRANSPONDER_ARCITIMER:
            transponderIrInitArcitimer(&transponder);
            break;
        case TRANSPONDER_ILAP:
            transponderIrInitIlap(&transponder);
            break;
        case TRANSPONDER_ERLT:
            transponderIrInitERLT(&transponder);
            break;
        default:
            return false;
    }

    transponderIrHardwareInit(ioTag, &transponder);

    return true;
}

bool isTransponderIrReady(void)
{
    return !transponderIrDataTransferInProgress;
}

void transponderIrWaitForTransmitComplete(void)
{
#ifdef DEBUG
    static uint32_t waitCounter = 0;
#endif

    while (transponderIrDataTransferInProgress) {
#ifdef DEBUG
        waitCounter++;
#endif
    }
}

void transponderIrUpdateData(const uint8_t* transponderData)
{
     transponderIrWaitForTransmitComplete();
     transponder.vTable->updateTransponderDMABuffer(&transponder, transponderData);
}

void transponderIrDMAEnable(transponder_t *transponder)
{
    if (!transponderInitialised) {
        return;
    }

    if (DMA_SetCurrDataCounter(&TimHandle, timerChannel, transponder->transponderIrDMABuffer.ilap, transponder->dma_buffer_size) != HAL_OK) {
        /* DMA set error */
        transponderIrDataTransferInProgress = 0;
        return;
    }

    /* Reset timer counter */
    __HAL_TIM_SET_COUNTER(&TimHandle, 0);
    /* Enable channel DMA requests */
    TIM_DMACmd(&TimHandle, timerChannel, ENABLE);
}

void transponderIrDisable(void)
{
    if (!transponderInitialised) {
        return;
    }

    TIM_DMACmd(&TimHandle, timerChannel, DISABLE);
    if (output & TIMER_OUTPUT_N_CHANNEL) {
        HAL_TIMEx_PWMN_Stop(&TimHandle, timerChannel);
    } else {
        HAL_TIM_PWM_Stop(&TimHandle, timerChannel);
    }


    IOInit(transponderIO, OWNER_TRANSPONDER, 0);

#ifdef TRANSPONDER_INVERTED
    IOHi(transponderIO);
#else
    IOLo(transponderIO);
#endif

    IOConfigGPIOAF(transponderIO, IO_CONFIG(GPIO_MODE_AF_PP, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_PULLDOWN), alternateFunction);
}

void transponderIrTransmit(void)
{
    transponderIrWaitForTransmitComplete();

    transponderIrDataTransferInProgress = 1;
    transponderIrDMAEnable(&transponder);
}
#endif
