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

#ifdef USE_LED_STRIP

#include "common/color.h"

#include "drivers/dma.h"
#include "drivers/dma_reqmap.h"
#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/rcc.h"
#include "drivers/system.h"
#include "drivers/timer.h"

#include "light_ws2811strip.h"

static IO_t ws2811IO = IO_NONE;

static TIM_HandleTypeDef TimHandle;
static uint16_t timerChannel = 0;

FAST_CODE void WS2811_DMA_IRQHandler(dmaChannelDescriptor_t* descriptor)
{
    HAL_DMA_IRQHandler(TimHandle.hdma[descriptor->userParam]);
    TIM_DMACmd(&TimHandle, timerChannel, DISABLE);
    ws2811LedDataTransferInProgress = false;
}

bool ws2811LedStripHardwareInit(ioTag_t ioTag)
{
    if (!ioTag) {
        return false;
    }

    const timerHardware_t *timerHardware = timerAllocate(ioTag, OWNER_LED_STRIP, 0);

    if (timerHardware == NULL) {
        return false;
    }

    TIM_TypeDef *timer = timerHardware->tim;
    timerChannel = timerHardware->channel;

    dmaResource_t *dmaRef;

#if defined(USE_DMA_SPEC)
    const dmaChannelSpec_t *dmaSpec = dmaGetChannelSpecByTimer(timerHardware);

    if (dmaSpec == NULL) {
        return false;
    }

    dmaRef = dmaSpec->ref;
    uint32_t dmaChannel = dmaSpec->channel;
#else
    dmaRef = timerHardware->dmaRef;
    uint32_t dmaChannel = timerHardware->dmaChannel;
#endif

    if (dmaRef == NULL) {
        return false;
    }
    TimHandle.Instance = timer;

    /* Compute the prescaler value */
    uint16_t prescaler = timerGetPrescalerByDesiredMhz(timer, WS2811_TIMER_MHZ);
    uint16_t period = timerGetPeriodByPrescaler(timer, prescaler, WS2811_CARRIER_HZ);

    BIT_COMPARE_1 = period / 3 * 2;
    BIT_COMPARE_0 = period / 3;

    TimHandle.Init.Prescaler = prescaler;
    TimHandle.Init.Period = period; // 800kHz
    TimHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
    if (HAL_TIM_PWM_Init(&TimHandle) != HAL_OK) {
        /* Initialization Error */
        return false;
    }

    static DMA_HandleTypeDef hdma_tim;

    ws2811IO = IOGetByTag(ioTag);
    IOInit(ws2811IO, OWNER_LED_STRIP, 0);
    IOConfigGPIOAF(ws2811IO, IO_CONFIG(GPIO_MODE_AF_PP, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_PULLDOWN), timerHardware->alternateFunction);

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

    dmaInit(dmaGetIdentifier(dmaRef), OWNER_LED_STRIP, 0);
    dmaSetHandler(dmaGetIdentifier(dmaRef), WS2811_DMA_IRQHandler, NVIC_PRIO_WS2811_DMA, dmaIndex);

    /* Initialize TIMx DMA handle */
    if (HAL_DMA_Init(TimHandle.hdma[dmaIndex]) != HAL_OK) {
        /* Initialization Error */
        return false;
    }

    TIM_OC_InitTypeDef TIM_OCInitStructure;

    /* PWM1 Mode configuration: Channel1 */
    TIM_OCInitStructure.OCMode = TIM_OCMODE_PWM1;
    TIM_OCInitStructure.OCIdleState = TIM_OCIDLESTATE_RESET;
    TIM_OCInitStructure.OCPolarity = (timerHardware->output & TIMER_OUTPUT_INVERTED) ? TIM_OCPOLARITY_LOW : TIM_OCPOLARITY_HIGH;
    TIM_OCInitStructure.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    TIM_OCInitStructure.OCNPolarity = (timerHardware->output & TIMER_OUTPUT_INVERTED) ? TIM_OCNPOLARITY_LOW : TIM_OCNPOLARITY_HIGH;
    TIM_OCInitStructure.Pulse = 0;
    TIM_OCInitStructure.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &TIM_OCInitStructure, timerChannel) != HAL_OK) {
        /* Configuration Error */
        return false;
    }
    if (timerHardware->output & TIMER_OUTPUT_N_CHANNEL) {
        if (HAL_TIMEx_PWMN_Start(&TimHandle, timerChannel) != HAL_OK) {
            /* Starting PWM generation Error */
            return false;
        }
    } else {
        if (HAL_TIM_PWM_Start(&TimHandle, timerChannel) != HAL_OK) {
            /* Starting PWM generation Error */
            return false;
        }
    }

    return true;
}

void ws2811LedStripDMAEnable(void)
{
    if (DMA_SetCurrDataCounter(&TimHandle, timerChannel, ledStripDMABuffer, WS2811_DMA_BUFFER_SIZE) != HAL_OK) {
        /* DMA set error */
        ws2811LedDataTransferInProgress = false;
        return;
    }
    /* Reset timer counter */
    __HAL_TIM_SET_COUNTER(&TimHandle,0);
    /* Enable channel DMA requests */
    TIM_DMACmd(&TimHandle,timerChannel,ENABLE);
}
#endif
