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
#include <math.h>
#include <string.h>

#include "platform.h"

#ifdef USE_DSHOT

#include "drivers/dma_reqmap.h"
#include "drivers/io.h"
#include "timer.h"
#include "pwm_output.h"
#include "drivers/dshot.h"
#include "drivers/dshot_dpwm.h"
#include "drivers/dshot_command.h"
#include "drivers/nvic.h"
#include "dma.h"
#include "rcc.h"
#include "pg/timerup.h"

static HAL_StatusTypeDef result;

static uint8_t dmaMotorTimerCount = 0;
static motorDmaTimer_t dmaMotorTimers[MAX_DMA_TIMERS];
static motorDmaOutput_t dmaMotors[MAX_SUPPORTED_MOTORS];

motorDmaOutput_t *getMotorDmaOutput(uint8_t index)
{
    return &dmaMotors[index];
}

uint8_t getTimerIndex(TIM_TypeDef *timer)
{
    for (int i = 0; i < dmaMotorTimerCount; i++) {
        if (dmaMotorTimers[i].timer == timer) {
            return i;
        }
    }
    dmaMotorTimers[dmaMotorTimerCount++].timer = timer;
    return dmaMotorTimerCount - 1;
}

// TIM_CHANNEL_x TIM_CHANNEL_x/4 TIM_DMA_ID_CCx TIM_DMA_CCx
// 0x0           0               1              0x0200
// 0x4           1               2              0x0400
// 0x8           2               3              0x0800
// 0xC           3               4              0x1000
//
// TIM_CHANNEL_TO_TIM_DMA_ID_CC = (TIM_CHANNEL_x / 4) + 1
// TIM_CHANNEL_TO_TIM_DMA_CC = 0x200 << (TIM_CHANNEL_x / 4)

// pwmChannelDMAStart
// A variant of HAL_TIM_PWM_Start_DMA/HAL_TIMEx_PWMN_Start_DMA that only disables DMA on a timer channel:
//   1. Configure and enable DMA Stream
//   2. Enable DMA request on a timer channel

void pwmChannelDMAStart(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData, uint16_t Length)
{
    switch (Channel) {
    case TIM_CHANNEL_1:
        HAL_DMA_Start_IT(htim->hdma[TIM_DMA_ID_CC1], (uint32_t)pData, (uint32_t)&htim->Instance->CCR1, Length);
        __HAL_TIM_ENABLE_DMA(htim, TIM_DMA_CC1);
    break;

    case TIM_CHANNEL_2:
        HAL_DMA_Start_IT(htim->hdma[TIM_DMA_ID_CC2], (uint32_t)pData, (uint32_t)&htim->Instance->CCR2, Length);
        __HAL_TIM_ENABLE_DMA(htim, TIM_DMA_CC2);
        break;

    case TIM_CHANNEL_3:
        HAL_DMA_Start_IT(htim->hdma[TIM_DMA_ID_CC3], (uint32_t)pData, (uint32_t)&htim->Instance->CCR3,Length);
        __HAL_TIM_ENABLE_DMA(htim, TIM_DMA_CC3);
        break;

    case TIM_CHANNEL_4:
        HAL_DMA_Start_IT(htim->hdma[TIM_DMA_ID_CC4], (uint32_t)pData, (uint32_t)&htim->Instance->CCR4, Length);
        __HAL_TIM_ENABLE_DMA(htim, TIM_DMA_CC4);
        break;
    }
}

// pwmChannelDMAStop
// A variant of HAL_TIM_PWM_Stop_DMA/HAL_TIMEx_PWMN_Stop_DMA that only disables DMA on a timer channel
//   1. Disable the TIM Capture/Compare 1 DMA request

void pwmChannelDMAStop(TIM_HandleTypeDef *htim, uint32_t Channel)
{
    switch (Channel) {
    case TIM_CHANNEL_1:
        __HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC1);
        break;

    case TIM_CHANNEL_2:
        __HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC2);
        break;

    case TIM_CHANNEL_3:
        __HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC3);
        break;

    case TIM_CHANNEL_4:
        __HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC4);
        break;
    }
}

// pwmBurstDMAStart
// A variant of HAL_TIM_DMABurst_WriteStart that can handle multiple bursts.
// (HAL_TIM_DMABurst_WriteStart can only handle single burst)

void pwmBurstDMAStart(TIM_HandleTypeDef *htim, uint32_t BurstBaseAddress, uint32_t BurstRequestSrc, uint32_t BurstUnit, uint32_t* BurstBuffer, uint32_t BurstLength)
{
    // Setup DMA stream
    HAL_DMA_Start_IT(htim->hdma[TIM_DMA_ID_UPDATE], (uint32_t)BurstBuffer, (uint32_t)&htim->Instance->DMAR, BurstLength);

    // Configure burst mode DMA */
   htim->Instance->DCR = BurstBaseAddress | BurstUnit;

    // Enable burst mode DMA
    __HAL_TIM_ENABLE_DMA(htim, BurstRequestSrc);
}

FAST_CODE void pwmWriteDshotInt(uint8_t index, uint16_t value)
{
    motorDmaOutput_t *const motor = &dmaMotors[index];

    if (!motor->configured) {
        return;
    }

    /*If there is a command ready to go overwrite the value and send that instead*/
    if (dshotCommandIsProcessing()) {
        value = dshotCommandGetCurrent(index);
        if (value) {
            motor->protocolControl.requestTelemetry = true;
        }
    }

    if (!motor->timerHardware
#ifndef USE_DMA_SPEC
        // When USE_DMA_SPEC is in effect, motor->timerHardware remains NULL if valid DMA is not assigned.
        || !motor->timerHardware->dmaRef
#endif
    )
    {
        return;
    }

    motor->protocolControl.value = value;

    uint16_t packet = prepareDshotPacket(&motor->protocolControl);
    uint8_t bufferSize;

#ifdef USE_DSHOT_DMAR
    if (useBurstDshot) {
        bufferSize = loadDmaBuffer(&motor->timer->dmaBurstBuffer[timerLookupChannelIndex(motor->timerHardware->channel)], 4, packet);
        motor->timer->dmaBurstLength = bufferSize * 4;
    } else
#endif
    {
        bufferSize = loadDmaBuffer(motor->dmaBuffer, 1, packet);

        pwmChannelDMAStart(&motor->TimHandle, motor->timerHardware->channel, motor->dmaBuffer, bufferSize);
    }
}

void pwmCompleteDshotMotorUpdate(void)
{
    // If there is a dshot command loaded up, time it correctly with motor update

    if (!dshotCommandQueueEmpty() && !dshotCommandOutputIsEnabled(dshotPwmDevice.count)) {
        return;
    }

#ifdef USE_DSHOT_DMAR
    if (useBurstDshot) {
        for (int i = 0; i < dmaMotorTimerCount; i++) {
            motorDmaTimer_t *burstDmaTimer = &dmaMotorTimers[i];

            // Transfer CCR1 through CCR4 for each burst
            pwmBurstDMAStart(&burstDmaTimer->timHandle,
                    TIM_DMABASE_CCR1, TIM_DMA_UPDATE, TIM_DMABURSTLENGTH_4TRANSFERS,
                    (uint32_t*)burstDmaTimer->dmaBurstBuffer, burstDmaTimer->dmaBurstLength);
        }
    } else
#endif
    {
        // XXX Empty for non-burst?
    }
}

FAST_CODE static void motor_DMA_IRQHandler(dmaChannelDescriptor_t* descriptor)
{
    if (DMA_GET_FLAG_STATUS(descriptor, DMA_IT_TCIF)) {

#ifdef USE_DSHOT_DMAR
        if (useBurstDshot) {
            motorDmaTimer_t *burstDmaTimer = &dmaMotorTimers[descriptor->userParam];

            HAL_TIM_DMABurst_WriteStop(&burstDmaTimer->timHandle, TIM_DMA_UPDATE);
            HAL_DMA_IRQHandler(&burstDmaTimer->hdma_tim);
        } else
#endif
        {
            motorDmaOutput_t * const motor = &dmaMotors[descriptor->userParam];

            pwmChannelDMAStop(&motor->TimHandle,motor->timerHardware->channel);
            HAL_DMA_IRQHandler(motor->TimHandle.hdma[motor->timerDmaIndex]);
        }

        DMA_CLEAR_FLAG(descriptor, DMA_IT_TCIF);
    }
}

bool pwmDshotMotorHardwareConfig(const timerHardware_t *timerHardware, uint8_t motorIndex, uint8_t reorderedMotorIndex, motorPwmProtocolTypes_e pwmProtocolType, uint8_t output)
{
    dmaResource_t *dmaRef = NULL;
    uint32_t dmaChannel;

#ifdef USE_DMA_SPEC
    const dmaChannelSpec_t *dmaSpec = dmaGetChannelSpecByTimer(timerHardware);

    if (dmaSpec) {
        dmaRef = dmaSpec->ref;
        dmaChannel = dmaSpec->channel;
    }
#else
    dmaRef = timerHardware->dmaRef;
    dmaChannel = timerHardware->dmaChannel;
#endif

#ifdef USE_DSHOT_DMAR
    if (useBurstDshot) {
#ifdef USE_DMA_SPEC
        uint8_t timnum = timerGetTIMNumber(timerHardware->tim);
        dmaoptValue_t dmaopt = timerUpConfig(timnum - 1)->dmaopt;
        const dmaChannelSpec_t *dmaChannelSpec = dmaGetChannelSpecByPeripheral(DMA_PERIPH_TIMUP, timnum - 1, dmaopt);
        dmaRef = dmaChannelSpec->ref;
        dmaChannel = dmaChannelSpec->channel;
#else
        dmaRef = timerHardware->dmaTimUPRef;
        dmaChannel = timerHardware->dmaTimUPChannel;
#endif
    }
#endif

    if (dmaRef == NULL) {
        return false;
    }

    motorDmaOutput_t * const motor = &dmaMotors[motorIndex];
    motor->timerHardware = timerHardware;

    TIM_TypeDef *timer = timerHardware->tim; // "timer" is confusing; "tim"?
    const IO_t motorIO = IOGetByTag(timerHardware->tag);
    uint8_t pupMode = (output & TIMER_OUTPUT_INVERTED) ? GPIO_PULLDOWN : GPIO_PULLUP;
#ifdef USE_DSHOT_TELEMETRY
    if (useDshotTelemetry) {
        output ^= TIMER_OUTPUT_INVERTED;
    }
#endif

    motor->iocfg = IO_CONFIG(GPIO_MODE_AF_PP, GPIO_SPEED_FREQ_VERY_HIGH, pupMode);
    const uint8_t timerIndex = getTimerIndex(timer);
    const bool configureTimer = (timerIndex == dmaMotorTimerCount - 1);

    IOInit(motorIO, OWNER_MOTOR, RESOURCE_INDEX(reorderedMotorIndex));
    IOConfigGPIOAF(motorIO, motor->iocfg, timerHardware->alternateFunction);

    // Configure time base

    if (configureTimer) {
        RCC_ClockCmd(timerRCC(timer), ENABLE);

        motor->TimHandle.Instance = timerHardware->tim; // timer
        motor->TimHandle.Init.Prescaler = (uint16_t)(lrintf((float) timerClock(timer) / getDshotHz(pwmProtocolType) + 0.01f) - 1);
        motor->TimHandle.Init.Period = pwmProtocolType == PWM_TYPE_PROSHOT1000 ? MOTOR_NIBBLE_LENGTH_PROSHOT : MOTOR_BITLENGTH;
        motor->TimHandle.Init.RepetitionCounter = 0;
        motor->TimHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
        motor->TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
        motor->TimHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

        result = HAL_TIM_PWM_Init(&motor->TimHandle);

        if (result != HAL_OK) {
            /* Initialization Error */
            return false;
        }
    }

/*
From LL version
chan oinv IDLE NIDLE POL  NPOL
N    I    -    Low   -    Low
N    -    -    Low   -    High
P    I    High -     Low  -
P    -    High -     High -
*/

    /* PWM mode 1 configuration */

    TIM_OC_InitTypeDef TIM_OCInitStructure;
    TIM_OCInitStructure.OCMode = TIM_OCMODE_PWM1;

    if (output & TIMER_OUTPUT_N_CHANNEL) {
        TIM_OCInitStructure.OCIdleState = TIM_OCIDLESTATE_RESET;
        TIM_OCInitStructure.OCPolarity = (output & TIMER_OUTPUT_INVERTED) ? TIM_OCPOLARITY_HIGH : TIM_OCPOLARITY_LOW;
        TIM_OCInitStructure.OCNIdleState = TIM_OCNIDLESTATE_RESET;
        TIM_OCInitStructure.OCNPolarity = (output & TIMER_OUTPUT_INVERTED) ? TIM_OCNPOLARITY_HIGH : TIM_OCNPOLARITY_LOW;
    } else {
        TIM_OCInitStructure.OCIdleState = TIM_OCIDLESTATE_SET;
        TIM_OCInitStructure.OCPolarity = (output & TIMER_OUTPUT_INVERTED) ? TIM_OCPOLARITY_LOW : TIM_OCPOLARITY_HIGH;
        TIM_OCInitStructure.OCNIdleState = TIM_OCNIDLESTATE_SET;
        TIM_OCInitStructure.OCNPolarity = (output & TIMER_OUTPUT_INVERTED) ? TIM_OCNPOLARITY_LOW : TIM_OCNPOLARITY_HIGH;
    }
    TIM_OCInitStructure.OCFastMode = TIM_OCFAST_DISABLE;
    TIM_OCInitStructure.Pulse = 0;

    result = HAL_TIM_PWM_ConfigChannel(&motor->TimHandle, &TIM_OCInitStructure, motor->timerHardware->channel);

    if (result != HAL_OK) {
        /* Configuration Error */
        return false;
    }

    // DMA setup

    motor->timer = &dmaMotorTimers[timerIndex];

#ifdef USE_DSHOT_DMAR
    if (useBurstDshot) {
        motor->timer->dmaBurstRef = dmaRef;

        if (!configureTimer) {
            motor->configured = true;
            return false;
        }
    } else
#endif
    {
        motor->timerDmaSource = timerDmaSource(timerHardware->channel);
        motor->timer->timerDmaSources |= motor->timerDmaSource;
        motor->timerDmaIndex = timerDmaIndex(timerHardware->channel);
    }

    dmaIdentifier_e identifier = dmaGetIdentifier(dmaRef);

#ifdef USE_DSHOT_DMAR
    if (useBurstDshot) {
        dmaInit(identifier, OWNER_TIMUP, timerGetTIMNumber(timerHardware->tim));
        dmaSetHandler(identifier, motor_DMA_IRQHandler, NVIC_PRIO_DSHOT_DMA, timerIndex);
    } else
#endif
    {
        dmaInit(identifier, OWNER_MOTOR, RESOURCE_INDEX(reorderedMotorIndex));
        dmaSetHandler(identifier, motor_DMA_IRQHandler, NVIC_PRIO_DSHOT_DMA, motorIndex);
    }

#ifdef USE_DSHOT_DMAR
    if (useBurstDshot) {
        motor->timer->hdma_tim.Init.Direction = DMA_MEMORY_TO_PERIPH;
        motor->timer->hdma_tim.Init.PeriphInc = DMA_PINC_DISABLE;
        motor->timer->hdma_tim.Init.MemInc = DMA_MINC_ENABLE;
        motor->timer->hdma_tim.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD ;
        motor->timer->hdma_tim.Init.MemDataAlignment = DMA_MDATAALIGN_WORD ;
        motor->timer->hdma_tim.Init.Mode = DMA_NORMAL;
        motor->timer->hdma_tim.Init.Priority = DMA_PRIORITY_HIGH;
#if !defined(STM32G4)
        motor->timer->hdma_tim.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        motor->timer->hdma_tim.Init.PeriphBurst = DMA_PBURST_SINGLE;
        motor->timer->hdma_tim.Init.MemBurst = DMA_MBURST_SINGLE;
        motor->timer->hdma_tim.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
#endif

        motor->timer->dmaBurstBuffer = &dshotBurstDmaBuffer[timerIndex][0];
        motor->timer->timHandle = motor->TimHandle;
        memset(motor->timer->dmaBurstBuffer, 0, DSHOT_DMA_BUFFER_SIZE * 4 * sizeof(uint32_t));

        /* Set hdma_tim instance */
        motor->timer->hdma_tim.Instance = (DMA_ARCH_TYPE *)dmaRef;
        motor->timer->hdma_tim.Init.Request = dmaChannel;

        /* Link hdma_tim to hdma[TIM_DMA_ID_UPDATE] (update) */
        __HAL_LINKDMA(&motor->timer->timHandle, hdma[TIM_DMA_ID_UPDATE], motor->timer->hdma_tim);

        /* Initialize TIMx DMA handle */
        result = HAL_DMA_Init(motor->timer->timHandle.hdma[TIM_DMA_ID_UPDATE]);

    } else
#endif
    {
        motor->hdma_tim.Init.Direction = DMA_MEMORY_TO_PERIPH;
        motor->hdma_tim.Init.PeriphInc = DMA_PINC_DISABLE;
        motor->hdma_tim.Init.MemInc = DMA_MINC_ENABLE;
        motor->hdma_tim.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD ;
        motor->hdma_tim.Init.MemDataAlignment = DMA_MDATAALIGN_WORD ;
        motor->hdma_tim.Init.Mode = DMA_NORMAL;
        motor->hdma_tim.Init.Priority = DMA_PRIORITY_HIGH;
#if !defined(STM32G4)
        motor->hdma_tim.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        motor->hdma_tim.Init.PeriphBurst = DMA_PBURST_SINGLE;
        motor->hdma_tim.Init.MemBurst = DMA_MBURST_SINGLE;
        motor->hdma_tim.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
#endif

        motor->dmaBuffer = &dshotDmaBuffer[motorIndex][0];
        motor->dmaBuffer[DSHOT_DMA_BUFFER_SIZE-2] = 0; // XXX Is this necessary? -> probably.
        motor->dmaBuffer[DSHOT_DMA_BUFFER_SIZE-1] = 0; // XXX Is this necessary?

        motor->hdma_tim.Instance = (DMA_ARCH_TYPE *)dmaRef;
        motor->hdma_tim.Init.Request = dmaChannel;

        /* Link hdma_tim to hdma[x] (channelx) */
        __HAL_LINKDMA(&motor->TimHandle, hdma[motor->timerDmaIndex], motor->hdma_tim);

        /* Initialize TIMx DMA handle */
        result = HAL_DMA_Init(motor->TimHandle.hdma[motor->timerDmaIndex]);

    }

    if (result != HAL_OK) {
        /* Initialization Error */
        return false;
    }

    // Start the timer channel now.
    // Enabling/disabling DMA request can restart a new cycle without PWM start/stop.

    if (motor->timerHardware->output & TIMER_OUTPUT_N_CHANNEL) {
        result = HAL_TIMEx_PWMN_Start(&motor->TimHandle, motor->timerHardware->channel);
    } else {
        result = HAL_TIM_PWM_Start(&motor->TimHandle, motor->timerHardware->channel);
    }

    if (result != HAL_OK) {
        /* Starting PWM generation Error */
        return false;
    }

    motor->configured = true;

    return true;
}
#endif
