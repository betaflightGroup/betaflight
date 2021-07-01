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

#ifdef USE_TIMER

#include "common/utils.h"

#include "drivers/dma.h"
#include "drivers/io.h"
#include "drivers/timer_def.h"

#include "stm32h7xx.h"
#include "rcc.h"
#include "timer.h"

const timerDef_t timerDefinitions[HARDWARE_TIMER_DEFINITION_COUNT] = {
    { .TIMx = TIM1,  .rcc = RCC_APB2(TIM1),   .inputIrq = TIM1_CC_IRQn},
    { .TIMx = TIM2,  .rcc = RCC_APB1L(TIM2),  .inputIrq = TIM2_IRQn},
    { .TIMx = TIM3,  .rcc = RCC_APB1L(TIM3),  .inputIrq = TIM3_IRQn},
    { .TIMx = TIM4,  .rcc = RCC_APB1L(TIM4),  .inputIrq = TIM4_IRQn},
    { .TIMx = TIM5,  .rcc = RCC_APB1L(TIM5),  .inputIrq = TIM5_IRQn},
    { .TIMx = TIM6,  .rcc = RCC_APB1L(TIM6),  .inputIrq = 0},
    { .TIMx = TIM7,  .rcc = RCC_APB1L(TIM7),  .inputIrq = 0},
    { .TIMx = TIM8,  .rcc = RCC_APB2(TIM8),   .inputIrq = TIM8_CC_IRQn},
    { .TIMx = TIM12, .rcc = RCC_APB1L(TIM12), .inputIrq = TIM8_BRK_TIM12_IRQn},
    { .TIMx = TIM13, .rcc = RCC_APB1L(TIM13), .inputIrq = TIM8_UP_TIM13_IRQn},
    { .TIMx = TIM14, .rcc = RCC_APB1L(TIM14), .inputIrq = TIM8_TRG_COM_TIM14_IRQn},
    { .TIMx = TIM15, .rcc = RCC_APB2(TIM15),  .inputIrq = TIM15_IRQn},
    { .TIMx = TIM16, .rcc = RCC_APB2(TIM16),  .inputIrq = TIM16_IRQn},
    { .TIMx = TIM17, .rcc = RCC_APB2(TIM17),  .inputIrq = TIM17_IRQn},
};

#if defined(USE_TIMER_MGMT)
const timerHardware_t fullTimerHardware[FULL_TIMER_CHANNEL_COUNT] = {
// Auto-generated from 'timer_def.h'
// Port A
    DEF_TIM(TIM2, CH1, PA0, TIM_USE_ANY, 0, 0, 0),
    DEF_TIM(TIM2, CH2, PA1, TIM_USE_ANY, 0, 0, 0),
    DEF_TIM(TIM2, CH3, PA2, TIM_USE_ANY, 0, 0, 0),
    DEF_TIM(TIM2, CH4, PA3, TIM_USE_ANY, 0, 0, 0),
    DEF_TIM(TIM2, CH1, PA5, TIM_USE_ANY, 0, 0, 0),
    DEF_TIM(TIM1, CH1N, PA7, TIM_USE_ANY, 0, 0, 0),
    DEF_TIM(TIM1, CH1, PA8, TIM_USE_ANY, 0, 0, 0),
    DEF_TIM(TIM1, CH2, PA9, TIM_USE_ANY, 0, 0, 0),
    DEF_TIM(TIM1, CH3, PA10, TIM_USE_ANY, 0, 0, 0),
    DEF_TIM(TIM1, CH4, PA11, TIM_USE_ANY, 0, 0, 0),
    DEF_TIM(TIM2, CH1, PA15, TIM_USE_ANY, 0, 0, 0),

    DEF_TIM(TIM5, CH1, PA0, TIM_USE_ANY, 0, 0, 0),
    DEF_TIM(TIM5, CH2, PA1, TIM_USE_ANY, 0, 0, 0),
    DEF_TIM(TIM5, CH3, PA2, TIM_USE_ANY, 0, 0, 0),
    DEF_TIM(TIM5, CH4, PA3, TIM_USE_ANY, 0, 0, 0),
    DEF_TIM(TIM3, CH1, PA6, TIM_USE_ANY, 0, 0, 0),
    DEF_TIM(TIM3, CH2, PA7, TIM_USE_ANY, 0, 0, 0),

    DEF_TIM(TIM8, CH1N, PA5, TIM_USE_ANY, 0, 0, 0),
    DEF_TIM(TIM8, CH1N, PA7, TIM_USE_ANY, 0, 0, 0),

    DEF_TIM(TIM13, CH1, PA6, TIM_USE_ANY, 0, 0, 0),
    DEF_TIM(TIM14, CH1, PA7, TIM_USE_ANY, 0, 0, 0),

    DEF_TIM(TIM15, CH1N, PA1, TIM_USE_ANY, 0, 0, 0),
    DEF_TIM(TIM15, CH1, PA2, TIM_USE_ANY, 0, 0, 0),
    DEF_TIM(TIM15, CH2, PA2, TIM_USE_ANY, 0, 0, 0),

// Port B
    DEF_TIM(TIM1, CH2N, PB0, TIM_USE_ANY, 0, 0, 0),
    DEF_TIM(TIM1, CH3N, PB1, TIM_USE_ANY, 0, 0, 0),
    DEF_TIM(TIM2, CH2, PB3, TIM_USE_ANY, 0, 0, 0),
    DEF_TIM(TIM16, CH1N, PB6, TIM_USE_ANY, 0, 0, 0),
    DEF_TIM(TIM17, CH1N, PB7, TIM_USE_ANY, 0, 0, 0),
    DEF_TIM(TIM16, CH1, PB8, TIM_USE_ANY, 0, 0, 0),
    DEF_TIM(TIM17, CH1, PB9, TIM_USE_ANY, 0, 0, 0),
    DEF_TIM(TIM2, CH3, PB10, TIM_USE_ANY, 0, 0, 0),
    DEF_TIM(TIM2, CH4, PB11, TIM_USE_ANY, 0, 0, 0),
    DEF_TIM(TIM1, CH1N, PB13, TIM_USE_ANY, 0, 0, 0),
    DEF_TIM(TIM1, CH2N, PB14, TIM_USE_ANY, 0, 0, 0),
    DEF_TIM(TIM1, CH3N, PB15, TIM_USE_ANY, 0, 0, 0),

    DEF_TIM(TIM3, CH3, PB0, TIM_USE_ANY, 0, 0, 0),
    DEF_TIM(TIM3, CH4, PB1, TIM_USE_ANY, 0, 0, 0),
    DEF_TIM(TIM3, CH1, PB4, TIM_USE_ANY, 0, 0, 0),
    DEF_TIM(TIM3, CH2, PB5, TIM_USE_ANY, 0, 0, 0),
    DEF_TIM(TIM4, CH1, PB6, TIM_USE_ANY, 0, 0, 0),
    DEF_TIM(TIM4, CH2, PB7, TIM_USE_ANY, 0, 0, 0),
    DEF_TIM(TIM4, CH3, PB8, TIM_USE_ANY, 0, 0, 0),
    DEF_TIM(TIM4, CH4, PB9, TIM_USE_ANY, 0, 0, 0),

    DEF_TIM(TIM12, CH1, PB14, TIM_USE_ANY, 0, 0, 0),
    DEF_TIM(TIM12, CH2, PB15, TIM_USE_ANY, 0, 0, 0),

// Port C
    DEF_TIM(TIM3, CH1, PC6, TIM_USE_ANY, 0, 0, 0),
    DEF_TIM(TIM3, CH2, PC7, TIM_USE_ANY, 0, 0, 0),
    DEF_TIM(TIM3, CH3, PC8, TIM_USE_ANY, 0, 0, 0),
    DEF_TIM(TIM3, CH4, PC9, TIM_USE_ANY, 0, 0, 0),

    DEF_TIM(TIM8, CH1, PC6, TIM_USE_ANY, 0, 0, 0),
    DEF_TIM(TIM8, CH2, PC7, TIM_USE_ANY, 0, 0, 0),
    DEF_TIM(TIM8, CH3, PC8, TIM_USE_ANY, 0, 0, 0),
    DEF_TIM(TIM8, CH4, PC9, TIM_USE_ANY, 0, 0, 0),

// Port D
    DEF_TIM(TIM4, CH1, PD12, TIM_USE_ANY, 0, 0, 0),
    DEF_TIM(TIM4, CH2, PD13, TIM_USE_ANY, 0, 0, 0),
    DEF_TIM(TIM4, CH3, PD14, TIM_USE_ANY, 0, 0, 0),
    DEF_TIM(TIM4, CH4, PD15, TIM_USE_ANY, 0, 0, 0),

// Port E
    DEF_TIM(TIM1, CH1N, PE8, TIM_USE_ANY, 0, 0, 0),
    DEF_TIM(TIM1, CH1, PE9, TIM_USE_ANY, 0, 0, 0),
    DEF_TIM(TIM1, CH2N, PE10, TIM_USE_ANY, 0, 0, 0),
    DEF_TIM(TIM1, CH2, PE11, TIM_USE_ANY, 0, 0, 0),
    DEF_TIM(TIM1, CH3N, PE12, TIM_USE_ANY, 0, 0, 0),
    DEF_TIM(TIM1, CH3, PE13, TIM_USE_ANY, 0, 0, 0),
    DEF_TIM(TIM1, CH4, PE14, TIM_USE_ANY, 0, 0, 0),

    DEF_TIM(TIM15, CH1N, PE4, TIM_USE_ANY, 0, 0, 0),
    DEF_TIM(TIM15, CH1, PE5, TIM_USE_ANY, 0, 0, 0),
    DEF_TIM(TIM15, CH2, PE6, TIM_USE_ANY, 0, 0, 0),

// Port F
    DEF_TIM(TIM16, CH1, PF6, TIM_USE_ANY, 0, 0, 0),
    DEF_TIM(TIM17, CH1, PF7, TIM_USE_ANY, 0, 0, 0),
    DEF_TIM(TIM16, CH1N, PF8, TIM_USE_ANY, 0, 0, 0),
    DEF_TIM(TIM17, CH1N, PF9, TIM_USE_ANY, 0, 0, 0),

    DEF_TIM(TIM13, CH1N, PF8, TIM_USE_ANY, 0, 0, 0),
    DEF_TIM(TIM14, CH1N, PF9, TIM_USE_ANY, 0, 0, 0),

// Port H
// Port H is not available for LPQFP-100 or 144 and TFBGA-100 package
//    DEF_TIM(TIM12, CH1, PH6, TIM_USE_ANY, 0, 0, 0),
//    DEF_TIM(TIM12, CH2, PH9, TIM_USE_ANY, 0, 0, 0),
//    DEF_TIM(TIM5, CH1, PH10, TIM_USE_ANY, 0, 0, 0),
//    DEF_TIM(TIM5, CH2, PH11, TIM_USE_ANY, 0, 0, 0),
//    DEF_TIM(TIM5, CH3, PH12, TIM_USE_ANY, 0, 0, 0),
//    DEF_TIM(TIM8, CH1N, PH13, TIM_USE_ANY, 0, 0, 0),
//    DEF_TIM(TIM8, CH2N, PH14, TIM_USE_ANY, 0, 0, 0),
//    DEF_TIM(TIM8, CH3N, PH15, TIM_USE_ANY, 0, 0, 0),
};
#endif


uint32_t timerClock(TIM_TypeDef *tim)
{
    int timpre;
    uint32_t pclk;
    uint32_t ppre;

    // Implement the table:
    // RM0433 (Rev 6) Table 52.
    // RM0455 (Rev 3) Table 55.
    // "Ratio between clock timer and pclk"
    // (Tables are the same, just D2 or CD difference)

#if defined(STM32H743xx) || defined(STM32H750xx) || defined(STM32H723xx) || defined(STM32H725xx) || defined(STM32H730xx)
#define PERIPH_PRESCALER(bus) ((RCC->D2CFGR & RCC_D2CFGR_D2PPRE ## bus) >> RCC_D2CFGR_D2PPRE ## bus ## _Pos)
#elif defined(STM32H7A3xx) || defined(STM32H7A3xxQ)
#define PERIPH_PRESCALER(bus) ((RCC->CDCFGR2 & RCC_CDCFGR2_CDPPRE ## bus) >> RCC_CDCFGR2_CDPPRE ## bus ## _Pos)
#else
#error Unknown MCU type
#endif

    if (tim == TIM1 || tim == TIM8 || tim == TIM15 || tim == TIM16 || tim == TIM17) {
        // Timers on APB2
        pclk = HAL_RCC_GetPCLK2Freq();
        ppre = PERIPH_PRESCALER(2);
    } else {
        // Timers on APB1
        pclk = HAL_RCC_GetPCLK1Freq();
        ppre = PERIPH_PRESCALER(1);
    }

    timpre = (RCC->CFGR & RCC_CFGR_TIMPRE) ? 1 : 0;

    int index = (timpre << 3) | ppre;

    static uint8_t periphToKernel[16] = { // The mutiplier table
        1, 1, 1, 1, 2, 2, 2, 2, // TIMPRE = 0
        1, 1, 1, 1, 2, 4, 4, 4  // TIMPRE = 1
    };

    return pclk * periphToKernel[index];

#undef PERIPH_PRESCALER
}
#endif
