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
#include "drivers/io.h"

#include "drivers/dma.h"
#include "drivers/timer.h"
#include "drivers/timer_def.h"

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {

    DEF_TIM(TIM11,  CH1,  PB9,  TIM_USE_PPM,   0, 0 ), // PPM IN
    DEF_TIM(TIM2,   CH2,  PB3,  TIM_USE_MOTOR, 0, 0 ), // S1_OUT - UP1-1
    DEF_TIM(TIM3,   CH1,  PB4,  TIM_USE_MOTOR, 0, 0 ), // S2_OUT - UP1-2
    DEF_TIM(TIM4,   CH1,  PB6,  TIM_USE_MOTOR, 0, 0 ), // S3_OUT - UP1-2
    DEF_TIM(TIM4,   CH2,  PB7,  TIM_USE_MOTOR, 0, 0 ), // S4_OUT - UP1-2
    DEF_TIM(TIM2,   CH1,  PA15, TIM_USE_ANY,   0, 0 ), // S5_OUT - UP1-1
    DEF_TIM(TIM10,  CH1,  PB8,  TIM_USE_ANY,   0, 0 ), // S6_OUT - DMA NONE

    DEF_TIM(TIM1,   CH1,  PA8,  TIM_USE_LED,   0, 0 ), // LED_STRIP - DMA2_ST3_CH6

};
