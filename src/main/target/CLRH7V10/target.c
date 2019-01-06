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
    DEF_TIM(TIM15, CH1, PE5,  TIM_USE_LED,                 0,  9,  0 ),
    DEF_TIM(TIM15, CH2, PE6,  TIM_USE_NONE,                0,  0,  0 ),

    DEF_TIM(TIM8,  CH3, PC8,  TIM_USE_CAMERA_CONTROL,      0,  0,  0 ),
    DEF_TIM(TIM8,  CH4, PC9,  TIM_USE_NONE,                0,  0,  0 ),

    DEF_TIM(TIM3,  CH1, PC6,  TIM_USE_NONE,                0,  0,  0 ), // UART6_TX backdoor
    DEF_TIM(TIM3,  CH2, PC7,  TIM_USE_PPM,                 0,  0,  0 ), // UART6_RX backdoor

    DEF_TIM(TIM2,  CH1, PA0,  TIM_USE_MOTOR,               0,  0,  0 ), // M1 TIM2 or TIM5
    DEF_TIM(TIM2,  CH2, PA1,  TIM_USE_MOTOR,               0,  1,  0 ), // M2
    DEF_TIM(TIM2,  CH3, PA2,  TIM_USE_MOTOR,               0,  2,  0 ), // M3
    DEF_TIM(TIM2,  CH4, PA3,  TIM_USE_MOTOR,               0,  3,  0 ), // M4
    DEF_TIM(TIM4,  CH1, PD12, TIM_USE_MOTOR,               0,  0,  0 ), // M5
    DEF_TIM(TIM4,  CH2, PD13, TIM_USE_MOTOR,               0,  0,  0 ), // M6
    DEF_TIM(TIM4,  CH3, PD14, TIM_USE_MOTOR,               0,  0,  0 ), // M7
    DEF_TIM(TIM4,  CH4, PD15, TIM_USE_MOTOR,               0,  0,  0 ), // M8
};
