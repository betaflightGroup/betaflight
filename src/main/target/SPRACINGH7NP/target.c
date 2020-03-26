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
    DEF_TIM(TIM16, CH1, PB8,  TIM_USE_LED,                 0,  10, 10 ), // LED Strip
    DEF_TIM(TIM17, CH1, PB9,  TIM_USE_TRANSPONDER,         0,  11, 11 ), // Transponder

    // UART2TX and UART6RX+TX are on RX connector
    DEF_TIM(TIM8,  CH1, PC6,  TIM_USE_PPM | TIM_USE_PWM,   0,  0,  0 ), // Also UART6_TX
    DEF_TIM(TIM8,  CH2, PC7,  TIM_USE_PWM,                 0,  0,  0 ), // Also UART6_RX

    // 4in1 ESC connector on top of board; Pads/Picoblade connector on bottom of board.
    DEF_TIM(TIM5,  CH1, PA0,  TIM_USE_MOTOR,               0,  0,  0 ), // M1
    DEF_TIM(TIM5,  CH2, PA1,  TIM_USE_MOTOR,               0,  1,  0 ),
    DEF_TIM(TIM5,  CH3, PA2,  TIM_USE_MOTOR,               0,  2,  0 ),
    DEF_TIM(TIM5,  CH4, PA3,  TIM_USE_MOTOR,               0,  3,  0 ), // M4

    // Pads/Picoblade connector on bottom of board.
    DEF_TIM(TIM3,  CH1, PA6,  TIM_USE_MOTOR,               0,  4,  1 ), // M5 - Also TIM13/CH1
    DEF_TIM(TIM3,  CH2, PA7,  TIM_USE_MOTOR,               0,  5,  1 ), //      Also TIM14/CH1
    DEF_TIM(TIM3,  CH3, PB0,  TIM_USE_MOTOR,               0,  6,  1 ), //      Also TIM8/CH2_N
    DEF_TIM(TIM3,  CH4, PB1,  TIM_USE_MOTOR,               0,  7,  1 ), // M8 - Also TIM8/CH3_N

    // Pads on bottom of board.
    DEF_TIM(TIM12, CH2, PB15, TIM_USE_PWM,                 0,  0,  0 ), // Also USART1 RX
    DEF_TIM(TIM12, CH1, PB14, TIM_USE_PWM,                 0,  0,  0 ), // Also USART1 TX

    // Switched between Camera 1/2.
    DEF_TIM(TIM4,  CH3, PD14, TIM_USE_CAMERA_CONTROL,      0,  13,  13 ),

    DEF_TIM(TIM15, CH1, PE5,  TIM_USE_VIDEO_PIXEL,         0,  15, 15 ), // Pixel DMA
    DEF_TIM(TIM1,  CH1, PA8,  TIM_USE_VIDEO_SYNC,          0,  14, 14 ), // Sync
};

