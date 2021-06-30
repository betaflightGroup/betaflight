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

    DEF_TIM(TIM11,  CH1,  PB9,   TIM_USE_PPM,   0, 0 ), // PPM IN
    DEF_TIM(TIM8,   CH3,  PC8,   TIM_USE_MOTOR, 0, 0 ), // S1_OUT – UP2-1
    DEF_TIM(TIM8,   CH4,  PC9,   TIM_USE_MOTOR, 0, 0 ), // S2_OUT – UP2-1
    DEF_TIM(TIM1,   CH1,  PA8,   TIM_USE_MOTOR, 0, 0 ), // S3_OUT – UP2-5
    DEF_TIM(TIM1,   CH2,  PA9,   TIM_USE_MOTOR, 0, 0 ), // S4_OUT – UP2-5
    
    DEF_TIM(TIM4,   CH3,  PB8,   TIM_USE_CAMERA_CONTROL,   0, 0 ), // FC CAM – DMA1_ST7

    DEF_TIM(TIM2,   CH2,  PB3,   TIM_USE_LED,   0, 0 ), // LED_STRIP – DMA1_ST6

};
