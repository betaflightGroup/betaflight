
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

#pragma once

#define NOINLINE __attribute__((noinline))

#if !defined(UNIT_TEST) && !defined(SIMULATOR_BUILD) && !(USBD_DEBUG_LEVEL > 0)
#pragma GCC poison sprintf snprintf
#endif

#if defined(STM32G474xx)
#include "stm32g4xx.h"
#include "stm32g4xx_hal.h"
#include "system_stm32g4xx.h"

#include "stm32g4xx_ll_spi.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_dma.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_tim.h"
#include "stm32g4xx_ll_system.h"
#include "drivers/stm32g4xx_ll_ex.h"

// Chip Unique ID on G4
#define U_ID_0 (*(uint32_t*)UID_BASE)
#define U_ID_1 (*(uint32_t*)(UID_BASE + 4))
#define U_ID_2 (*(uint32_t*)(UID_BASE + 8))

#ifndef STM32G4
#define STM32G4
#endif

#elif defined(STM32H743xx) || defined(STM32H750xx) || defined(STM32H7A3xx) || defined(STM32H7A3xxQ) || defined(STM32H723xx) || defined(STM32H725xx) || defined(STM32H730xx)
#include "stm32h7xx.h"
#include "stm32h7xx_hal.h"
#include "system_stm32h7xx.h"

#include "stm32h7xx_ll_spi.h"
#include "stm32h7xx_ll_gpio.h"
#include "stm32h7xx_ll_dma.h"
#include "stm32h7xx_ll_rcc.h"
#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx_ll_tim.h"
#include "stm32h7xx_ll_system.h"
#include "drivers/stm32h7xx_ll_ex.h"

// Chip Unique ID on H7
#define U_ID_0 (*(uint32_t*)UID_BASE)
#define U_ID_1 (*(uint32_t*)(UID_BASE + 4))
#define U_ID_2 (*(uint32_t*)(UID_BASE + 8))

#ifndef STM32H7
#define STM32H7
#endif

#elif defined(STM32F722xx) || defined(STM32F745xx) || defined(STM32F746xx) || defined(STM32F765xx)
#include "stm32f7xx.h"
#include "stm32f7xx_hal.h"
#include "system_stm32f7xx.h"

#include "stm32f7xx_ll_spi.h"
#include "stm32f7xx_ll_gpio.h"
#include "stm32f7xx_ll_dma.h"
#include "stm32f7xx_ll_rcc.h"
#include "stm32f7xx_ll_bus.h"
#include "stm32f7xx_ll_tim.h"
#include "stm32f7xx_ll_system.h"
#include "drivers/stm32f7xx_ll_ex.h"

// Chip Unique ID on F7
#define U_ID_0 (*(uint32_t*)UID_BASE)
#define U_ID_1 (*(uint32_t*)(UID_BASE + 4))
#define U_ID_2 (*(uint32_t*)(UID_BASE + 8))

#ifndef STM32F7
#define STM32F7
#endif

#elif defined(STM32F40_41xxx) || defined (STM32F411xE) || defined (STM32F446xx)

#include "stm32f4xx.h"

// Chip Unique ID on F405
#define U_ID_0 (*(uint32_t*)0x1fff7a10)
#define U_ID_1 (*(uint32_t*)0x1fff7a14)
#define U_ID_2 (*(uint32_t*)0x1fff7a18)

#ifndef STM32F4
#define STM32F4
#endif

#elif defined(STM32F303xC)
#include "stm32f30x_conf.h"
#include "stm32f30x_rcc.h"
#include "stm32f30x_gpio.h"
#include "core_cm4.h"

// Chip Unique ID on F303
#define U_ID_0 (*(uint32_t*)0x1FFFF7AC)
#define U_ID_1 (*(uint32_t*)0x1FFFF7B0)
#define U_ID_2 (*(uint32_t*)0x1FFFF7B4)

#ifndef STM32F3
#define STM32F3
#endif

#elif defined(STM32F10X)

#include "stm32f10x_conf.h"
#include "stm32f10x_gpio.h"
#include "core_cm3.h"

// Chip Unique ID on F103
#define U_ID_0 (*(uint32_t*)0x1FFFF7E8)
#define U_ID_1 (*(uint32_t*)0x1FFFF7EC)
#define U_ID_2 (*(uint32_t*)0x1FFFF7F0)

#ifndef STM32F1
#define STM32F1
#endif

#elif defined(SIMULATOR_BUILD)

// Nop

#else // STM32F10X
#error "Invalid chipset specified. Update platform.h"
#endif

#include "target/common_pre.h"
#include "target.h"
#include "target/common_deprecated_post.h"
#include "target/common_post.h"
#include "target/common_defaults_post.h"
