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

#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/rcc.h"

#include "common/utils.h"

// io ports defs are stored in array by index now
struct ioPortDef_s {
    rccPeriphTag_t rcc;
};

#if defined(STM32F1)
const struct ioPortDef_s ioPortDefs[] = {
    { RCC_APB2(IOPA) },
    { RCC_APB2(IOPB) },
    { RCC_APB2(IOPC) },
    { RCC_APB2(IOPD) },
    { RCC_APB2(IOPE) },
{
#if defined (STM32F10X_HD) || defined (STM32F10X_XL) || defined (STM32F10X_HD_VL)
    RCC_APB2(IOPF),
#else
    0,
#endif
},
{
#if defined (STM32F10X_HD) || defined (STM32F10X_XL) || defined (STM32F10X_HD_VL)
    RCC_APB2(IOPG),
#else
    0,
#endif
},
};
#elif defined(STM32F3)
const struct ioPortDef_s ioPortDefs[] = {
    { RCC_AHB(GPIOA) },
    { RCC_AHB(GPIOB) },
    { RCC_AHB(GPIOC) },
    { RCC_AHB(GPIOD) },
    { RCC_AHB(GPIOE) },
    { RCC_AHB(GPIOF) },
};
#elif defined(STM32F4)
const struct ioPortDef_s ioPortDefs[] = {
    { RCC_AHB1(GPIOA) },
    { RCC_AHB1(GPIOB) },
    { RCC_AHB1(GPIOC) },
    { RCC_AHB1(GPIOD) },
    { RCC_AHB1(GPIOE) },
    { RCC_AHB1(GPIOF) },
};
#elif defined(STM32F7)
const struct ioPortDef_s ioPortDefs[] = {
    { RCC_AHB1(GPIOA) },
    { RCC_AHB1(GPIOB) },
    { RCC_AHB1(GPIOC) },
    { RCC_AHB1(GPIOD) },
    { RCC_AHB1(GPIOE) },
    { RCC_AHB1(GPIOF) },
};
#elif defined(STM32H7)
const struct ioPortDef_s ioPortDefs[] = {
    { RCC_AHB4(GPIOA) },
    { RCC_AHB4(GPIOB) },
    { RCC_AHB4(GPIOC) },
    { RCC_AHB4(GPIOD) },
    { RCC_AHB4(GPIOE) },
    { RCC_AHB4(GPIOF) },
    { RCC_AHB4(GPIOG) },
    { RCC_AHB4(GPIOH) },
#if !(defined(STM32H723xx) || defined(STM32H725xx) || defined(STM32H730xx))
    { RCC_AHB4(GPIOI) },
#endif
};
#elif defined(STM32G4)
const struct ioPortDef_s ioPortDefs[] = {
    { RCC_AHB2(GPIOA) },
    { RCC_AHB2(GPIOB) },
    { RCC_AHB2(GPIOC) },
    { RCC_AHB2(GPIOD) },
    { RCC_AHB2(GPIOE) },
    { RCC_AHB2(GPIOF) },
};
#endif

ioRec_t* IO_Rec(IO_t io)
{
    return io;
}

GPIO_TypeDef* IO_GPIO(IO_t io)
{
    const ioRec_t *ioRec = IO_Rec(io);
    return ioRec->gpio;
}

uint16_t IO_Pin(IO_t io)
{
    const ioRec_t *ioRec = IO_Rec(io);
    return ioRec->pin;
}

// port index, GPIOA == 0
int IO_GPIOPortIdx(IO_t io)
{
    if (!io) {
        return -1;
    }
    return (((size_t)IO_GPIO(io) - GPIOA_BASE) >> 10);     // ports are 0x400 apart
}

int IO_EXTI_PortSourceGPIO(IO_t io)
{
    return IO_GPIOPortIdx(io);
}

int IO_GPIO_PortSource(IO_t io)
{
    return IO_GPIOPortIdx(io);
}

// zero based pin index
int IO_GPIOPinIdx(IO_t io)
{
    if (!io) {
        return -1;
    }
    return 31 - __builtin_clz(IO_Pin(io));  // CLZ is a bit faster than FFS
}

int IO_EXTI_PinSource(IO_t io)
{
    return IO_GPIOPinIdx(io);
}

int IO_GPIO_PinSource(IO_t io)
{
    return IO_GPIOPinIdx(io);
}

// mask on stm32f103, bit index on stm32f303
uint32_t IO_EXTI_Line(IO_t io)
{
    if (!io) {
        return 0;
    }
#if defined(STM32F1) || defined(STM32F4) || defined(STM32F7) || defined(STM32H7) || defined(STM32G4)
    return 1 << IO_GPIOPinIdx(io);
#elif defined(STM32F3)
    return IO_GPIOPinIdx(io);
#elif defined(SIMULATOR_BUILD)
    return 0;
#else
# error "Unknown target type"
#endif
}

bool IORead(IO_t io)
{
    if (!io) {
        return false;
    }
#if defined(USE_FULL_LL_DRIVER)
    return (LL_GPIO_ReadInputPort(IO_GPIO(io)) & IO_Pin(io));
#elif defined(USE_HAL_DRIVER)
    return !! HAL_GPIO_ReadPin(IO_GPIO(io), IO_Pin(io));
#else
    return (IO_GPIO(io)->IDR & IO_Pin(io));
#endif
}

void IOWrite(IO_t io, bool hi)
{
    if (!io) {
        return;
    }
#if defined(USE_FULL_LL_DRIVER)
    LL_GPIO_SetOutputPin(IO_GPIO(io), IO_Pin(io) << (hi ? 0 : 16));
#elif defined(USE_HAL_DRIVER)
    HAL_GPIO_WritePin(IO_GPIO(io), IO_Pin(io), hi ? GPIO_PIN_SET : GPIO_PIN_RESET);
#elif defined(STM32F4)
    if (hi) {
        IO_GPIO(io)->BSRRL = IO_Pin(io);
    } else {
        IO_GPIO(io)->BSRRH = IO_Pin(io);
    }
#else
    IO_GPIO(io)->BSRR = IO_Pin(io) << (hi ? 0 : 16);
#endif
}

void IOHi(IO_t io)
{
    if (!io) {
        return;
    }
#if defined(USE_FULL_LL_DRIVER)
    LL_GPIO_SetOutputPin(IO_GPIO(io), IO_Pin(io));
#elif defined(USE_HAL_DRIVER)
    HAL_GPIO_WritePin(IO_GPIO(io), IO_Pin(io), GPIO_PIN_SET);
#elif defined(STM32F4)
    IO_GPIO(io)->BSRRL = IO_Pin(io);
#else
    IO_GPIO(io)->BSRR = IO_Pin(io);
#endif
}

void IOLo(IO_t io)
{
    if (!io) {
        return;
    }
#if defined(USE_FULL_LL_DRIVER)
    LL_GPIO_ResetOutputPin(IO_GPIO(io), IO_Pin(io));
#elif defined(USE_HAL_DRIVER)
    HAL_GPIO_WritePin(IO_GPIO(io), IO_Pin(io), GPIO_PIN_RESET);
#elif defined(STM32F4)
    IO_GPIO(io)->BSRRH = IO_Pin(io);
#else
    IO_GPIO(io)->BRR = IO_Pin(io);
#endif
}

void IOToggle(IO_t io)
{
    if (!io) {
        return;
    }

    uint32_t mask = IO_Pin(io);
    // Read pin state from ODR but write to BSRR because it only changes the pins
    // high in the mask value rather than all pins. XORing ODR directly risks
    // setting other pins incorrectly because it change all pins' state.
#if defined(USE_FULL_LL_DRIVER)
    if (LL_GPIO_ReadOutputPort(IO_GPIO(io)) & mask) {
        mask <<= 16;   // bit is set, shift mask to reset half
    }
    LL_GPIO_SetOutputPin(IO_GPIO(io), mask);
#elif defined(USE_HAL_DRIVER)
    UNUSED(mask);
    HAL_GPIO_TogglePin(IO_GPIO(io), IO_Pin(io));
#elif defined(STM32F4)
    if (IO_GPIO(io)->ODR & mask) {
        IO_GPIO(io)->BSRRH = mask;
    } else {
        IO_GPIO(io)->BSRRL = mask;
    }
#else
    if (IO_GPIO(io)->ODR & mask)
        mask <<= 16;   // bit is set, shift mask to reset half

    IO_GPIO(io)->BSRR = mask;
#endif
}

// claim IO pin, set owner and resources
void IOInit(IO_t io, resourceOwner_e owner, uint8_t index)
{
    if (!io) {
        return;
    }
    ioRec_t *ioRec = IO_Rec(io);
    ioRec->owner = owner;
    ioRec->index = index;
}

void IORelease(IO_t io)
{
    if (!io) {
        return;
    }
    ioRec_t *ioRec = IO_Rec(io);
    ioRec->owner = OWNER_FREE;
}

resourceOwner_e IOGetOwner(IO_t io)
{
    if (!io) {
        return OWNER_FREE;
    }
    const ioRec_t *ioRec = IO_Rec(io);
    return ioRec->owner;
}

bool IOIsFreeOrPreinit(IO_t io)
{
    resourceOwner_e owner = IOGetOwner(io);

    if (owner == OWNER_FREE || owner == OWNER_PREINIT) {
        return true;
    }

    return false;
}

#if defined(STM32F1)

void IOConfigGPIO(IO_t io, ioConfig_t cfg)
{
    if (!io) {
        return;
    }

    const rccPeriphTag_t rcc = ioPortDefs[IO_GPIOPortIdx(io)].rcc;
    RCC_ClockCmd(rcc, ENABLE);

    GPIO_InitTypeDef init = {
        .GPIO_Pin = IO_Pin(io),
        .GPIO_Speed = cfg & 0x03,
        .GPIO_Mode = cfg & 0x7c,
    };
    GPIO_Init(IO_GPIO(io), &init);
}

#elif defined(STM32H7) || defined(STM32G4)

void IOConfigGPIO(IO_t io, ioConfig_t cfg)
{
    IOConfigGPIOAF(io, cfg, 0);
}

void IOConfigGPIOAF(IO_t io, ioConfig_t cfg, uint8_t af)
{
    if (!io) {
        return;
    }

    rccPeriphTag_t rcc = ioPortDefs[IO_GPIOPortIdx(io)].rcc;
    RCC_ClockCmd(rcc, ENABLE);

    GPIO_InitTypeDef init = {
        .Pin = IO_Pin(io),
        .Mode = (cfg >> 0) & 0x13,
        .Speed = (cfg >> 2) & 0x03,
        .Pull = (cfg >> 5) & 0x03,
        .Alternate = af
    };

    HAL_GPIO_Init(IO_GPIO(io), &init);
}

#elif defined(STM32F7)

void IOConfigGPIO(IO_t io, ioConfig_t cfg)
{
    IOConfigGPIOAF(io, cfg, 0);
}

void IOConfigGPIOAF(IO_t io, ioConfig_t cfg, uint8_t af)
{
    if (!io) {
        return;
    }

    const rccPeriphTag_t rcc = ioPortDefs[IO_GPIOPortIdx(io)].rcc;
    RCC_ClockCmd(rcc, ENABLE);

    LL_GPIO_InitTypeDef init = {
        .Pin = IO_Pin(io),
        .Mode = (cfg >> 0) & 0x03,
        .Speed = (cfg >> 2) & 0x03,
        .OutputType = (cfg >> 4) & 0x01,
        .Pull = (cfg >> 5) & 0x03,
        .Alternate = af
    };

    LL_GPIO_Init(IO_GPIO(io), &init);
}

#elif defined(STM32F3) || defined(STM32F4)

void IOConfigGPIO(IO_t io, ioConfig_t cfg)
{
    if (!io) {
        return;
    }

    const rccPeriphTag_t rcc = ioPortDefs[IO_GPIOPortIdx(io)].rcc;
    RCC_ClockCmd(rcc, ENABLE);

    GPIO_InitTypeDef init = {
        .GPIO_Pin = IO_Pin(io),
        .GPIO_Mode = (cfg >> 0) & 0x03,
        .GPIO_Speed = (cfg >> 2) & 0x03,
        .GPIO_OType = (cfg >> 4) & 0x01,
        .GPIO_PuPd = (cfg >> 5) & 0x03,
    };
    GPIO_Init(IO_GPIO(io), &init);
}

void IOConfigGPIOAF(IO_t io, ioConfig_t cfg, uint8_t af)
{
    if (!io) {
        return;
    }

    const rccPeriphTag_t rcc = ioPortDefs[IO_GPIOPortIdx(io)].rcc;
    RCC_ClockCmd(rcc, ENABLE);
    GPIO_PinAFConfig(IO_GPIO(io), IO_GPIO_PinSource(io), af);

    GPIO_InitTypeDef init = {
        .GPIO_Pin = IO_Pin(io),
        .GPIO_Mode = (cfg >> 0) & 0x03,
        .GPIO_Speed = (cfg >> 2) & 0x03,
        .GPIO_OType = (cfg >> 4) & 0x01,
        .GPIO_PuPd = (cfg >> 5) & 0x03,
    };
    GPIO_Init(IO_GPIO(io), &init);
}
#endif

#if DEFIO_PORT_USED_COUNT > 0
static const uint16_t ioDefUsedMask[DEFIO_PORT_USED_COUNT] = { DEFIO_PORT_USED_LIST };
static const uint8_t ioDefUsedOffset[DEFIO_PORT_USED_COUNT] = { DEFIO_PORT_OFFSET_LIST };
#else
// Avoid -Wpedantic warning
static const uint16_t ioDefUsedMask[1] = {0};
static const uint8_t ioDefUsedOffset[1] = {0};
#endif
#if DEFIO_IO_USED_COUNT
ioRec_t ioRecs[DEFIO_IO_USED_COUNT];
#else
// Avoid -Wpedantic warning
ioRec_t ioRecs[1];
#endif

// initialize all ioRec_t structures from ROM
// currently only bitmask is used, this may change in future
void IOInitGlobal(void)
{
    ioRec_t *ioRec = ioRecs;

    for (unsigned port = 0; port < ARRAYLEN(ioDefUsedMask); port++) {
        for (unsigned pin = 0; pin < sizeof(ioDefUsedMask[0]) * 8; pin++) {
            if (ioDefUsedMask[port] & (1 << pin)) {
                ioRec->gpio = (GPIO_TypeDef *)(GPIOA_BASE + (port << 10));   // ports are 0x400 apart
                ioRec->pin = 1 << pin;
                ioRec++;
            }
        }
    }
}

IO_t IOGetByTag(ioTag_t tag)
{
    const int portIdx = DEFIO_TAG_GPIOID(tag);
    const int pinIdx = DEFIO_TAG_PIN(tag);

    if (portIdx < 0 || portIdx >= DEFIO_PORT_USED_COUNT) {
        return NULL;
    }
    // check if pin exists
    if (!(ioDefUsedMask[portIdx] & (1 << pinIdx))) {
        return NULL;
    }
    // count bits before this pin on single port
    int offset = __builtin_popcount(((1 << pinIdx) - 1) & ioDefUsedMask[portIdx]);
    // and add port offset
    offset += ioDefUsedOffset[portIdx];
    return ioRecs + offset;
}

void IOTraversePins(IOTraverseFuncPtr_t fnPtr)
{
    for (int i = 0; i < DEFIO_IO_USED_COUNT; i++) {
        fnPtr(&ioRecs[i]);
    }
}
