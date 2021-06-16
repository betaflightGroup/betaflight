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

#if defined(USE_I2C) && !defined(SOFT_I2C)

#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/nvic.h"
#include "drivers/time.h"
#include "drivers/rcc.h"

#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_impl.h"
#include "drivers/bus_i2c_timing.h"

// Number of bits in I2C protocol phase
#define LEN_ADDR 7
#define LEN_RW 1
#define LEN_ACK 1

// Clock period in us during unstick transfer
#define UNSTICK_CLK_US 10

// Allow 500us for clock strech to complete during unstick
#define UNSTICK_CLK_STRETCH (500/UNSTICK_CLK_US)

static void i2cUnstick(IO_t scl, IO_t sda);

#define IOCFG_I2C_PU IO_CONFIG(GPIO_MODE_AF_OD, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_PULLUP)
#define IOCFG_I2C    IO_CONFIG(GPIO_MODE_AF_OD, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_NOPULL)

#define GPIO_AF4_I2C GPIO_AF4_I2C1

const i2cHardware_t i2cHardware[I2CDEV_COUNT] = {
#if defined(STM32F7)
#ifdef USE_I2C_DEVICE_1
    {
        .device = I2CDEV_1,
        .reg = I2C1,
        .sclPins = { I2CPINDEF(PB6), I2CPINDEF(PB8) },
        .sdaPins = { I2CPINDEF(PB7), I2CPINDEF(PB9) },
        .rcc = RCC_APB1(I2C1),
        .ev_irq = I2C1_EV_IRQn,
        .er_irq = I2C1_ER_IRQn,
    },
#endif
#ifdef USE_I2C_DEVICE_2
    {
        .device = I2CDEV_2,
        .reg = I2C2,
        .sclPins = { I2CPINDEF(PB10), I2CPINDEF(PF1) },
        .sdaPins = { I2CPINDEF(PB11), I2CPINDEF(PF0) },
        .rcc = RCC_APB1(I2C2),
        .ev_irq = I2C2_EV_IRQn,
        .er_irq = I2C2_ER_IRQn,
    },
#endif
#ifdef USE_I2C_DEVICE_3
    {
        .device = I2CDEV_3,
        .reg = I2C3,
        .sclPins = { I2CPINDEF(PA8) },
        .sdaPins = { I2CPINDEF(PC9) },
        .rcc = RCC_APB1(I2C3),
        .ev_irq = I2C3_EV_IRQn,
        .er_irq = I2C3_ER_IRQn,
    },
#endif
#ifdef USE_I2C_DEVICE_4
    {
        .device = I2CDEV_4,
        .reg = I2C4,
        .sclPins = { I2CPINDEF(PD12), I2CPINDEF(PF14) },
        .sdaPins = { I2CPINDEF(PD13), I2CPINDEF(PF15) },
        .rcc = RCC_APB1(I2C4),
        .ev_irq = I2C4_EV_IRQn,
        .er_irq = I2C4_ER_IRQn,
    },
#endif
#elif defined(STM32H7)
#ifdef USE_I2C_DEVICE_1
    {
        .device = I2CDEV_1,
        .reg = I2C1,
        .sclPins = { I2CPINDEF(PB6, GPIO_AF4_I2C1), I2CPINDEF(PB8, GPIO_AF4_I2C1) },
        .sdaPins = { I2CPINDEF(PB7, GPIO_AF4_I2C1), I2CPINDEF(PB9, GPIO_AF4_I2C1) },
        .rcc = RCC_APB1L(I2C1),
        .ev_irq = I2C1_EV_IRQn,
        .er_irq = I2C1_ER_IRQn,
    },
#endif
#ifdef USE_I2C_DEVICE_2
    {
        .device = I2CDEV_2,
        .reg = I2C2,
        .sclPins = { I2CPINDEF(PB10, GPIO_AF4_I2C2), I2CPINDEF(PF1, GPIO_AF4_I2C2) },
        .sdaPins = { I2CPINDEF(PB11, GPIO_AF4_I2C2), I2CPINDEF(PF0, GPIO_AF4_I2C2) },
        .rcc = RCC_APB1L(I2C2),
        .ev_irq = I2C2_EV_IRQn,
        .er_irq = I2C2_ER_IRQn,
    },
#endif
#ifdef USE_I2C_DEVICE_3
    {
        .device = I2CDEV_3,
        .reg = I2C3,
        .sclPins = { I2CPINDEF(PA8, GPIO_AF4_I2C3) },
        .sdaPins = { I2CPINDEF(PC9, GPIO_AF4_I2C3) },
        .rcc = RCC_APB1L(I2C3),
        .ev_irq = I2C3_EV_IRQn,
        .er_irq = I2C3_ER_IRQn,
    },
#endif
#ifdef USE_I2C_DEVICE_4
    {
        .device = I2CDEV_4,
        .reg = I2C4,
        .sclPins = { I2CPINDEF(PD12, GPIO_AF4_I2C4), I2CPINDEF(PF14, GPIO_AF4_I2C4), I2CPINDEF(PB6, GPIO_AF6_I2C4), I2CPINDEF(PB8, GPIO_AF6_I2C4) },
        .sdaPins = { I2CPINDEF(PD13, GPIO_AF4_I2C4), I2CPINDEF(PF15, GPIO_AF4_I2C4), I2CPINDEF(PB7, GPIO_AF6_I2C4), I2CPINDEF(PB9, GPIO_AF6_I2C4) },
        .rcc = RCC_APB4(I2C4),
        .ev_irq = I2C4_EV_IRQn,
        .er_irq = I2C4_ER_IRQn,
    },
#endif
#elif defined(STM32G4)
#ifdef USE_I2C_DEVICE_1
    {
        .device = I2CDEV_1,
        .reg = I2C1,

        // Some boards are overloading SWD pins with I2C1 for maximum pin utilization on 48-pin CE(U) packages.
        // Be carefull when using SWD on these boards if I2C1 pins are defined by default.

        .sclPins = { I2CPINDEF(PA13, GPIO_AF4_I2C1), I2CPINDEF(PA15, GPIO_AF4_I2C1), I2CPINDEF(PB6,  GPIO_AF4_I2C1), I2CPINDEF(PB8,  GPIO_AF4_I2C1), },
        .sdaPins = { I2CPINDEF(PA14, GPIO_AF4_I2C1), I2CPINDEF(PB7,  GPIO_AF4_I2C1), I2CPINDEF(PB9,  GPIO_AF4_I2C1), },
        .rcc = RCC_APB11(I2C1),
        .ev_irq = I2C1_EV_IRQn,
        .er_irq = I2C1_ER_IRQn,
    },
#endif
#ifdef USE_I2C_DEVICE_2
    {
        .device = I2CDEV_2,
        .reg = I2C2,
        .sclPins = { I2CPINDEF(PA9,  GPIO_AF4_I2C2), },
        .sdaPins = { I2CPINDEF(PA10, GPIO_AF4_I2C2), },
        .rcc = RCC_APB11(I2C2),
        .ev_irq = I2C2_EV_IRQn,
        .er_irq = I2C2_ER_IRQn,
    },
#endif
#ifdef USE_I2C_DEVICE_3
    {
        .device = I2CDEV_3,
        .reg = I2C3,
        .sclPins = { I2CPINDEF(PA10, GPIO_AF2_I2C3), I2CPINDEF(PC8,  GPIO_AF8_I2C3), },
        .sdaPins = { I2CPINDEF(PB5,  GPIO_AF8_I2C3), I2CPINDEF(PC9,  GPIO_AF8_I2C3), I2CPINDEF(PC11, GPIO_AF8_I2C3), },
        .rcc = RCC_APB11(I2C3),
        .ev_irq = I2C3_EV_IRQn,
        .er_irq = I2C3_ER_IRQn,
    },
#endif
#ifdef USE_I2C_DEVICE_4
    {
        .device = I2CDEV_4,
        .reg = I2C4,

        // Here, SWDIO(PA13) is overloaded with I2C4_SCL, too.
        // See comment in the I2C1 section above.

        .sclPins = { I2CPINDEF(PA13, GPIO_AF3_I2C4), I2CPINDEF(PB6,  GPIO_AF3_I2C4), I2CPINDEF(PC6,  GPIO_AF8_I2C4), },
        .sdaPins = { I2CPINDEF(PB7,  GPIO_AF4_I2C4), I2CPINDEF(PC7,  GPIO_AF8_I2C4), },
        .rcc = RCC_APB12(I2C4),
        .ev_irq = I2C4_EV_IRQn,
        .er_irq = I2C4_ER_IRQn,
    },
#endif
#endif
};

i2cDevice_t i2cDevice[I2CDEV_COUNT];

void i2cInit(I2CDevice device)
{
    if (device == I2CINVALID) {
        return;
    }

    i2cDevice_t *pDev = &i2cDevice[device];

    const i2cHardware_t *hardware = pDev->hardware;
    const IO_t scl = pDev->scl;
    const IO_t sda = pDev->sda;

    if (!hardware || IOGetOwner(scl) || IOGetOwner(sda)) {
        return;
    }

    IOInit(scl, OWNER_I2C_SCL, RESOURCE_INDEX(device));
    IOInit(sda, OWNER_I2C_SDA, RESOURCE_INDEX(device));

    // Enable RCC
    RCC_ClockCmd(hardware->rcc, ENABLE);

    i2cUnstick(scl, sda);

    // Init pins
#if defined(STM32F7)
    IOConfigGPIOAF(scl, pDev->pullUp ? IOCFG_I2C_PU : IOCFG_I2C, GPIO_AF4_I2C);
    IOConfigGPIOAF(sda, pDev->pullUp ? IOCFG_I2C_PU : IOCFG_I2C, GPIO_AF4_I2C);
#elif defined(STM32H7) || defined(STM32G4)
    IOConfigGPIOAF(scl, pDev->pullUp ? IOCFG_I2C_PU : IOCFG_I2C, pDev->sclAF);
    IOConfigGPIOAF(sda, pDev->pullUp ? IOCFG_I2C_PU : IOCFG_I2C, pDev->sdaAF);
#else
    IOConfigGPIO(scl, IOCFG_AF_OD);
    IOConfigGPIO(sda, IOCFG_AF_OD);
#endif

    // Init I2C peripheral

    I2C_HandleTypeDef *pHandle = &pDev->handle;

    memset(pHandle, 0, sizeof(*pHandle));

    pHandle->Instance = pDev->hardware->reg;

    // Compute TIMINGR value based on peripheral clock for this device instance

    uint32_t i2cPclk;

#if defined(STM32F7) || defined(STM32G4)
    // F7 Clock source configured in startup/system_stm32f7xx.c as:
    //   I2C1234 : PCLK1
    // G4 Clock source configured in startup/system_stm32g4xx.c as:
    //   I2C1234 : PCLK1
    i2cPclk = HAL_RCC_GetPCLK1Freq();
#elif defined(STM32H7)
    // Clock sources configured in startup/system_stm32h7xx.c as:
    //   I2C123 : D2PCLK1 (rcc_pclk1 for APB1)
    //   I2C4   : D3PCLK1 (rcc_pclk4 for APB4)
    i2cPclk = (pHandle->Instance == I2C4) ? HAL_RCCEx_GetD3PCLK1Freq() : HAL_RCC_GetPCLK1Freq();
#else
#error Unknown MCU type
#endif

    pHandle->Init.Timing = i2cClockTIMINGR(i2cPclk, pDev->clockSpeed, 0);

    pHandle->Init.OwnAddress1 = 0x0;
    pHandle->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    pHandle->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    pHandle->Init.OwnAddress2 = 0x0;
    pHandle->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    pHandle->Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    HAL_I2C_Init(pHandle);

    // Enable the Analog I2C Filter
    HAL_I2CEx_ConfigAnalogFilter(pHandle, I2C_ANALOGFILTER_ENABLE);

    // Setup interrupt handlers
    HAL_NVIC_SetPriority(hardware->er_irq, NVIC_PRIORITY_BASE(NVIC_PRIO_I2C_ER), NVIC_PRIORITY_SUB(NVIC_PRIO_I2C_ER));
    HAL_NVIC_EnableIRQ(hardware->er_irq);
    HAL_NVIC_SetPriority(hardware->ev_irq, NVIC_PRIORITY_BASE(NVIC_PRIO_I2C_EV), NVIC_PRIORITY_SUB(NVIC_PRIO_I2C_EV));
    HAL_NVIC_EnableIRQ(hardware->ev_irq);
}

static void i2cUnstick(IO_t scl, IO_t sda)
{
    int i;

    IOHi(scl);
    IOHi(sda);

    IOConfigGPIO(scl, IOCFG_OUT_OD);
    IOConfigGPIO(sda, IOCFG_OUT_OD);

    // Clock out, with SDA high:
    //   7 data bits
    //   1 READ bit
    //   1 cycle for the ACK
    for (i = 0; i < (LEN_ADDR + LEN_RW + LEN_ACK); i++) {
        // Wait for any clock stretching to finish
        int timeout = UNSTICK_CLK_STRETCH;
        while (!IORead(scl) && timeout) {
            delayMicroseconds(UNSTICK_CLK_US);
            timeout--;
        }

        // Pull low
        IOLo(scl); // Set bus low
        delayMicroseconds(UNSTICK_CLK_US/2);
        IOHi(scl); // Set bus high
        delayMicroseconds(UNSTICK_CLK_US/2);
    }

    // Generate a stop condition in case there was none
    IOLo(scl);
    delayMicroseconds(UNSTICK_CLK_US/2);
    IOLo(sda);
    delayMicroseconds(UNSTICK_CLK_US/2);

    IOHi(scl); // Set bus scl high
    delayMicroseconds(UNSTICK_CLK_US/2);
    IOHi(sda); // Set bus sda high
}

#endif
