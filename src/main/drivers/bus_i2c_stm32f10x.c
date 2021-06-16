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
#include <stdlib.h>
#include <string.h>

#include "platform.h"

#if defined(USE_I2C) && !defined(SOFT_I2C)

#include "drivers/io.h"
#include "drivers/time.h"
#include "drivers/nvic.h"
#include "drivers/rcc.h"

#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_impl.h"

// Number of bits in I2C protocol phase
#define LEN_ADDR 7
#define LEN_RW 1
#define LEN_ACK 1

// Clock period in us during unstick transfer
#define UNSTICK_CLK_US 10

// Allow 500us for clock strech to complete during unstick
#define UNSTICK_CLK_STRETCH (500/UNSTICK_CLK_US)

static void i2c_er_handler(I2CDevice device);
static void i2c_ev_handler(I2CDevice device);
static void i2cUnstick(IO_t scl, IO_t sda);

#ifdef STM32F4
#define IOCFG_I2C_PU IO_CONFIG(GPIO_Mode_AF, 0, GPIO_OType_OD, GPIO_PuPd_UP)
#define IOCFG_I2C    IO_CONFIG(GPIO_Mode_AF, 0, GPIO_OType_OD, GPIO_PuPd_NOPULL)
#else // STM32F4
#define IOCFG_I2C   IO_CONFIG(GPIO_Mode_AF_OD, GPIO_Speed_50MHz)
#endif

const i2cHardware_t i2cHardware[I2CDEV_COUNT] = {
#ifdef USE_I2C_DEVICE_1
    {
        .device = I2CDEV_1,
        .reg = I2C1,
        .sclPins = {
            I2CPINDEF(PB6, GPIO_AF_I2C1),
            I2CPINDEF(PB8, GPIO_AF_I2C1),
        },
        .sdaPins = {
            I2CPINDEF(PB7, GPIO_AF_I2C1),
            I2CPINDEF(PB9, GPIO_AF_I2C1),
        },
        .rcc = RCC_APB1(I2C1),
        .ev_irq = I2C1_EV_IRQn,
        .er_irq = I2C1_ER_IRQn,
    },
#endif
#ifdef USE_I2C_DEVICE_2
    {
        .device = I2CDEV_2,
        .reg = I2C2,
        .sclPins = {
            I2CPINDEF(PB10, GPIO_AF_I2C2),
            I2CPINDEF(PF1,  GPIO_AF_I2C2),
        },
        .sdaPins = {
#if defined(STM32F446xx)
            I2CPINDEF(PC12, GPIO_AF_I2C2),
#else
            I2CPINDEF(PB11, GPIO_AF_I2C2),
#endif
            I2CPINDEF(PF0,  GPIO_AF_I2C2),

#if defined(STM32F40_41xxx) || defined (STM32F411xE)
            // STM32F401xx/STM32F410xx/STM32F411xE/STM32F412xG
            I2CPINDEF(PB3,  GPIO_AF9_I2C2),
            I2CPINDEF(PB9,  GPIO_AF9_I2C2),
#endif
        },
        .rcc = RCC_APB1(I2C2),
        .ev_irq = I2C2_EV_IRQn,
        .er_irq = I2C2_ER_IRQn,
    },
#endif
#ifdef USE_I2C_DEVICE_3
    {
        .device = I2CDEV_3,
        .reg = I2C3,
        .sclPins = {
            I2CPINDEF(PA8, GPIO_AF_I2C3),
        },
        .sdaPins = {
            I2CPINDEF(PC9, GPIO_AF_I2C3),

#if defined(STM32F40_41xxx) || defined (STM32F411xE)
            // STM32F401xx/STM32F410xx/STM32F411xE/STM32F412xG
            I2CPINDEF(PB4, GPIO_AF9_I2C3),
            I2CPINDEF(PB8, GPIO_AF9_I2C3),
#endif
        },
        .rcc = RCC_APB1(I2C3),
        .ev_irq = I2C3_EV_IRQn,
        .er_irq = I2C3_ER_IRQn,
    },
#endif
};

i2cDevice_t i2cDevice[I2CDEV_COUNT];

static volatile uint16_t i2cErrorCount = 0;

void I2C1_ER_IRQHandler(void) {
    i2c_er_handler(I2CDEV_1);
}

void I2C1_EV_IRQHandler(void) {
    i2c_ev_handler(I2CDEV_1);
}

void I2C2_ER_IRQHandler(void) {
    i2c_er_handler(I2CDEV_2);
}

void I2C2_EV_IRQHandler(void) {
    i2c_ev_handler(I2CDEV_2);
}

#ifdef STM32F4
void I2C3_ER_IRQHandler(void) {
    i2c_er_handler(I2CDEV_3);
}

void I2C3_EV_IRQHandler(void) {
    i2c_ev_handler(I2CDEV_3);
}
#endif

static bool i2cHandleHardwareFailure(I2CDevice device)
{
    i2cErrorCount++;
    // reinit peripheral + clock out garbage
    i2cInit(device);
    return false;
}

bool i2cWriteBuffer(I2CDevice device, uint8_t addr_, uint8_t reg_, uint8_t len_, uint8_t *data)
{
    if (device == I2CINVALID || device > I2CDEV_COUNT) {
        return false;
    }

    I2C_TypeDef *I2Cx = i2cDevice[device].reg;

    if (!I2Cx) {
        return false;
    }

    i2cState_t *state = &i2cDevice[device].state;
    if (state->busy) {
        return false;
    }

    timeUs_t timeoutStartUs = microsISR();

    state->addr = addr_ << 1;
    state->reg = reg_;
    state->writing = 1;
    state->reading = 0;
    state->write_p = data;
    state->read_p = data;
    state->bytes = len_;
    state->busy = 1;
    state->error = false;

    if (!(I2Cx->CR2 & I2C_IT_EVT)) {                                    // if we are restarting the driver
        if (!(I2Cx->CR1 & I2C_CR1_START)) {                             // ensure sending a start
            while (I2Cx->CR1 & I2C_CR1_STOP) {                          // wait for any stop to finish sending
                if (cmpTimeUs(microsISR(), timeoutStartUs) >= I2C_TIMEOUT_US) {
                    return i2cHandleHardwareFailure(device);
                }
            }
            I2C_GenerateSTART(I2Cx, ENABLE);                            // send the start for the new job
        }
        I2C_ITConfig(I2Cx, I2C_IT_EVT | I2C_IT_ERR, ENABLE);            // allow the interrupts to fire off again
    }

    return true;
}

bool i2cBusy(I2CDevice device, bool *error)
{
    i2cState_t *state = &i2cDevice[device].state;

    if (error) {
        *error = state->error;
    }
    return state->busy;
}

bool i2cWait(I2CDevice device)
{
    i2cState_t *state = &i2cDevice[device].state;
    timeUs_t timeoutStartUs = microsISR();

    while (state->busy) {
        if (cmpTimeUs(microsISR(), timeoutStartUs) >= I2C_TIMEOUT_US) {
            return i2cHandleHardwareFailure(device) && i2cWait(device);
        }
    }

    return !(state->error);
}

bool i2cWrite(I2CDevice device, uint8_t addr_, uint8_t reg_, uint8_t data)
{
    return i2cWriteBuffer(device, addr_, reg_, 1, &data) && i2cWait(device);
}

bool i2cReadBuffer(I2CDevice device, uint8_t addr_, uint8_t reg_, uint8_t len, uint8_t* buf)
{
    if (device == I2CINVALID || device > I2CDEV_COUNT) {
        return false;
    }

    I2C_TypeDef *I2Cx = i2cDevice[device].reg;
    if (!I2Cx) {
        return false;
    }

    i2cState_t *state = &i2cDevice[device].state;
    if (state->busy) {
        return false;
    }

    timeUs_t timeoutStartUs = microsISR();

    state->addr = addr_ << 1;
    state->reg = reg_;
    state->writing = 0;
    state->reading = 1;
    state->read_p = buf;
    state->write_p = buf;
    state->bytes = len;
    state->busy = 1;
    state->error = false;

    if (!(I2Cx->CR2 & I2C_IT_EVT)) {                                    // if we are restarting the driver
        if (!(I2Cx->CR1 & I2C_CR1_START)) {                             // ensure sending a start
            while (I2Cx->CR1 & I2C_CR1_STOP) {                          // wait for any stop to finish sending
                if (cmpTimeUs(microsISR(), timeoutStartUs) >= I2C_TIMEOUT_US) {
                    return i2cHandleHardwareFailure(device);
                }
            }
            I2C_GenerateSTART(I2Cx, ENABLE);                            // send the start for the new job
        }
        I2C_ITConfig(I2Cx, I2C_IT_EVT | I2C_IT_ERR, ENABLE);            // allow the interrupts to fire off again
    }

    return true;
}

bool i2cRead(I2CDevice device, uint8_t addr_, uint8_t reg_, uint8_t len, uint8_t* buf)
{
    return i2cReadBuffer(device, addr_, reg_, len, buf) && i2cWait(device);
}

static void i2c_er_handler(I2CDevice device) {

    I2C_TypeDef *I2Cx = i2cDevice[device].hardware->reg;

    i2cState_t *state = &i2cDevice[device].state;

    // Read the I2C1 status register
    volatile uint32_t SR1Register = I2Cx->SR1;

    if (SR1Register & (I2C_SR1_BERR | I2C_SR1_ARLO | I2C_SR1_AF | I2C_SR1_OVR)) // an error
        state->error = true;

    // If AF, BERR or ARLO, abandon the current job and commence new if there are jobs
    if (SR1Register & (I2C_SR1_BERR | I2C_SR1_ARLO | I2C_SR1_AF)) {
        (void)I2Cx->SR2;                                                        // read second status register to clear ADDR if it is set (note that BTF will not be set after a NACK)
        I2C_ITConfig(I2Cx, I2C_IT_BUF, DISABLE);                                // disable the RXNE/TXE interrupt - prevent the ISR tailchaining onto the ER (hopefully)
        if (!(SR1Register & I2C_SR1_ARLO) && !(I2Cx->CR1 & I2C_CR1_STOP)) {     // if we dont have an ARLO error, ensure sending of a stop
            if (I2Cx->CR1 & I2C_CR1_START) {                                    // We are currently trying to send a start, this is very bad as start, stop will hang the peripheral
                while (I2Cx->CR1 & I2C_CR1_START) {; }                         // wait for any start to finish sending
                I2C_GenerateSTOP(I2Cx, ENABLE);                                 // send stop to finalise bus transaction
                while (I2Cx->CR1 & I2C_CR1_STOP) {; }                          // wait for stop to finish sending
                i2cInit(device);                                                // reset and configure the hardware
            }
            else {
                I2C_GenerateSTOP(I2Cx, ENABLE);                                 // stop to free up the bus
                I2C_ITConfig(I2Cx, I2C_IT_EVT | I2C_IT_ERR, DISABLE);           // Disable EVT and ERR interrupts while bus inactive
            }
        }
    }
    I2Cx->SR1 &= ~(I2C_SR1_BERR | I2C_SR1_ARLO | I2C_SR1_AF | I2C_SR1_OVR);     // reset all the error bits to clear the interrupt
    state->busy = 0;
}

void i2c_ev_handler(I2CDevice device) {

    I2C_TypeDef *I2Cx = i2cDevice[device].hardware->reg;

    i2cState_t *state = &i2cDevice[device].state;

    static uint8_t subaddress_sent, final_stop;                                 // flag to indicate if subaddess sent, flag to indicate final bus condition
    static int8_t index;                                                        // index is signed -1 == send the subaddress
    uint8_t SReg_1 = I2Cx->SR1;                                                 // read the status register here

    if (SReg_1 & I2C_SR1_SB) {                                                  // we just sent a start - EV5 in ref manual
        I2Cx->CR1 &= ~I2C_CR1_POS;                                              // reset the POS bit so ACK/NACK applied to the current byte
        I2C_AcknowledgeConfig(I2Cx, ENABLE);                                    // make sure ACK is on
        index = 0;                                                              // reset the index
        if (state->reading && (subaddress_sent || 0xFF == state->reg)) {          // we have sent the subaddr
            subaddress_sent = 1;                                                // make sure this is set in case of no subaddress, so following code runs correctly
            if (state->bytes == 2)
                I2Cx->CR1 |= I2C_CR1_POS;                                       // set the POS bit so NACK applied to the final byte in the two byte read
            I2C_Send7bitAddress(I2Cx, state->addr, I2C_Direction_Receiver);      // send the address and set hardware mode
        }
        else {                                                                // direction is Tx, or we havent sent the sub and rep start
            I2C_Send7bitAddress(I2Cx, state->addr, I2C_Direction_Transmitter);   // send the address and set hardware mode
            if (state->reg != 0xFF)                                              // 0xFF as subaddress means it will be ignored, in Tx or Rx mode
                index = -1;                                                     // send a subaddress
        }
    }
    else if (SReg_1 & I2C_SR1_ADDR) {                                         // we just sent the address - EV6 in ref manual
        // Read SR1,2 to clear ADDR
        __DMB();                                                                // memory fence to control hardware
        if (state->bytes == 1 && state->reading && subaddress_sent) {             // we are receiving 1 byte - EV6_3
            I2C_AcknowledgeConfig(I2Cx, DISABLE);                               // turn off ACK
            __DMB();
            (void)I2Cx->SR2;                                                    // clear ADDR after ACK is turned off
            I2C_GenerateSTOP(I2Cx, ENABLE);                                     // program the stop
            final_stop = 1;
            I2C_ITConfig(I2Cx, I2C_IT_BUF, ENABLE);                     // allow us to have an EV7
        }
        else {                                                        // EV6 and EV6_1
            (void)I2Cx->SR2;                                            // clear the ADDR here
            __DMB();
            if (state->bytes == 2 && state->reading && subaddress_sent) {         // rx 2 bytes - EV6_1
                I2C_AcknowledgeConfig(I2Cx, DISABLE);                           // turn off ACK
                I2C_ITConfig(I2Cx, I2C_IT_BUF, DISABLE);                        // disable TXE to allow the buffer to fill
            }
            else if (state->bytes == 3 && state->reading && subaddress_sent)    // rx 3 bytes
                I2C_ITConfig(I2Cx, I2C_IT_BUF, DISABLE);                        // make sure RXNE disabled so we get a BTF in two bytes time
            else                                                                // receiving greater than three bytes, sending subaddress, or transmitting
                I2C_ITConfig(I2Cx, I2C_IT_BUF, ENABLE);
        }
    }
    else if (SReg_1 & I2C_SR1_BTF) {                                  // Byte transfer finished - EV7_2, EV7_3 or EV8_2
        final_stop = 1;
        if (state->reading && subaddress_sent) {                         // EV7_2, EV7_3
            if (state->bytes > 2) {                                      // EV7_2
                I2C_AcknowledgeConfig(I2Cx, DISABLE);                   // turn off ACK
                state->read_p[index++] = (uint8_t)I2Cx->DR;              // read data N-2
                I2C_GenerateSTOP(I2Cx, ENABLE);                         // program the Stop
                final_stop = 1;                                         // required to fix hardware
                state->read_p[index++] = (uint8_t)I2Cx->DR;              // read data N - 1
                I2C_ITConfig(I2Cx, I2C_IT_BUF, ENABLE);                 // enable TXE to allow the final EV7
            }
            else {                                                    // EV7_3
                if (final_stop)
                    I2C_GenerateSTOP(I2Cx, ENABLE);                     // program the Stop
                else
                    I2C_GenerateSTART(I2Cx, ENABLE);                    // program a rep start
                state->read_p[index++] = (uint8_t)I2Cx->DR;                    // read data N - 1
                state->read_p[index++] = (uint8_t)I2Cx->DR;                    // read data N
                index++;                                                // to show job completed
            }
        }
        else {                                                        // EV8_2, which may be due to a subaddress sent or a write completion
            if (subaddress_sent || (state->writing)) {
                if (final_stop)
                    I2C_GenerateSTOP(I2Cx, ENABLE);                     // program the Stop
                else
                    I2C_GenerateSTART(I2Cx, ENABLE);                    // program a rep start
                index++;                                                // to show that the job is complete
            }
            else {                                                    // We need to send a subaddress
                I2C_GenerateSTART(I2Cx, ENABLE);                        // program the repeated Start
                subaddress_sent = 1;                                    // this is set back to zero upon completion of the current task
            }
        }
        // we must wait for the start to clear, otherwise we get constant BTF
        while (I2Cx->CR1 & I2C_CR1_START) {; }
    }
    else if (SReg_1 & I2C_SR1_RXNE) {                                 // Byte received - EV7
        state->read_p[index++] = (uint8_t)I2Cx->DR;
        if (state->bytes == (index + 3))
            I2C_ITConfig(I2Cx, I2C_IT_BUF, DISABLE);                    // disable TXE to allow the buffer to flush so we can get an EV7_2
        if (state->bytes == index)                                             // We have completed a final EV7
            index++;                                                    // to show job is complete
    }
    else if (SReg_1 & I2C_SR1_TXE) {                                  // Byte transmitted EV8 / EV8_1
        if (index != -1) {                                              // we dont have a subaddress to send
            I2Cx->DR = state->write_p[index++];
            if (state->bytes == index)                                         // we have sent all the data
                I2C_ITConfig(I2Cx, I2C_IT_BUF, DISABLE);                // disable TXE to allow the buffer to flush
        }
        else {
            index++;
            I2Cx->DR = state->reg;                                             // send the subaddress
            if (state->reading || !(state->bytes))                                      // if receiving or sending 0 bytes, flush now
                I2C_ITConfig(I2Cx, I2C_IT_BUF, DISABLE);                // disable TXE to allow the buffer to flush
        }
    }
    if (index == state->bytes + 1) {                                           // we have completed the current job
        subaddress_sent = 0;                                            // reset this here
        if (final_stop)                                                 // If there is a final stop and no more jobs, bus is inactive, disable interrupts to prevent BTF
            I2C_ITConfig(I2Cx, I2C_IT_EVT | I2C_IT_ERR, DISABLE);       // Disable EVT and ERR interrupts while bus inactive
        state->busy = 0;
    }
}

void i2cInit(I2CDevice device)
{
    if (device == I2CINVALID)
        return;

    i2cDevice_t *pDev = &i2cDevice[device];
    const i2cHardware_t *hw = pDev->hardware;
    const IO_t scl = pDev->scl;
    const IO_t sda = pDev->sda;

    if (!hw || IOGetOwner(scl) || IOGetOwner(sda)) {
        return;
    }

    I2C_TypeDef *I2Cx = hw->reg;

    memset(&pDev->state, 0, sizeof(pDev->state));

    NVIC_InitTypeDef nvic;
    I2C_InitTypeDef i2cInit;

    IOInit(scl, OWNER_I2C_SCL, RESOURCE_INDEX(device));
    IOInit(sda, OWNER_I2C_SDA, RESOURCE_INDEX(device));

    // Enable RCC
    RCC_ClockCmd(hw->rcc, ENABLE);

    I2C_ITConfig(I2Cx, I2C_IT_EVT | I2C_IT_ERR, DISABLE);

    i2cUnstick(scl, sda);

    // Init pins
#ifdef STM32F4
    IOConfigGPIOAF(scl, pDev->pullUp ? IOCFG_I2C_PU : IOCFG_I2C, pDev->sclAF);
    IOConfigGPIOAF(sda, pDev->pullUp ? IOCFG_I2C_PU : IOCFG_I2C, pDev->sdaAF);
#else
    IOConfigGPIO(scl, IOCFG_I2C);
    IOConfigGPIO(sda, IOCFG_I2C);
#endif

    I2C_DeInit(I2Cx);
    I2C_StructInit(&i2cInit);

    I2C_ITConfig(I2Cx, I2C_IT_EVT | I2C_IT_ERR, DISABLE);               // Disable EVT and ERR interrupts - they are enabled by the first request
    i2cInit.I2C_Mode = I2C_Mode_I2C;
    i2cInit.I2C_DutyCycle = I2C_DutyCycle_2;
    i2cInit.I2C_OwnAddress1 = 0;
    i2cInit.I2C_Ack = I2C_Ack_Enable;
    i2cInit.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    i2cInit.I2C_ClockSpeed = pDev->clockSpeed * 1000;

    I2C_Cmd(I2Cx, ENABLE);
    I2C_Init(I2Cx, &i2cInit);

    I2C_StretchClockCmd(I2Cx, ENABLE);

    // I2C ER Interrupt
    nvic.NVIC_IRQChannel = hw->er_irq;
    nvic.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(NVIC_PRIO_I2C_ER);
    nvic.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(NVIC_PRIO_I2C_ER);
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    // I2C EV Interrupt
    nvic.NVIC_IRQChannel = hw->ev_irq;
    nvic.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(NVIC_PRIO_I2C_EV);
    nvic.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(NVIC_PRIO_I2C_EV);
    NVIC_Init(&nvic);
}

uint16_t i2cGetErrorCounter(void)
{
    return i2cErrorCount;
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
