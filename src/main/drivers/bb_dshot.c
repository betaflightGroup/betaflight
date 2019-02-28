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
#include <math.h>
#include <string.h>

#include "platform.h"

#ifdef USE_BB_DSHOT

#include "build/debug.h"

#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/dma.h"
#include "drivers/nvic.h"
#include "drivers/pwm_output.h"
#include "drivers/pwm_output_counts.h"
#include "drivers/timer.h"

uint32_t portData[3 * 16];

static void dshotPortOutputDataInit(uint32_t *buffer, uint16_t portMask)
{
    uint32_t resetMask = (portMask << 16);
    int bitpos;

    for (bitpos = 0; bitpos < 16; bitpos++) {
        buffer[bitpos * 3 + 0] |= portMask ; // Always set all ports
        buffer[bitpos * 3 + 1] = 0;          // Reset bits are port dependent
        buffer[bitpos * 3 + 2] |= resetMask; // Always reset all ports
    }
}

static void dshotPortOutputDataSet(uint32_t *buffer, int portNumber, uint16_t value)
{
    uint32_t resetBit = (1 << (portNumber + 16));

    for (int pos = 0; pos < 16; pos++) {
        if (!(value & 0x8000)) {
            buffer[pos * 3 + 1] |= resetBit;
        }
        value <<= 1;
    }
}

static void dshotPortOutputDataClear(uint32_t *buffer)
{
    for (int bitpos = 0; bitpos < 16; bitpos++) {
        buffer[bitpos * 3 + 1] = 0;
    }
}

#if 0
// Not tested
static void proshotPortDataInit(uint32_t *buffer, uint16_t portMask)
{
    for (int nibble = 0; nibble < 4; nibble++) {
        buffer[nibble * 32 + 0] |= portMask; // Rising edge
    }
}

static void proshotPortDataSet(uint32_t *buffer, int portNumber, uint16_t value)
{
    uint32_t resetBit = (1 << (portNumber + 16));

    for (int nibble = 0; nibble < 4; nibble++) {
        uint32_t nibData = (value & 0xf000) >> 12;
        value <<= 4;
        buffer[nibble * 32 + 8 + nibData] = resetBit;
    }
}

static void proshotPortDataClear(uint32_t *buffer)
{
    for (int nibble = 0; nibble < 4; nibble++) {
        for (int i = 1; i < 32; i++) {
            buffer[nibble * 32 + i] = 0;
        }
    }
}
#endif

// XXX MOTOR_xSHOTyyyy_HZ is not usable as generic frequency for timers.
// XXX Trying to fiddle with constants here.

// Symbol rate [symbol/sec]
#define MOTOR_DSHOT1200_SYMBOL_RATE    (1200 * 1000)
#define MOTOR_DSHOT600_SYMBOL_RATE     (600 * 1000)
#define MOTOR_DSHOT300_SYMBOL_RATE     (300 * 1000)
#define MOTOR_DSHOT150_SYMBOL_RATE     (150 * 1000)
#define MOTOR_PROSHOT1000_SYMBOL_RATE  (250 * 1000)

#define MOTOR_DSHOT1200_SYMBOL_TIME    (1 / MOTOR_DSHOT1200_SYMBOL_RATE)   // 0.833us
#define MOTOR_DSHOT600_SYMBOL_TIME     (1 / MOTOR_DSHOT600_SYMBOL_RATE)    // 1.666us
#define MOTOR_DSHOT300_SYMBOL_TIME     (1 / MOTOR_DSHOT300_SYMBOL_RATE)    // 3.333us
#define MOTOR_DSHOT150_SYMBOL_TIME     (1 / MOTOR_DSHOT150_SYMBOL_RATE)    // 6.666us
#define MOTOR_PROSHOT1000_SYMBOL_TIME  (1 / MOTOR_PROSHOT1000_SYMBOL_RATE) // 4.000us

#define MOTOR_DSHOT_BIT_PER_SYMBOL         1
#define MOTOR_PROSHOT1000_BIT_PER_SYMBOL   4

#define MOTOR_DSHOT_STATE_PER_SYMBOL       3  // Initial high, 0/1, low
#define MOTOR_PROSHOT1000_STATE_PER_SYMBOL 32 // 8 preamble + 16 + 8 postamble

#define MOTOR_DSHOT_BUFFER_SIZE            ((16 / MOTOR_DSHOT_BIT_PER_SYMBOL) * MOTOR_DSHOT_STATE_PER_SYMBOL)
#define MOTOR_PROSHOT1000_BUFFER_SIZE      ((16 / MOTOR_PROSHOT1000_BIT_PER_SYMBOL) * MOTOR_PROSHOT1000_STATE_PER_SYMBOL)

// Per timer
typedef struct motorPacer_s {
    TIM_TypeDef *tim;
    uint16_t dmaSources;
} motorPacer_t;

// Per GPIO port and timer channel
typedef struct motorPort_s {
    //int portIndex; // XXX Not used if managed by gpioToMotorPort[] array
    GPIO_TypeDef *gpio;
    const timerHardware_t *timhw;
    uint16_t dmaSource;

    // Output
    TIM_TimeBaseInitTypeDef outputTimebase;
    uint32_t outputARR;
    DMA_InitTypeDef outputDmaInit;
    uint32_t *portOutputBuffer;
    uint32_t portOutputCount;
} motorPort_t;

// Per motor output
typedef struct directDshotMotor_s {
    int pinIndex;            // pinIndex of this motor output within a group that motorPort points to
    IO_t io;                 // IO_t for this output
    motorPort_t *motorPort;
} directDshotMotor_t;

#define MAX_MOTOR_PACERS  2
motorPacer_t motorPacers[MAX_MOTOR_PACERS];  // TIM1 or TIM8
int usedMotorPacers = 0;

motorPort_t motorPorts[MAX_SUPPORTED_MOTOR_PORTS];
int usedMotorPorts = 0;

directDshotMotor_t directDshotMotors[MAX_SUPPORTED_MOTORS];

motorPort_t *gpioToMotorPort[10]; // GPIO group to motorPort mapping

// DShot requires 3 [word/bit] * 16 [bit] = 48 [word]
// ProShot requires 32 [word/nibble] * 4 [nibble] = 128 [word]

uint32_t portDmaBuffer[MOTOR_PROSHOT1000_BUFFER_SIZE * MAX_SUPPORTED_MOTOR_PORTS];

static void directDshotIOInit(IO_t io, int index)
{
    IOInit(io, OWNER_MOTOR, RESOURCE_INDEX(index));
}

// XXX Too expensive to do input/output switching per motor bases?
// XXX Consider an extension of IOConfigGPIO (IOConfigGPIOGroup?) that takes multiple lines (GPIO_Init handles it).

static void directDshotSwitchToOutput(uint8_t motorCount)
{
    for (int i = 0; i < motorCount; i++) {
        IO_t io = directDshotMotors[i].io;
        if (io) {
            IOLo(io);
            IOConfigGPIO(io, IOCFG_OUT_PP);
        }
    }

    for (int i = 0; i < usedMotorPorts; i++) {
        motorPort_t *motorPort = &motorPorts[i];
        const timerHardware_t *timhw = motorPort->timhw;
        DMA_Stream_TypeDef *stream = timhw->dmaRef;

        TIM_Cmd(timhw->tim, DISABLE);

        timhw->tim->ARR = motorPort->outputARR;
	timhw->tim->EGR = TIM_PSCReloadMode_Immediate;

        DMA_DeInit(stream);
        DMA_Init(stream, &motorPort->outputDmaInit);

        // Needs this, as it is DeInit'ed above...
        DMA_ITConfig(stream, DMA_IT_TC, ENABLE);
    }
}

#if 0
static void directDshotSwitchToInput(IO_t io)
{
}
#endif

static void directDshotDMAIrqHandler(dmaChannelDescriptor_t *descriptor)
{
    motorPort_t *motorPort = (motorPort_t *)descriptor->userParam;

    DMA_Cmd(motorPort->timhw->dmaRef, DISABLE);

    TIM_DMACmd(motorPort->timhw->tim, motorPort->dmaSource, DISABLE);

    DMA_CLEAR_FLAG(descriptor, DMA_IT_TCIF);

#if 0
    if (input) {
        // Decode input
        // Switch to output
    } else {
        // Switch to input
    }
#endif
}

// motorPacer management

motorPacer_t *directDshotFindMotorPacer(TIM_TypeDef *tim)
{
    for (int i = 0; i < MAX_MOTOR_PACERS; i++) {

        motorPacer_t *motorPacer = &motorPacers[i];

        if (motorPacer->tim == NULL) {
            motorPacer->tim = tim;
            ++usedMotorPacers;
            return motorPacer;
        }

        if (motorPacer->tim == tim) {
            return motorPacer;
        }
    }

    return NULL;
}

// motorPort management

motorPort_t *directDshotFindMotorPort(int portIndex)
{
    return gpioToMotorPort[portIndex];
}

motorPort_t *directDshotAllocMotorPort(int portIndex)
{
    if (usedMotorPorts == MAX_SUPPORTED_MOTOR_PORTS) {
        return NULL;
    }

    motorPort_t *motorPort = &motorPorts[usedMotorPorts];

    gpioToMotorPort[portIndex] = motorPort;
    ++usedMotorPorts;

    return motorPort;
}

// Return frequency of smallest change [state/sec]

uint32_t getDshotBaseFrequency(motorPwmProtocolTypes_e pwmProtocolType)
{
    switch (pwmProtocolType) {
    case(PWM_TYPE_PROSHOT1000):
        return MOTOR_PROSHOT1000_SYMBOL_RATE * MOTOR_PROSHOT1000_STATE_PER_SYMBOL;
    case(PWM_TYPE_DSHOT1200):
        return MOTOR_DSHOT1200_SYMBOL_RATE * MOTOR_DSHOT_STATE_PER_SYMBOL;
    case(PWM_TYPE_DSHOT600):
        return MOTOR_DSHOT600_SYMBOL_RATE * MOTOR_DSHOT_STATE_PER_SYMBOL;
    case(PWM_TYPE_DSHOT300):
        return MOTOR_DSHOT300_SYMBOL_RATE * MOTOR_DSHOT_STATE_PER_SYMBOL;
    default:
    case(PWM_TYPE_DSHOT150):
        return MOTOR_DSHOT150_SYMBOL_RATE * MOTOR_DSHOT_STATE_PER_SYMBOL;
    }
}

void directDshotTimebaseSetup(const timerHardware_t *timhw, TIM_TimeBaseInitTypeDef *init, motorPwmProtocolTypes_e pwmProtocolType, int multiplier)
{

    // TIM_TimeBaseInitStruct.TIM_Prescaler = (uint16_t)(lrintf((float) timerClock(timhw->tim) / (3 * getDshotHz(pwmProtocolType)) + 0.01f) - 1);
    // TIM_TimeBaseInitStruct.TIM_Period = 20 - 1;

    init->TIM_Prescaler = 0; // Feed raw timerClock
    uint32_t basefreq = getDshotBaseFrequency(pwmProtocolType) * multiplier;
    uint32_t timerclock = timerClock(timhw->tim);
    uint32_t period = timerclock / basefreq;
    // TIM_TimeBaseInitStruct.TIM_Period = timerClock(timhw->tim) / getDshotBaseFrequency(pwmProtocolType);
    init->TIM_Period = period;

    init->TIM_ClockDivision = TIM_CKD_DIV1;
    init->TIM_CounterMode = TIM_CounterMode_Up;
}

void directDshotTimerChannelInit(motorPort_t *motorPort)
{
    const timerHardware_t *timhw = motorPort->timhw;
    TIM_OCInitTypeDef TIM_OCStruct;

    TIM_OCStructInit(&TIM_OCStruct);
    TIM_OCStruct.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCStruct.TIM_OCPolarity = TIM_OCPolarity_Low;

    TIM_OCStruct.TIM_Pulse = 499;

    timerOCInit(timhw->tim, timhw->channel, &TIM_OCStruct);
    timerOCPreloadConfig(timhw->tim, timhw->channel, TIM_OCPreload_Enable);

#if 0
        // XXX Preload config may have to be switched on and off before and after OCInit.
        timerOCPreloadConfig(timer, timerHardware->channel, TIM_OCPreload_Disable);
        timerOCInit(timer, timerHardware->channel, pOcInit);
        timerOCPreloadConfig(timer, timerHardware->channel, TIM_OCPreload_Enable);
#endif
}

void directDshotAllocDMA(motorPort_t *motorPort)
{
    const timerHardware_t *timhw = motorPort->timhw;

    DMA_Stream_TypeDef *stream = timhw->dmaRef;
    dmaIdentifier_e dmaIdentifier = dmaGetIdentifier(stream);
    dmaInit(dmaIdentifier, OWNER_DSHOT, RESOURCE_INDEX(motorPort - motorPorts));
    motorPort->dmaSource = timerDmaSource(timhw->channel);

    motorPacer_t *motorPacer = directDshotFindMotorPacer(timhw->tim);
    motorPacer->dmaSources |= motorPort->dmaSource;

    dmaSetHandler(dmaIdentifier, directDshotDMAIrqHandler, NVIC_BUILD_PRIORITY(2, 1), (uint32_t)motorPort);
    DMA_ITConfig(stream, DMA_IT_TC, ENABLE);
}

#define DIRECT_DSHOT_DIRECTION_OUTPUT 0
#define DIRECT_DSHOT_DIRECTION_INPUT  1

void directDshotDMASetup(motorPort_t *motorPort, uint8_t direction)
{
    const timerHardware_t *timhw = motorPort->timhw;
    DMA_InitTypeDef *dmainit;

    if (direction == DIRECT_DSHOT_DIRECTION_OUTPUT) {
        dmainit = &motorPort->outputDmaInit;
        DMA_StructInit(dmainit);
        dmainit->DMA_DIR = DMA_DIR_MemoryToPeripheral;
        dmainit->DMA_Mode = DMA_Mode_Normal;
        dmainit->DMA_Channel = timhw->dmaChannel;
        dmainit->DMA_Priority = DMA_Priority_Medium;
        dmainit->DMA_BufferSize = motorPort->portOutputCount;
        dmainit->DMA_PeripheralBaseAddr = (uint32_t)&motorPort->gpio->BSRRL;
        dmainit->DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        dmainit->DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
        dmainit->DMA_Memory0BaseAddr = (uint32_t)motorPort->portOutputBuffer;
        dmainit->DMA_MemoryInc = DMA_MemoryInc_Enable;
        dmainit->DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
        dmainit->DMA_FIFOMode = DMA_FIFOMode_Disable ;
        dmainit->DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull ;
        dmainit->DMA_MemoryBurst = DMA_MemoryBurst_Single ;
        dmainit->DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    }
}

// Find TIM1 and/or TIM8 in timerHardware array for direct DShot

void directDshotInit(motorPwmProtocolTypes_e pwmProtocolType)
{
    UNUSED(pwmProtocolType);

    const timerHardware_t *timhw;

    for (int motorPortIndex = 0; motorPortIndex < MAX_SUPPORTED_MOTOR_PORTS; motorPortIndex++) {
        timhw = timerGetByUsage(TIM_USE_BB_DSHOT, motorPortIndex);

        if (!timhw) {
            break;
        }

        motorPorts[motorPortIndex].timhw = timhw;
    }

    // extern void directTelemetryInit(motorPwmProtocolTypes_e pwmProtocolType);
    // directTelemetryInit(pwmProtocolType);
}

//
// directDshot only use pin info associated with timerHardware;
// it does not use the timer channel associated with the pin.
//

extern motorDmaOutput_t dmaMotors[MAX_SUPPORTED_MOTORS];

void directDshotMotorConfig(const timerHardware_t *timhw, uint8_t motorIndex, motorPwmProtocolTypes_e pwmProtocolType, uint8_t output)
{
    UNUSED(output); // Not supported

    IO_t io = IOGetByTag(timhw->tag);

    int pinIndex = IO_GPIOPinIdx(io);
    int portIndex = IO_GPIOPortIdx(io);

    motorPort_t *motorPort = directDshotFindMotorPort(portIndex);

    if (!motorPort) {
        motorPort = directDshotAllocMotorPort(portIndex);
        if (!motorPort) {
            return;
        }

        motorPort->gpio = IO_GPIO(io);

        uint16_t bufferSize = (pwmProtocolType == PWM_TYPE_PROSHOT1000) ? MOTOR_PROSHOT1000_BUFFER_SIZE : MOTOR_DSHOT_BUFFER_SIZE;

        motorPort->portOutputCount = bufferSize;
        motorPort->portOutputBuffer = &portDmaBuffer[(motorPort - motorPorts) * bufferSize];

        directDshotTimebaseSetup(motorPort->timhw, &motorPort->outputTimebase, pwmProtocolType, 1);
        TIM_TimeBaseInit(timhw->tim, &motorPort->outputTimebase);
        motorPort->outputARR = timhw->tim->ARR;

        directDshotTimerChannelInit(motorPort);
        directDshotAllocDMA(motorPort);
        directDshotDMASetup(motorPort, DIRECT_DSHOT_DIRECTION_OUTPUT);
    }

    directDshotMotors[motorIndex].pinIndex = pinIndex;
    directDshotMotors[motorIndex].io = io;
    directDshotMotors[motorIndex].motorPort = motorPort;

    directDshotIOInit(io, motorIndex);

    dshotPortOutputDataInit(motorPort->portOutputBuffer, (1 << pinIndex));

    motorDmaOutput_t *const motor = &dmaMotors[motorIndex];
    motor->configured = true;
}

void directDshotUpdateStart(uint8_t motorCount)
{
    // Switch direction to output

    directDshotSwitchToOutput(motorCount);

    for (int i = 0; i < motorCount; i++) {
        dshotPortOutputDataClear(motorPorts[i].portOutputBuffer);
    }

    for (int i = 0; i < usedMotorPacers; i++) {
        motorPacer_t *motorPacer = &motorPacers[i];
        TIM_Cmd(motorPacer->tim, DISABLE);
    }
}

void directDshotWrite(int motorIndex, uint16_t value)
{
    motorDmaOutput_t *const motor = &dmaMotors[motorIndex];

    if (!motor->configured) {
        return;
    }

    // If there is a command ready to go overwrite the value and send that instead

    if (pwmDshotCommandIsProcessing()) {
        value = pwmGetDshotCommand(motorIndex);
        if (value) {
            motor->requestTelemetry = true;
        }
    }

    motor->value = value;

    uint16_t packet = prepareDshotPacket(motor);

    directDshotMotor_t *ddmotor = &directDshotMotors[motorIndex];
    motorPort_t *motorPort = ddmotor->motorPort;

    dshotPortOutputDataSet(motorPort->portOutputBuffer, ddmotor->pinIndex, packet);
}

void directDshotUpdateComplete(uint8_t motorCount)
{
    UNUSED(motorCount);

    // If there is a dshot command loaded up, time it correctly with motor update

    if (pwmDshotCommandIsQueued()) {
        if (!pwmDshotCommandOutputIsEnabled(motorCount)) {
            return;
        }
    }

    for (int i = 0; i < usedMotorPorts; i++) {
        motorPort_t *motorPort = &motorPorts[i];
        DMA_Cmd(motorPort->timhw->dmaRef, ENABLE);
    }

    for (int i = 0; i < usedMotorPacers; i++) {
        motorPacer_t *motorPacer = &motorPacers[i];
        TIM_Cmd(motorPacer->tim, DISABLE);
        TIM_SetCounter(motorPacer->tim, 0);
        TIM_DMACmd(motorPacer->tim, motorPacer->dmaSources, ENABLE);
        TIM_Cmd(motorPacer->tim, ENABLE);
    }
}
#endif // USE_BB_DSHOT
