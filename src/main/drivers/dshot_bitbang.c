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

#ifdef USE_DSHOT_BITBANG

#include "build/debug.h"

#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/dma.h"
#include "drivers/dma_reqmap.h"
#include "drivers/dshot.h"
#include "drivers/dshot_bitbang.h"
#include "drivers/dshot_bitbang_impl.h"
#include "drivers/dshot_command.h"
#include "drivers/motor.h"
#include "drivers/nvic.h"
#include "drivers/pwm_output.h" // XXX for pwmOutputPort_t motors[]; should go away with refactoring
#include "drivers/dshot_dpwm.h" // XXX for motorDmaOutput_t *getMotorDmaOutput(uint8_t index); should go away with refactoring
#include "drivers/dshot_bitbang_decode.h"
#include "drivers/time.h"
#include "drivers/timer.h"

#include "pg/motor.h"

#if defined(USE_DEBUG_PIN)
#include "build/debug_pin.h"
#else
#define dbgPinInit()
#define dbgPinHi(x)
#define dbgPinLo(x)
#endif

FAST_DATA_ZERO_INIT bbPacer_t bbPacers[MAX_MOTOR_PACERS];  // TIM1 or TIM8
FAST_DATA_ZERO_INIT int usedMotorPacers = 0;

FAST_DATA_ZERO_INIT bbPort_t bbPorts[MAX_SUPPORTED_MOTOR_PORTS];
FAST_DATA_ZERO_INIT int usedMotorPorts;

FAST_DATA_ZERO_INIT bbMotor_t bbMotors[MAX_SUPPORTED_MOTORS];

static FAST_DATA_ZERO_INIT int motorCount;
dshotBitbangStatus_e bbStatus;

// For MCUs that use MPU to control DMA coherency, there might be a performance hit
// on manipulating input buffer content especially if it is read multiple times,
// as the buffer region is attributed as not cachable.
// If this is not desirable, we should use manual cache invalidation.
#ifdef USE_DSHOT_CACHE_MGMT
#define BB_OUTPUT_BUFFER_ATTRIBUTE __attribute__((aligned(32)))
#define BB_INPUT_BUFFER_ATTRIBUTE  __attribute__((aligned(32)))
#else
#if defined(STM32F4)
#define BB_OUTPUT_BUFFER_ATTRIBUTE
#define BB_INPUT_BUFFER_ATTRIBUTE
#elif defined(STM32F7)
#define BB_OUTPUT_BUFFER_ATTRIBUTE FAST_DATA_ZERO_INIT
#define BB_INPUT_BUFFER_ATTRIBUTE  FAST_DATA_ZERO_INIT
#elif defined(STM32H7)
#define BB_OUTPUT_BUFFER_ATTRIBUTE DMA_RAM
#define BB_INPUT_BUFFER_ATTRIBUTE  DMA_RAM
#elif defined(STM32G4)
#define BB_OUTPUT_BUFFER_ATTRIBUTE DMA_RAM_W
#define BB_INPUT_BUFFER_ATTRIBUTE  DMA_RAM_R
#endif
#endif // USE_DSHOT_CACHE_MGMT

BB_OUTPUT_BUFFER_ATTRIBUTE uint32_t bbOutputBuffer[MOTOR_DSHOT_BUF_CACHE_ALIGN_LENGTH * MAX_SUPPORTED_MOTOR_PORTS];
BB_INPUT_BUFFER_ATTRIBUTE uint16_t bbInputBuffer[DSHOT_BB_PORT_IP_BUF_CACHE_ALIGN_LENGTH * MAX_SUPPORTED_MOTOR_PORTS];

uint8_t bbPuPdMode;
FAST_DATA_ZERO_INIT timeUs_t dshotFrameUs;


const timerHardware_t bbTimerHardware[] = {
#if defined(STM32F4) || defined(STM32F7)
#if !defined(STM32F411xE)
    DEF_TIM(TIM8,  CH1, NONE,  TIM_USE_NONE, 0, 1),
    DEF_TIM(TIM8,  CH2, NONE,  TIM_USE_NONE, 0, 1),
    DEF_TIM(TIM8,  CH3, NONE,  TIM_USE_NONE, 0, 1),
    DEF_TIM(TIM8,  CH4, NONE,  TIM_USE_NONE, 0, 0),
#endif
    DEF_TIM(TIM1,  CH1, NONE,  TIM_USE_NONE, 0, 1),
    DEF_TIM(TIM1,  CH1, NONE,  TIM_USE_NONE, 0, 2),
    DEF_TIM(TIM1,  CH2, NONE,  TIM_USE_NONE, 0, 1),
    DEF_TIM(TIM1,  CH3, NONE,  TIM_USE_NONE, 0, 1),
    DEF_TIM(TIM1,  CH4, NONE,  TIM_USE_NONE, 0, 0),

#elif defined(STM32G4) || defined(STM32H7)
    // XXX TODO: STM32G4 and STM32H7 can use any timer for pacing

    // DMA request numbers are duplicated for TIM1 and TIM8:
    //   - Any pacer can serve a GPIO port.
    //   - For quads (or less), 4 pacers can cover the worst case scenario of
    //     4 motors scattered across 4 different GPIO ports.
    //   - For hexas (and larger), more channels may become necessary,
    //     in which case the DMA request numbers should be modified.
    DEF_TIM(TIM8,  CH1, NONE,  TIM_USE_NONE, 0, 0, 0),
    DEF_TIM(TIM8,  CH2, NONE,  TIM_USE_NONE, 0, 1, 0),
    DEF_TIM(TIM8,  CH3, NONE,  TIM_USE_NONE, 0, 2, 0),
    DEF_TIM(TIM8,  CH4, NONE,  TIM_USE_NONE, 0, 3, 0),
    DEF_TIM(TIM1,  CH1, NONE,  TIM_USE_NONE, 0, 0, 0),
    DEF_TIM(TIM1,  CH2, NONE,  TIM_USE_NONE, 0, 1, 0),
    DEF_TIM(TIM1,  CH3, NONE,  TIM_USE_NONE, 0, 2, 0),
    DEF_TIM(TIM1,  CH4, NONE,  TIM_USE_NONE, 0, 3, 0),

#else
#error MCU dependent code required
#endif
};

static FAST_DATA_ZERO_INIT motorDevice_t bbDevice;
static FAST_DATA_ZERO_INIT timeUs_t lastSendUs;

static motorPwmProtocolTypes_e motorPwmProtocol;

// DMA GPIO output buffer formatting

static void bbOutputDataInit(uint32_t *buffer, uint16_t portMask, bool inverted)
{
    uint32_t resetMask;
    uint32_t setMask;

    if (inverted) {
        resetMask = portMask;
        setMask = (portMask << 16);
    } else {
        resetMask = (portMask << 16);
        setMask = portMask;
    }

    int bitpos;

    for (bitpos = 0; bitpos < 16; bitpos++) {
        buffer[bitpos * 3 + 0] |= setMask ; // Always set all ports
        buffer[bitpos * 3 + 1] = 0;          // Reset bits are port dependent
        buffer[bitpos * 3 + 2] |= resetMask; // Always reset all ports
    }
}

static void bbOutputDataSet(uint32_t *buffer, int pinNumber, uint16_t value, bool inverted)
{
    uint32_t middleBit;

    if (inverted) {
        middleBit = (1 << (pinNumber + 0));
    } else {
        middleBit = (1 << (pinNumber + 16));
    }

    for (int pos = 0; pos < 16; pos++) {
        if (!(value & 0x8000)) {
            buffer[pos * 3 + 1] |= middleBit;
        }
        value <<= 1;
    }
}

static void bbOutputDataClear(uint32_t *buffer)
{
    // Middle position to no change
    for (int bitpos = 0; bitpos < 16; bitpos++) {
        buffer[bitpos * 3 + 1] = 0;
    }
}

// bbPacer management

static bbPacer_t *bbFindMotorPacer(TIM_TypeDef *tim)
{
    for (int i = 0; i < MAX_MOTOR_PACERS; i++) {

        bbPacer_t *bbPacer = &bbPacers[i];

        if (bbPacer->tim == NULL) {
            bbPacer->tim = tim;
            ++usedMotorPacers;
            return bbPacer;
        }

        if (bbPacer->tim == tim) {
            return bbPacer;
        }
    }

    return NULL;
}

// bbPort management

static bbPort_t *bbFindMotorPort(int portIndex)
{
    for (int i = 0; i < usedMotorPorts; i++) {
        if (bbPorts[i].portIndex == portIndex) {
            return &bbPorts[i];
        }
    }
    return NULL;
}

static bbPort_t *bbAllocMotorPort(int portIndex)
{
    if (usedMotorPorts >= MAX_SUPPORTED_MOTOR_PORTS) {
        bbStatus = DSHOT_BITBANG_STATUS_TOO_MANY_PORTS;
        return NULL;
    }

    bbPort_t *bbPort = &bbPorts[usedMotorPorts];

    if (!bbPort->timhw) {
        // No more pacer channel available
        bbStatus = DSHOT_BITBANG_STATUS_NO_PACER;
        return NULL;
    }

    bbPort->portIndex = portIndex;
    bbPort->owner.owner = OWNER_DSHOT_BITBANG;
    bbPort->owner.resourceIndex = RESOURCE_INDEX(portIndex);

    ++usedMotorPorts;

    return bbPort;
}

const resourceOwner_t *dshotBitbangTimerGetOwner(int8_t timerNumber, uint16_t timerChannel)
{
    for (int index = 0; index < usedMotorPorts; index++) {
        const timerHardware_t *timer = bbPorts[index].timhw;
        if (timerGetTIMNumber(timer->tim) == timerNumber && timer->channel == timerChannel) {
            return &bbPorts[index].owner;
        }
    }

    return &freeOwner;
}

// Return frequency of smallest change [state/sec]

static uint32_t getDshotBaseFrequency(motorPwmProtocolTypes_e pwmProtocolType)
{
    switch (pwmProtocolType) {
    case(PWM_TYPE_DSHOT600):
        return MOTOR_DSHOT600_SYMBOL_RATE * MOTOR_DSHOT_STATE_PER_SYMBOL;
    case(PWM_TYPE_DSHOT300):
        return MOTOR_DSHOT300_SYMBOL_RATE * MOTOR_DSHOT_STATE_PER_SYMBOL;
    default:
    case(PWM_TYPE_DSHOT150):
        return MOTOR_DSHOT150_SYMBOL_RATE * MOTOR_DSHOT_STATE_PER_SYMBOL;
    }
}

static void bbAllocDma(bbPort_t *bbPort)
{
    const timerHardware_t *timhw = bbPort->timhw;

#ifdef USE_DMA_SPEC
    dmaoptValue_t dmaopt = dmaGetOptionByTimer(timhw);
    const dmaChannelSpec_t *dmaChannelSpec = dmaGetChannelSpecByTimerValue(timhw->tim, timhw->channel, dmaopt);
    bbPort->dmaResource = dmaChannelSpec->ref;
    bbPort->dmaChannel = dmaChannelSpec->channel;
#else
    bbPort->dmaResource = timhw->dmaRef;
    bbPort->dmaChannel = timhw->dmaChannel;
#endif

    dmaIdentifier_e dmaIdentifier = dmaGetIdentifier(bbPort->dmaResource);
    dmaInit(dmaIdentifier, OWNER_DSHOT_BITBANG, bbPort->owner.resourceIndex);
    bbPort->dmaSource = timerDmaSource(timhw->channel);

    bbPacer_t *bbPacer = bbFindMotorPacer(timhw->tim);
    bbPacer->dmaSources |= bbPort->dmaSource;

    dmaSetHandler(dmaIdentifier, bbDMAIrqHandler, NVIC_BUILD_PRIORITY(2, 1), (uint32_t)bbPort);

    bbDMA_ITConfig(bbPort);
}

FAST_CODE void bbDMAIrqHandler(dmaChannelDescriptor_t *descriptor)
{
    dbgPinHi(0);

    bbPort_t *bbPort = (bbPort_t *)descriptor->userParam;

    bbDMA_Cmd(bbPort, DISABLE);

    bbTIM_DMACmd(bbPort->timhw->tim, bbPort->dmaSource, DISABLE);

    if (DMA_GET_FLAG_STATUS(descriptor, DMA_IT_TEIF)) {
        while (1) {};
    }

    DMA_CLEAR_FLAG(descriptor, DMA_IT_TCIF);

#ifdef USE_DSHOT_TELEMETRY
    if (useDshotTelemetry) {
        if (bbPort->direction == DSHOT_BITBANG_DIRECTION_INPUT) {
#ifdef DEBUG_COUNT_INTERRUPT
            bbPort->inputIrq++;
#endif
        } else {
#ifdef DEBUG_COUNT_INTERRUPT
            bbPort->outputIrq++;
#endif

            // Switch to input

            bbSwitchToInput(bbPort);

            bbTIM_DMACmd(bbPort->timhw->tim, bbPort->dmaSource, ENABLE);
        }
    }
#endif
    dbgPinLo(0);
}

// Setup bbPorts array elements so that they each have a TIM1 or TIM8 channel
// in timerHardware array for BB-DShot.

static void bbFindPacerTimer(void)
{
    for (int bbPortIndex = 0; bbPortIndex < MAX_SUPPORTED_MOTOR_PORTS; bbPortIndex++) {
        for (unsigned timerIndex = 0; timerIndex < ARRAYLEN(bbTimerHardware); timerIndex++) {
            const timerHardware_t *timer = &bbTimerHardware[timerIndex];
            int timNumber = timerGetTIMNumber(timer->tim);
            if ((motorConfig()->dev.useDshotBitbangedTimer == DSHOT_BITBANGED_TIMER_TIM1 && timNumber != 1)
                || (motorConfig()->dev.useDshotBitbangedTimer == DSHOT_BITBANGED_TIMER_TIM8 && timNumber != 8)) {
                continue;
            }
            bool timerConflict = false;
            for (int channel = 0; channel < CC_CHANNELS_PER_TIMER; channel++) {
                const resourceOwner_e timerOwner = timerGetOwner(timNumber, CC_CHANNEL_FROM_INDEX(channel))->owner;
                if (timerOwner != OWNER_FREE && timerOwner != OWNER_DSHOT_BITBANG) {
                    timerConflict = true;
                    break;
                }
            }

            for (int index = 0; index < bbPortIndex; index++) {
                const timerHardware_t* t = bbPorts[index].timhw;
                if (timerGetTIMNumber(t->tim) == timNumber && timer->channel == t->channel) {
                    timerConflict = true;
                    break;
                }
            }

            if (timerConflict) {
                continue;
            }

#ifdef USE_DMA_SPEC
            dmaoptValue_t dmaopt = dmaGetOptionByTimer(timer);
            const dmaChannelSpec_t *dmaChannelSpec = dmaGetChannelSpecByTimerValue(timer->tim, timer->channel, dmaopt);
            dmaResource_t *dma = dmaChannelSpec->ref;
#else
            dmaResource_t *dma = timer->dmaRef;
#endif
            dmaIdentifier_e dmaIdentifier = dmaGetIdentifier(dma);
            if (dmaGetOwner(dmaIdentifier)->owner == OWNER_FREE) {
                bbPorts[bbPortIndex].timhw = timer;

                break;
            }
        }
    }
}

static void bbTimebaseSetup(bbPort_t *bbPort, motorPwmProtocolTypes_e dshotProtocolType)
{
    uint32_t timerclock = timerClock(bbPort->timhw->tim);

    uint32_t outputFreq = getDshotBaseFrequency(dshotProtocolType);
    dshotFrameUs = 1000000 * 17 * 3 / outputFreq;
    bbPort->outputARR = timerclock / outputFreq - 1;

    // XXX Explain this formula
    uint32_t inputFreq = outputFreq * 5 * 2 * DSHOT_BITBANG_TELEMETRY_OVER_SAMPLE / 24;
    bbPort->inputARR = timerclock / inputFreq - 1;
}

//
// bb only use pin info associated with timerHardware entry designated as TIM_USE_MOTOR;
// it does not use the timer channel associated with the pin.
//

static bool bbMotorConfig(IO_t io, uint8_t motorIndex, motorPwmProtocolTypes_e pwmProtocolType, uint8_t output)
{
    int pinIndex = IO_GPIOPinIdx(io);
    int portIndex = IO_GPIOPortIdx(io);

    bbPort_t *bbPort = bbFindMotorPort(portIndex);

    if (!bbPort) {

        // New port group

        bbPort = bbAllocMotorPort(portIndex);
        if (!bbPort) {
            bbDevice.vTable.write = motorWriteNull;
            bbDevice.vTable.updateStart = motorUpdateStartNull;
            bbDevice.vTable.updateComplete = motorUpdateCompleteNull;

            return false;
        }

        bbPort->gpio = IO_GPIO(io);

        bbPort->portOutputCount = MOTOR_DSHOT_BUF_LENGTH;
        bbPort->portOutputBuffer = &bbOutputBuffer[(bbPort - bbPorts) * MOTOR_DSHOT_BUF_CACHE_ALIGN_LENGTH];

        bbPort->portInputCount = DSHOT_BB_PORT_IP_BUF_LENGTH;
        bbPort->portInputBuffer = &bbInputBuffer[(bbPort - bbPorts) * DSHOT_BB_PORT_IP_BUF_CACHE_ALIGN_LENGTH];

        bbTimebaseSetup(bbPort, pwmProtocolType);
        bbTIM_TimeBaseInit(bbPort, bbPort->outputARR);
        bbTimerChannelInit(bbPort);

        bbAllocDma(bbPort);
        bbDMAPreconfigure(bbPort, DSHOT_BITBANG_DIRECTION_OUTPUT);
        bbDMAPreconfigure(bbPort, DSHOT_BITBANG_DIRECTION_INPUT);

        bbDMA_ITConfig(bbPort);
    }

    bbMotors[motorIndex].pinIndex = pinIndex;
    bbMotors[motorIndex].io = io;
    bbMotors[motorIndex].output = output;
    bbMotors[motorIndex].bbPort = bbPort;

    IOInit(io, OWNER_MOTOR, RESOURCE_INDEX(motorIndex));

    // Setup GPIO_MODER and GPIO_ODR register manipulation values

    bbGpioSetup(&bbMotors[motorIndex]);

#ifdef USE_DSHOT_TELEMETRY
    if (useDshotTelemetry) {
        bbOutputDataInit(bbPort->portOutputBuffer, (1 << pinIndex), DSHOT_BITBANG_INVERTED);
    } else
#endif
    {
        bbOutputDataInit(bbPort->portOutputBuffer, (1 << pinIndex), DSHOT_BITBANG_NONINVERTED);
    }

    bbSwitchToOutput(bbPort);

    bbMotors[motorIndex].configured = true;

    return true;
}

static bool bbUpdateStart(void)
{
#ifdef USE_DSHOT_TELEMETRY
    if (useDshotTelemetry) {
#ifdef USE_DSHOT_TELEMETRY_STATS
        const timeMs_t currentTimeMs = millis();
#endif
        timeUs_t currentUs = micros();
        // don't send while telemetry frames might still be incoming
        if (cmpTimeUs(currentUs, lastSendUs) < (timeDelta_t)(40 + 2 * dshotFrameUs)) {
            return false;
        }

        for (int motorIndex = 0; motorIndex < MAX_SUPPORTED_MOTORS && motorIndex < motorCount; motorIndex++) {
#ifdef USE_DSHOT_CACHE_MGMT
            // Only invalidate the buffer once. If all motors are on a common port they'll share a buffer.
            bool invalidated = false;
            for (int i = 0; i < motorIndex; i++) {
                if (bbMotors[motorIndex].bbPort->portInputBuffer == bbMotors[i].bbPort->portInputBuffer) {
                    invalidated = true;
                }
            }
            if (!invalidated) {
                SCB_InvalidateDCache_by_Addr((uint32_t *)bbMotors[motorIndex].bbPort->portInputBuffer,
                                             DSHOT_BB_PORT_IP_BUF_CACHE_ALIGN_BYTES);
            }
#endif

#ifdef STM32F4
            uint32_t value = decode_bb_bitband(
                bbMotors[motorIndex].bbPort->portInputBuffer,
                bbMotors[motorIndex].bbPort->portInputCount - bbDMA_Count(bbMotors[motorIndex].bbPort),
                bbMotors[motorIndex].pinIndex);
#else
            uint32_t value = decode_bb(
                bbMotors[motorIndex].bbPort->portInputBuffer,
                bbMotors[motorIndex].bbPort->portInputCount - bbDMA_Count(bbMotors[motorIndex].bbPort),
                bbMotors[motorIndex].pinIndex);
#endif
            if (value == BB_NOEDGE) {
                continue;
            }
            dshotTelemetryState.readCount++;

            if (value != BB_INVALID) {
                dshotTelemetryState.motorState[motorIndex].telemetryValue = value;
                dshotTelemetryState.motorState[motorIndex].telemetryActive = true;
                if (motorIndex < 4) {
                    DEBUG_SET(DEBUG_DSHOT_RPM_TELEMETRY, motorIndex, value);
                }
            } else {
                dshotTelemetryState.invalidPacketCount++;
            }
#ifdef USE_DSHOT_TELEMETRY_STATS
            updateDshotTelemetryQuality(&dshotTelemetryQuality[motorIndex], value != BB_INVALID, currentTimeMs);
#endif
        }
    }
#endif
    for (int i = 0; i < usedMotorPorts; i++) {
        bbDMA_Cmd(&bbPorts[i], DISABLE);
        bbOutputDataClear(bbPorts[i].portOutputBuffer);
    }

    return true;
}

static void bbWriteInt(uint8_t motorIndex, uint16_t value)
{
    bbMotor_t *const bbmotor = &bbMotors[motorIndex];

    if (!bbmotor->configured) {
        return;
    }

    // fetch requestTelemetry from motors. Needs to be refactored.
    motorDmaOutput_t * const motor = getMotorDmaOutput(motorIndex);
    bbmotor->protocolControl.requestTelemetry = motor->protocolControl.requestTelemetry;
    motor->protocolControl.requestTelemetry = false;

    // If there is a command ready to go overwrite the value and send that instead
    if (dshotCommandIsProcessing()) {
        value = dshotCommandGetCurrent(motorIndex);
        if (value) {
            bbmotor->protocolControl.requestTelemetry = true;
        }
    }

    bbmotor->protocolControl.value = value;

    uint16_t packet = prepareDshotPacket(&bbmotor->protocolControl);

    bbPort_t *bbPort = bbmotor->bbPort;

#ifdef USE_DSHOT_TELEMETRY
    if (useDshotTelemetry) {
        bbOutputDataSet(bbPort->portOutputBuffer, bbmotor->pinIndex, packet, DSHOT_BITBANG_INVERTED);
    } else
#endif
    {
        bbOutputDataSet(bbPort->portOutputBuffer, bbmotor->pinIndex, packet, DSHOT_BITBANG_NONINVERTED);
    }
}

static void bbWrite(uint8_t motorIndex, float value)
{
    bbWriteInt(motorIndex, value);
}

static void bbUpdateComplete(void)
{
    // If there is a dshot command loaded up, time it correctly with motor update

    if (!dshotCommandQueueEmpty()) {
        if (!dshotCommandOutputIsEnabled(bbDevice.count)) {
            return;
        }
    }

#ifdef USE_DSHOT_CACHE_MGMT
    for (int motorIndex = 0; motorIndex < MAX_SUPPORTED_MOTORS && motorIndex < motorCount; motorIndex++) {
        // Only clean the buffer once. If all motors are on a common port they'll share a buffer.
        bool clean = false;
        for (int i = 0; i < motorIndex; i++) {
            if (bbMotors[motorIndex].bbPort->portOutputBuffer == bbMotors[i].bbPort->portOutputBuffer) {
                clean = true;
            }
        }
        if (!clean) {
            SCB_CleanDCache_by_Addr(bbMotors[motorIndex].bbPort->portOutputBuffer, MOTOR_DSHOT_BUF_CACHE_ALIGN_BYTES);
        }
    }
#endif

#ifdef USE_DSHOT_TELEMETRY
    for (int i = 0; i < usedMotorPorts; i++) {
        bbPort_t *bbPort = &bbPorts[i];

        if (useDshotTelemetry) {
            if (bbPort->direction == DSHOT_BITBANG_DIRECTION_INPUT) {
                bbPort->inputActive = false;
                bbSwitchToOutput(bbPort);
            }


            bbDMA_Cmd(bbPort, ENABLE);
        }
    }
#endif

    lastSendUs = micros();
    for (int i = 0; i < usedMotorPacers; i++) {
        bbPacer_t *bbPacer = &bbPacers[i];
        bbTIM_DMACmd(bbPacer->tim, bbPacer->dmaSources, ENABLE);
    }
}

static bool bbEnableMotors(void)
{
    for (int i = 0; i < motorCount; i++) {
        if (bbMotors[i].configured) {
            IOConfigGPIO(bbMotors[i].io, bbMotors[i].iocfg);
        }
    }
    return true;
}

static void bbDisableMotors(void)
{
    return;
}

static void bbShutdown(void)
{
    return;
}

static bool bbIsMotorEnabled(uint8_t index)
{
    return bbMotors[index].enabled;
}

static void bbPostInit()
{
    bbFindPacerTimer();

    for (int motorIndex = 0; motorIndex < MAX_SUPPORTED_MOTORS && motorIndex < motorCount; motorIndex++) {

        if (!bbMotorConfig(bbMotors[motorIndex].io, motorIndex, motorPwmProtocol, bbMotors[motorIndex].output)) {
            return NULL;
        }


        bbMotors[motorIndex].enabled = true;

        // Fill in motors structure for 4way access (XXX Should be refactored)

        motors[motorIndex].enabled = true;
    }
}

static motorVTable_t bbVTable = {
    .postInit = bbPostInit,
    .enable = bbEnableMotors,
    .disable = bbDisableMotors,
    .isMotorEnabled = bbIsMotorEnabled,
    .updateStart = bbUpdateStart,
    .write = bbWrite,
    .writeInt = bbWriteInt,
    .updateComplete = bbUpdateComplete,
    .convertExternalToMotor = dshotConvertFromExternal,
    .convertMotorToExternal = dshotConvertToExternal,
    .shutdown = bbShutdown,
};

dshotBitbangStatus_e dshotBitbangGetStatus()
{
    return bbStatus;
}

motorDevice_t *dshotBitbangDevInit(const motorDevConfig_t *motorConfig, uint8_t count)
{
    dbgPinInit();
    dbgPinLo(0);
    dbgPinLo(1);

    motorPwmProtocol = motorConfig->motorPwmProtocol;
    bbDevice.vTable = bbVTable;
    motorCount = count;
    bbStatus = DSHOT_BITBANG_STATUS_OK;

#ifdef USE_DSHOT_TELEMETRY
    useDshotTelemetry = motorConfig->useDshotTelemetry;
#endif

    memset(bbOutputBuffer, 0, sizeof(bbOutputBuffer));

    for (int motorIndex = 0; motorIndex < MAX_SUPPORTED_MOTORS && motorIndex < motorCount; motorIndex++) {
        const unsigned reorderedMotorIndex = motorConfig->motorOutputReordering[motorIndex];
        const timerHardware_t *timerHardware = timerGetByTag(motorConfig->ioTags[reorderedMotorIndex]);
        const IO_t io = IOGetByTag(motorConfig->ioTags[reorderedMotorIndex]);

        uint8_t output = motorConfig->motorPwmInversion ?  timerHardware->output ^ TIMER_OUTPUT_INVERTED : timerHardware->output;
        bbPuPdMode = (output & TIMER_OUTPUT_INVERTED) ? BB_GPIO_PULLDOWN : BB_GPIO_PULLUP;

#ifdef USE_DSHOT_TELEMETRY
        if (useDshotTelemetry) {
            output ^= TIMER_OUTPUT_INVERTED;
        }
#endif

        if (!IOIsFreeOrPreinit(io)) {
            /* not enough motors initialised for the mixer or a break in the motors */
            bbDevice.vTable.write = motorWriteNull;
            bbDevice.vTable.updateStart = motorUpdateStartNull;
            bbDevice.vTable.updateComplete = motorUpdateCompleteNull;
            bbStatus = DSHOT_BITBANG_STATUS_MOTOR_PIN_CONFLICT;
            return NULL;
        }

        int pinIndex = IO_GPIOPinIdx(io);

        bbMotors[motorIndex].pinIndex = pinIndex;
        bbMotors[motorIndex].io = io;
        bbMotors[motorIndex].output = output;
#if defined(STM32F4) || defined(STM32F3)
        bbMotors[motorIndex].iocfg = IO_CONFIG(GPIO_Mode_OUT, GPIO_Speed_50MHz, GPIO_OType_PP, bbPuPdMode);
#elif defined(STM32F7) || defined(STM32G4) || defined(STM32H7)
        bbMotors[motorIndex].iocfg = IO_CONFIG(GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_VERY_HIGH, bbPuPdMode);
#endif

        IOInit(io, OWNER_MOTOR, RESOURCE_INDEX(motorIndex));
        IOConfigGPIO(io, bbMotors[motorIndex].iocfg);
        if (output & TIMER_OUTPUT_INVERTED) {
            IOLo(io);
        } else {
            IOHi(io);
        }

        // Fill in motors structure for 4way access (XXX Should be refactored)
        motors[motorIndex].io = bbMotors[motorIndex].io;
    }

    return &bbDevice;
}

#endif // USE_DSHOT_BB
