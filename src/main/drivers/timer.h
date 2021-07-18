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

#include <stdbool.h>
#include <stdint.h>

#include "drivers/dma.h"
#include "drivers/io_types.h"
#include "drivers/rcc_types.h"
#include "drivers/resource.h"
#include "drivers/timer_def.h"

#include "pg/timerio.h"

#define CC_CHANNELS_PER_TIMER         4 // TIM_Channel_1..4
#define CC_INDEX_FROM_CHANNEL(x)      ((uint8_t)((x) >> 2))
#define CC_CHANNEL_FROM_INDEX(x)      ((uint16_t)(x) << 2)

typedef uint16_t captureCompare_t;        // 16 bit on both 103 and 303, just register access must be 32bit sometimes (use timCCR_t)

#if defined(STM32F3) || defined(STM32F4) || defined(STM32F7) || defined(STM32H7) || defined(STM32G4) || defined(UNIT_TEST) || defined(SIMULATOR_BUILD)
typedef uint32_t timCCR_t;
typedef uint32_t timCCER_t;
typedef uint32_t timSR_t;
typedef uint32_t timCNT_t;
#elif defined(STM32F1)
typedef uint16_t timCCR_t;
typedef uint16_t timCCER_t;
typedef uint16_t timSR_t;
typedef uint16_t timCNT_t;
#else
#error "Unknown CPU defined"
#endif

typedef enum {
    TIM_USE_ANY            = 0x0,
    TIM_USE_NONE           = 0x0,
    TIM_USE_PPM            = 0x1,
    TIM_USE_PWM            = 0x2,
    TIM_USE_MOTOR          = 0x4,
    TIM_USE_SERVO          = 0x8,
    TIM_USE_LED            = 0x10,
    TIM_USE_TRANSPONDER    = 0x20,
    TIM_USE_BEEPER         = 0x40,
    TIM_USE_CAMERA_CONTROL = 0x80,
    TIM_USE_VIDEO_SYNC     = 0x100,
    TIM_USE_VIDEO_PIXEL    = 0x200,
} timerUsageFlag_e;

// use different types from capture and overflow - multiple overflow handlers are implemented as linked list
struct timerCCHandlerRec_s;
struct timerOvrHandlerRec_s;
typedef void timerCCHandlerCallback(struct timerCCHandlerRec_s* self, uint16_t capture);
typedef void timerOvrHandlerCallback(struct timerOvrHandlerRec_s* self, uint16_t capture);

typedef struct timerCCHandlerRec_s {
    timerCCHandlerCallback* fn;
} timerCCHandlerRec_t;

typedef struct timerOvrHandlerRec_s {
    timerOvrHandlerCallback* fn;
    struct timerOvrHandlerRec_s* next;
} timerOvrHandlerRec_t;

typedef struct timerDef_s {
    TIM_TypeDef *TIMx;
    rccPeriphTag_t rcc;
    uint8_t inputIrq;
} timerDef_t;

typedef struct timerHardware_s {
    TIM_TypeDef *tim;
    ioTag_t tag;
    uint8_t channel;
    timerUsageFlag_e usageFlags;
    uint8_t output;
#if defined(STM32F3) || defined(STM32F4) || defined(STM32F7) || defined(STM32H7) || defined(STM32G4)
    uint8_t alternateFunction;
#endif

#if defined(USE_TIMER_DMA)

#if defined(USE_DMA_SPEC)
#if defined(STM32F4) || defined(STM32F7) || defined(STM32H7) || defined(STM32G4)
    dmaResource_t *dmaRefConfigured;
    uint32_t dmaChannelConfigured;
#endif
#else // USE_DMA_SPEC
    dmaResource_t *dmaRef;
#if defined(STM32F4) || defined(STM32F7) || defined(STM32H7) || defined(STM32G4)
    uint32_t dmaChannel; // XXX Can be much smaller (e.g. uint8_t)
#endif
#endif // USE_DMA_SPEC
    dmaResource_t *dmaTimUPRef;
#if defined(STM32F4) || defined(STM32F7) || defined(STM32H7) || defined(STM32G4)
    uint32_t dmaTimUPChannel;
#endif
    uint8_t dmaTimUPIrqHandler;
#endif
} timerHardware_t;

typedef enum {
    TIMER_OUTPUT_NONE      = 0,
    TIMER_OUTPUT_INVERTED  = (1 << 0),
    TIMER_OUTPUT_N_CHANNEL = (1 << 1),
} timerFlag_e;

#ifdef STM32F1
#if defined(STM32F10X_XL) || defined(STM32F10X_HD_VL)
#define HARDWARE_TIMER_DEFINITION_COUNT 14
#elif defined(STM32F10X_HD) || defined(STM32F10X_CL)
#define HARDWARE_TIMER_DEFINITION_COUNT 7
#else
#define HARDWARE_TIMER_DEFINITION_COUNT 4
#endif
#elif defined(STM32F3)
#define HARDWARE_TIMER_DEFINITION_COUNT 10
#elif defined(STM32F411xE)
#define HARDWARE_TIMER_DEFINITION_COUNT 10
#elif defined(STM32F4)
#define HARDWARE_TIMER_DEFINITION_COUNT 14
#elif defined(STM32F7)
#define HARDWARE_TIMER_DEFINITION_COUNT 14
#elif defined(STM32H7)
#define HARDWARE_TIMER_DEFINITION_COUNT 17
#define TIMUP_TIMERS ( BIT(1) | BIT(2) | BIT(3) | BIT(4) | BIT(5) | BIT(6) | BIT(7) | BIT(8) | BIT(15) | BIT(16) | BIT(17) )
#elif defined(STM32G4)
#define HARDWARE_TIMER_DEFINITION_COUNT 12
#define TIMUP_TIMERS ( BIT(1) | BIT(2) | BIT(3) | BIT(4) | BIT(5) | BIT(6) | BIT(7) | BIT(8) | BIT(15) | BIT(16) | BIT(17) | BIT(20))
#endif

#define MHZ_TO_HZ(x) ((x) * 1000000)

#if !defined(USE_UNIFIED_TARGET)
extern const timerHardware_t timerHardware[];
#endif


#if defined(USE_TIMER_MGMT)
#if defined(STM32F3)

#define FULL_TIMER_CHANNEL_COUNT 88

#elif defined(STM32F4)

#if defined(STM32F411xE)
#define FULL_TIMER_CHANNEL_COUNT 59
#else
#define FULL_TIMER_CHANNEL_COUNT 78
#endif

#elif defined(STM32F7)

#define FULL_TIMER_CHANNEL_COUNT 78

#elif defined(STM32H7)

#define FULL_TIMER_CHANNEL_COUNT 87

#elif defined(STM32G4)

#define FULL_TIMER_CHANNEL_COUNT 93 // XXX Need review

#endif

extern const timerHardware_t fullTimerHardware[];

#define TIMER_CHANNEL_COUNT FULL_TIMER_CHANNEL_COUNT
#define TIMER_HARDWARE fullTimerHardware

#if defined(STM32F7) || defined(STM32F4)

#if defined(STM32F411xE)
#define USED_TIMERS ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(5) | TIM_N(6) | TIM_N(7) | TIM_N(9) | TIM_N(10) | TIM_N(11) )
#else
#define USED_TIMERS ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(5) | TIM_N(6) | TIM_N(7) | TIM_N(8) | TIM_N(9) | TIM_N(10) | TIM_N(11) | TIM_N(12) | TIM_N(13) | TIM_N(14) )
#endif

#elif defined(STM32F3)

#define USED_TIMERS ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(6) | TIM_N(7) | TIM_N(8) | TIM_N(15) | TIM_N(16) | TIM_N(17) )

#elif defined(STM32F1)

#define USED_TIMERS ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) )

#elif defined(STM32H7)

#define USED_TIMERS ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(5) | TIM_N(6) | TIM_N(7) | TIM_N(8) | TIM_N(12) | TIM_N(13) | TIM_N(14) | TIM_N(15) | TIM_N(16) | TIM_N(17) )

#elif defined(STM32G4)

#define USED_TIMERS ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(5) | TIM_N(6) | TIM_N(7) | TIM_N(8) | TIM_N(15) | TIM_N(16) | TIM_N(17) | TIM_N(20) )

#else
    #error "No timer / channel tag definition found for CPU"
#endif

#else

#define TIMER_CHANNEL_COUNT USABLE_TIMER_CHANNEL_COUNT
#define TIMER_HARDWARE timerHardware

#endif // USE_TIMER_MGMT

extern const timerDef_t timerDefinitions[];

typedef enum {
    TYPE_FREE,
    TYPE_PWMINPUT,
    TYPE_PPMINPUT,
    TYPE_PWMOUTPUT_MOTOR,
    TYPE_PWMOUTPUT_FAST,
    TYPE_PWMOUTPUT_SERVO,
    TYPE_SOFTSERIAL_RX,
    TYPE_SOFTSERIAL_TX,
    TYPE_SOFTSERIAL_RXTX,        // bidirectional pin for softserial
    TYPE_SOFTSERIAL_AUXTIMER,    // timer channel is used for softserial. No IO function on pin
    TYPE_ADC,
    TYPE_SERIAL_RX,
    TYPE_SERIAL_TX,
    TYPE_SERIAL_RXTX,
    TYPE_TIMER
} channelType_t;

void timerConfigure(const timerHardware_t *timHw, uint16_t period, uint32_t hz);  // This interface should be replaced.

void timerChConfigIC(const timerHardware_t *timHw, bool polarityRising, unsigned inputFilterSamples);
void timerChConfigICDual(const timerHardware_t* timHw, bool polarityRising, unsigned inputFilterSamples);
void timerChICPolarity(const timerHardware_t *timHw, bool polarityRising);
volatile timCCR_t* timerChCCR(const timerHardware_t* timHw);
volatile timCCR_t* timerChCCRLo(const timerHardware_t* timHw);
volatile timCCR_t* timerChCCRHi(const timerHardware_t* timHw);
void timerChConfigOC(const timerHardware_t* timHw, bool outEnable, bool stateHigh);
void timerChConfigGPIO(const timerHardware_t* timHw, ioConfig_t mode);

void timerChCCHandlerInit(timerCCHandlerRec_t *self, timerCCHandlerCallback *fn);
void timerChOvrHandlerInit(timerOvrHandlerRec_t *self, timerOvrHandlerCallback *fn);
void timerChConfigCallbacks(const timerHardware_t *channel, timerCCHandlerRec_t *edgeCallback, timerOvrHandlerRec_t *overflowCallback);
void timerChConfigCallbacksDual(const timerHardware_t *channel, timerCCHandlerRec_t *edgeCallbackLo, timerCCHandlerRec_t *edgeCallbackHi, timerOvrHandlerRec_t *overflowCallback);
void timerChITConfigDualLo(const timerHardware_t* timHw, FunctionalState newState);
void timerChITConfig(const timerHardware_t* timHw, FunctionalState newState);
void timerChClearCCFlag(const timerHardware_t* timHw);

void timerChInit(const timerHardware_t *timHw, channelType_t type, int irqPriority, uint8_t irq);

void timerInit(void);
void timerStart(void);
void timerForceOverflow(TIM_TypeDef *tim);

uint32_t timerClock(TIM_TypeDef *tim);

void configTimeBase(TIM_TypeDef *tim, uint16_t period, uint32_t hz);  // TODO - just for migration

rccPeriphTag_t timerRCC(TIM_TypeDef *tim);
uint8_t timerInputIrq(TIM_TypeDef *tim);

#if defined(USE_TIMER_MGMT)
extern const resourceOwner_t freeOwner;

struct timerIOConfig_s;

struct timerIOConfig_s *timerIoConfigByTag(ioTag_t ioTag);
const resourceOwner_t *timerGetOwner(int8_t timerNumber, uint16_t timerChannel);
#endif
const timerHardware_t *timerGetByTag(ioTag_t ioTag);
const timerHardware_t *timerAllocate(ioTag_t ioTag, resourceOwner_e owner, uint8_t resourceIndex);
const timerHardware_t *timerGetByTagAndIndex(ioTag_t ioTag, unsigned timerIndex);
ioTag_t timerioTagGetByUsage(timerUsageFlag_e usageFlag, uint8_t index);

#if defined(USE_HAL_DRIVER)
TIM_HandleTypeDef* timerFindTimerHandle(TIM_TypeDef *tim);
HAL_StatusTypeDef TIM_DMACmd(TIM_HandleTypeDef *htim, uint32_t Channel, FunctionalState NewState);
HAL_StatusTypeDef DMA_SetCurrDataCounter(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData, uint16_t Length);
uint16_t timerDmaIndex(uint8_t channel);
#else
void timerOCInit(TIM_TypeDef *tim, uint8_t channel, TIM_OCInitTypeDef *init);
void timerOCPreloadConfig(TIM_TypeDef *tim, uint8_t channel, uint16_t preload);
#endif

volatile timCCR_t *timerCCR(TIM_TypeDef *tim, uint8_t channel);
uint16_t timerDmaSource(uint8_t channel);

uint16_t timerGetPrescalerByDesiredHertz(TIM_TypeDef *tim, uint32_t hz);
uint16_t timerGetPrescalerByDesiredMhz(TIM_TypeDef *tim, uint16_t mhz);
uint16_t timerGetPeriodByPrescaler(TIM_TypeDef *tim, uint16_t prescaler, uint32_t hz);

int8_t timerGetNumberByIndex(uint8_t index);
int8_t timerGetTIMNumber(const TIM_TypeDef *tim);
uint8_t timerLookupChannelIndex(const uint16_t channel);
