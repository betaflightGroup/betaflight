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

#include "drivers/resource.h"

// dmaResource_t is a opaque data type which represents a single DMA engine,
// called and implemented differently in different families of STM32s.
// The opaque data type provides uniform handling of the engine in source code.
// The engines are referenced by dmaResource_t through out the Betaflight code,
// and then converted back to DMA_ARCH_TYPE which is a native type for
// the particular MCU type when calling library functions.

typedef struct dmaResource_s dmaResource_t;

#if defined(STM32F4) || defined(STM32F7)
#define DMA_ARCH_TYPE DMA_Stream_TypeDef
#elif defined(STM32H7)
// H7 has stream based DMA and channel based BDMA, but we ignore BDMA (for now).
#define DMA_ARCH_TYPE DMA_Stream_TypeDef
#else
#define DMA_ARCH_TYPE DMA_Channel_TypeDef
#endif

struct dmaChannelDescriptor_s;
typedef void (*dmaCallbackHandlerFuncPtr)(struct dmaChannelDescriptor_s *channelDescriptor);

typedef struct dmaChannelDescriptor_s {
    DMA_TypeDef*                dma;
    dmaResource_t               *ref;
#if defined(STM32F4) || defined(STM32F7) || defined(STM32H7) || defined(STM32G4)
    uint8_t                     stream;
#endif
    dmaCallbackHandlerFuncPtr   irqHandlerCallback;
    uint8_t                     flagsShift;
    IRQn_Type                   irqN;
    uint32_t                    userParam;
    resourceOwner_t             owner;
    uint8_t                     resourceIndex;
    uint32_t                    completeFlag;
} dmaChannelDescriptor_t;

#if defined(STM32F7)
//#define HAL_CLEANINVALIDATECACHE(addr, size) (SCB_CleanInvalidateDCache_by_Addr((uint32_t*)((uint32_t)addr & ~0x1f), ((uint32_t)(addr + size + 0x1f) & ~0x1f) - ((uint32_t)addr & ~0x1f)))
//#define HAL_CLEANCACHE(addr, size) (SCB_CleanDCache_by_Addr((uint32_t*)((uint32_t)addr & ~0x1f), ((uint32_t)(addr + size + 0x1f) & ~0x1f) - ((uint32_t)addr & ~0x1f)))

#endif

#define DMA_IDENTIFIER_TO_INDEX(x) ((x) - 1)

#if defined(STM32F4) || defined(STM32F7) || defined(STM32H7)

typedef enum {
    DMA_NONE = 0,
    DMA1_ST0_HANDLER = 1,
    DMA1_ST1_HANDLER,
    DMA1_ST2_HANDLER,
    DMA1_ST3_HANDLER,
    DMA1_ST4_HANDLER,
    DMA1_ST5_HANDLER,
    DMA1_ST6_HANDLER,
    DMA1_ST7_HANDLER,
    DMA2_ST0_HANDLER,
    DMA2_ST1_HANDLER,
    DMA2_ST2_HANDLER,
    DMA2_ST3_HANDLER,
    DMA2_ST4_HANDLER,
    DMA2_ST5_HANDLER,
    DMA2_ST6_HANDLER,
    DMA2_ST7_HANDLER,
    DMA_LAST_HANDLER = DMA2_ST7_HANDLER
} dmaIdentifier_e;

#define DMA_DEVICE_NO(x)    ((((x)-1) / 8) + 1)
#define DMA_DEVICE_INDEX(x) ((((x)-1) % 8))
#define DMA_OUTPUT_INDEX    0
#define DMA_OUTPUT_STRING   "DMA%d Stream %d:"
#define DMA_INPUT_STRING    "DMA%d_ST%d"

#define DEFINE_DMA_CHANNEL(d, s, f) { \
    .dma = d, \
    .ref = (dmaResource_t *)d ## _Stream ## s, \
    .stream = s, \
    .irqHandlerCallback = NULL, \
    .flagsShift = f, \
    .irqN = d ## _Stream ## s ## _IRQn, \
    .userParam = 0, \
    .owner.owner = 0, \
    .owner.resourceIndex = 0 \
    }

#define DEFINE_DMA_IRQ_HANDLER(d, s, i) void DMA ## d ## _Stream ## s ## _IRQHandler(void) {\
                                                                const uint8_t index = DMA_IDENTIFIER_TO_INDEX(i); \
                                                                dmaCallbackHandlerFuncPtr handler = dmaDescriptors[index].irqHandlerCallback; \
                                                                if (handler) \
                                                                    handler(&dmaDescriptors[index]); \
                                                            }

#define DMA_CLEAR_FLAG(d, flag) if (d->flagsShift > 31) d->dma->HIFCR = (flag << (d->flagsShift - 32)); else d->dma->LIFCR = (flag << d->flagsShift)
#define DMA_GET_FLAG_STATUS(d, flag) (d->flagsShift > 31 ? d->dma->HISR & (flag << (d->flagsShift - 32)): d->dma->LISR & (flag << d->flagsShift))


#define DMA_IT_TCIF         ((uint32_t)0x00000020)
#define DMA_IT_HTIF         ((uint32_t)0x00000010)
#define DMA_IT_TEIF         ((uint32_t)0x00000008)
#define DMA_IT_DMEIF        ((uint32_t)0x00000004)
#define DMA_IT_FEIF         ((uint32_t)0x00000001)

dmaIdentifier_e dmaGetIdentifier(const dmaResource_t *stream);
dmaChannelDescriptor_t* dmaGetDmaDescriptor(const dmaResource_t *stream);
dmaResource_t *dmaGetRefByIdentifier(const dmaIdentifier_e identifier);
uint32_t dmaGetChannel(const uint8_t channel);

#else

#if defined(STM32G4)

typedef enum {
    DMA_NONE = 0,
    DMA1_CH1_HANDLER = 1,
    DMA1_CH2_HANDLER,
    DMA1_CH3_HANDLER,
    DMA1_CH4_HANDLER,
    DMA1_CH5_HANDLER,
    DMA1_CH6_HANDLER,
    DMA1_CH7_HANDLER,
    DMA1_CH8_HANDLER,
    DMA2_CH1_HANDLER,
    DMA2_CH2_HANDLER,
    DMA2_CH3_HANDLER,
    DMA2_CH4_HANDLER,
    DMA2_CH5_HANDLER,
    DMA2_CH6_HANDLER,
    DMA2_CH7_HANDLER,
    DMA2_CH8_HANDLER,
    DMA_LAST_HANDLER = DMA2_CH8_HANDLER
} dmaIdentifier_e;

#define DMA_DEVICE_NO(x)    ((((x)-1) / 8) + 1)
#define DMA_DEVICE_INDEX(x) ((((x)-1) % 8) + 1)

uint32_t dmaGetChannel(const uint8_t channel);

#else // !STM32G4

typedef enum {
    DMA_NONE = 0,
    DMA1_CH1_HANDLER = 1,
    DMA1_CH2_HANDLER,
    DMA1_CH3_HANDLER,
    DMA1_CH4_HANDLER,
    DMA1_CH5_HANDLER,
    DMA1_CH6_HANDLER,
    DMA1_CH7_HANDLER,
#if defined(STM32F3) || defined(STM32F10X_CL)
    DMA2_CH1_HANDLER,
    DMA2_CH2_HANDLER,
    DMA2_CH3_HANDLER,
    DMA2_CH4_HANDLER,
    DMA2_CH5_HANDLER,
    DMA_LAST_HANDLER = DMA2_CH5_HANDLER
#else
    DMA_LAST_HANDLER = DMA1_CH7_HANDLER
#endif
} dmaIdentifier_e;

#define DMA_DEVICE_NO(x)    ((((x)-1) / 7) + 1)
#define DMA_DEVICE_INDEX(x) ((((x)-1) % 7) + 1)

#endif // STM32G4

#define DMA_OUTPUT_INDEX    0
#define DMA_OUTPUT_STRING   "DMA%d Channel %d:"
#define DMA_INPUT_STRING    "DMA%d_CH%d"

#define DEFINE_DMA_CHANNEL(d, c, f) { \
    .dma = d, \
    .ref = (dmaResource_t *)d ## _Channel ## c, \
    .irqHandlerCallback = NULL, \
    .flagsShift = f, \
    .irqN = d ## _Channel ## c ## _IRQn, \
    .userParam = 0, \
    .owner.owner = 0, \
    .owner.resourceIndex = 0 \
    }

#if defined(USE_CCM_CODE) && defined(STM32F3)
#define DMA_HANDLER_CODE CCM_CODE
#else
#define DMA_HANDLER_CODE
#endif

#define DEFINE_DMA_IRQ_HANDLER(d, c, i) DMA_HANDLER_CODE void DMA ## d ## _Channel ## c ## _IRQHandler(void) {\
                                                                        const uint8_t index = DMA_IDENTIFIER_TO_INDEX(i); \
                                                                        dmaCallbackHandlerFuncPtr handler = dmaDescriptors[index].irqHandlerCallback; \
                                                                        if (handler) \
                                                                            handler(&dmaDescriptors[index]); \
                                                                    }

#define DMA_CLEAR_FLAG(d, flag) d->dma->IFCR = (flag << d->flagsShift)
#define DMA_GET_FLAG_STATUS(d, flag) (d->dma->ISR & (flag << d->flagsShift))

#define DMA_IT_TCIF         ((uint32_t)0x00000002)
#define DMA_IT_HTIF         ((uint32_t)0x00000004)
#define DMA_IT_TEIF         ((uint32_t)0x00000008)

dmaIdentifier_e dmaGetIdentifier(const dmaResource_t* channel);
dmaResource_t* dmaGetRefByIdentifier(const dmaIdentifier_e identifier);

#endif

// Macros to avoid direct register and register bit access

#if defined(STM32F4) || defined(STM32F7)
#define IS_DMA_ENABLED(reg) (((DMA_ARCH_TYPE *)(reg))->CR & DMA_SxCR_EN)
#define REG_NDTR NDTR
#elif defined(STM32H7)
// For H7, we have to differenciate DMA1/2 and BDMA for access to the control register.
// HAL library has a macro for this, but it is extremely inefficient in that it compares
// the address against all possible values.
// Here, we just compare the address against regions of memory.
#if defined(STM32H7A3xx) || defined(STM32H7A3xxQ)
// For H7A3, if it's lower than CD_AHB2PERIPH_BASE, then it's DMA1/2 and it's stream based.
// If not, it's BDMA and it's channel based.
#define IS_DMA_ENABLED(reg) \
    ((uint32_t)(reg) < CD_AHB2PERIPH_BASE) ? \
        (((DMA_Stream_TypeDef *)(reg))->CR & DMA_SxCR_EN) : \
        (((BDMA_Channel_TypeDef *)(reg))->CCR & BDMA_CCR_EN)
#else
// For H743 and H750, if it's not in D3 peripheral area, then it's DMA1/2 and it's stream based.
// If not, it's BDMA and it's channel based.
#define IS_DMA_ENABLED(reg) \
    ((uint32_t)(reg) < D3_AHB1PERIPH_BASE) ? \
        (((DMA_Stream_TypeDef *)(reg))->CR & DMA_SxCR_EN) : \
        (((BDMA_Channel_TypeDef *)(reg))->CCR & BDMA_CCR_EN)
#endif
#elif defined(STM32G4)
#define IS_DMA_ENABLED(reg) (((DMA_ARCH_TYPE *)(reg))->CCR & DMA_CCR_EN)
// Missing __HAL_DMA_SET_COUNTER in FW library V1.0.0
#define __HAL_DMA_SET_COUNTER(__HANDLE__, __COUNTER__) ((__HANDLE__)->Instance->CNDTR = (uint16_t)(__COUNTER__))
#else
#if defined(STM32F1)
#define DMA_CCR_EN 1 // Not defined anywhere ...
#endif
#define IS_DMA_ENABLED(reg) (((DMA_ARCH_TYPE *)(reg))->CCR & DMA_CCR_EN)
#define DMAx_SetMemoryAddress(reg, address) ((DMA_ARCH_TYPE *)(reg))->CMAR = (uint32_t)&s->port.txBuffer[s->port.txBufferTail]
#endif

void dmaInit(dmaIdentifier_e identifier, resourceOwner_e owner, uint8_t resourceIndex);
void dmaSetHandler(dmaIdentifier_e identifier, dmaCallbackHandlerFuncPtr callback, uint32_t priority, uint32_t userParam);

const resourceOwner_t *dmaGetOwner(dmaIdentifier_e identifier);
dmaChannelDescriptor_t* dmaGetDescriptorByIdentifier(const dmaIdentifier_e identifier);

//
// Wrapper macros to cast dmaResource_t back into DMA_ARCH_TYPE
//

#ifdef USE_HAL_DRIVER

// We actually need these LL case only

#define xLL_EX_DMA_DeInit(dmaResource) LL_EX_DMA_DeInit((DMA_ARCH_TYPE *)(dmaResource))
#define xLL_EX_DMA_Init(dmaResource, initstruct) LL_EX_DMA_Init((DMA_ARCH_TYPE *)(dmaResource), initstruct)
#define xLL_EX_DMA_DisableResource(dmaResource) LL_EX_DMA_DisableResource((DMA_ARCH_TYPE *)(dmaResource))
#define xLL_EX_DMA_EnableResource(dmaResource) LL_EX_DMA_EnableResource((DMA_ARCH_TYPE *)(dmaResource))
#define xLL_EX_DMA_GetDataLength(dmaResource) LL_EX_DMA_GetDataLength((DMA_ARCH_TYPE *)(dmaResource))
#define xLL_EX_DMA_SetDataLength(dmaResource, length) LL_EX_DMA_SetDataLength((DMA_ARCH_TYPE *)(dmaResource), length)
#define xLL_EX_DMA_EnableIT_TC(dmaResource) LL_EX_DMA_EnableIT_TC((DMA_ARCH_TYPE *)(dmaResource))

#else

#define xDMA_Init(dmaResource, initStruct) DMA_Init((DMA_ARCH_TYPE *)(dmaResource), initStruct)
#define xDMA_DeInit(dmaResource) DMA_DeInit((DMA_ARCH_TYPE *)(dmaResource))
#define xDMA_Cmd(dmaResource, newState) DMA_Cmd((DMA_ARCH_TYPE *)(dmaResource), newState)
#define xDMA_ITConfig(dmaResource, flags, newState) DMA_ITConfig((DMA_ARCH_TYPE *)(dmaResource), flags, newState)
#define xDMA_GetCurrDataCounter(dmaResource) DMA_GetCurrDataCounter((DMA_ARCH_TYPE *)(dmaResource))
#define xDMA_SetCurrDataCounter(dmaResource, count) DMA_SetCurrDataCounter((DMA_ARCH_TYPE *)(dmaResource), count)
#define xDMA_GetFlagStatus(dmaResource, flags) DMA_GetFlagStatus((DMA_ARCH_TYPE *)(dmaResource), flags)
#define xDMA_ClearFlag(dmaResource, flags) DMA_ClearFlag((DMA_ARCH_TYPE *)(dmaResource), flags)
#define xDMA_MemoryTargetConfig(dmaResource, address, target) DMA_MemoryTargetConfig((DMA_ARCH_TYPE *)(dmaResource), address, target)

#endif
