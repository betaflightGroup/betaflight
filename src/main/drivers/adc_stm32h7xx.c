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

#ifdef USE_ADC

#include "build/debug.h"

#include "drivers/dma_reqmap.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/rcc.h"
#include "drivers/resource.h"
#include "drivers/dma.h"

#include "drivers/sensor.h"

#include "drivers/adc.h"
#include "drivers/adc_impl.h"

#include "pg/adc.h"

// XXX Instance and DMA stream defs will be gone in unified target

#ifndef ADC1_INSTANCE
#define ADC1_INSTANCE NULL
#endif
#ifndef ADC2_INSTANCE
#define ADC2_INSTANCE NULL
#endif
#ifndef ADC3_INSTANCE
#define ADC3_INSTANCE NULL
#endif
#ifndef ADC1_DMA_STREAM
#define ADC1_DMA_STREAM NULL
#endif
#ifndef ADC2_DMA_STREAM
#define ADC2_DMA_STREAM NULL
#endif
#ifndef ADC3_DMA_STREAM
#define ADC3_DMA_STREAM NULL
#endif

const adcDevice_t adcHardware[ADCDEV_COUNT] = {
    {
        .ADCx = ADC1_INSTANCE,
        .rccADC = RCC_AHB1(ADC12),
#if !defined(USE_DMA_SPEC)
        .dmaResource = (dmaResource_t *)ADC1_DMA_STREAM,
        .channel = DMA_REQUEST_ADC1,
#endif
    },
    { .ADCx = ADC2_INSTANCE,
        .rccADC = RCC_AHB1(ADC12),
#if !defined(USE_DMA_SPEC)
        .dmaResource = (dmaResource_t *)ADC2_DMA_STREAM,
        .channel = DMA_REQUEST_ADC2,
#endif
    },
#if defined(ADC3)
    // ADC3 is not available on all H7 MCUs, e.g. H7A3
    // On H743 and H750, ADC3 can be serviced by BDMA also, but we settle for DMA1 or 2 (for now).
    {
        .ADCx = ADC3_INSTANCE,
        .rccADC = RCC_AHB4(ADC3),
#if !defined(USE_DMA_SPEC)
        .dmaResource = (dmaResource_t *)ADC3_DMA_STREAM,
        .channel = DMA_REQUEST_ADC3,
#endif
    }
#endif // ADC3
};

adcDevice_t adcDevice[ADCDEV_COUNT];

#if defined(STM32H743xx) || defined(STM32H750xx) || defined(STM32H723xx) || defined(STM32H725xx) || defined(STM32H730xx)
#define ADC_DEVICE_FOR_INTERNAL ADC_DEVICES_3
#elif defined(STM32H7A3xx) || defined(STM32H7A3xxQ)
#define ADC_DEVICE_FOR_INTERNAL ADC_DEVICES_2
#else
#error Unknown MCU
#endif

/* note these could be packed up for saving space */
const adcTagMap_t adcTagMap[] = {
#ifdef USE_ADC_INTERNAL
    // Pseudo entries for internal sensor.
    // Keep these at the beginning for easy indexing by ADC_TAG_MAP_{VREFINT,TEMPSENSOR}
#define ADC_TAG_MAP_VREFINT    0
#define ADC_TAG_MAP_TEMPSENSOR 1

#if defined(STM32H743xx) || defined(STM32H750xx) || defined(STM32H7A3xx) || defined(STM32H7A3xxQ)
    { DEFIO_TAG_E__NONE, ADC_DEVICE_FOR_INTERNAL,   ADC_CHANNEL_VREFINT,    18 },
    { DEFIO_TAG_E__NONE, ADC_DEVICE_FOR_INTERNAL,   ADC_CHANNEL_TEMPSENSOR, 17 },
#elif defined(STM32H723xx) || defined(STM32H725xx)
    { DEFIO_TAG_E__NONE, ADC_DEVICE_FOR_INTERNAL,   ADC_CHANNEL_VREFINT,    18 },
    { DEFIO_TAG_E__NONE, ADC_DEVICE_FOR_INTERNAL,   ADC_CHANNEL_TEMPSENSOR, 19 },
#endif
#endif
#if defined(STM32H7A3xx) || defined(STM32H7A3xxQ)
    // See DS13195 Rev 6 Page 51/52
    { DEFIO_TAG_E__PC0,  ADC_DEVICES_12,  ADC_CHANNEL_10, 10 },
    { DEFIO_TAG_E__PC1,  ADC_DEVICES_12,  ADC_CHANNEL_11, 11 },
    { DEFIO_TAG_E__PC2,  ADC_DEVICES_12,  ADC_CHANNEL_12,  0 },
    { DEFIO_TAG_E__PC3,  ADC_DEVICES_12,  ADC_CHANNEL_13,  1 },
#else
    { DEFIO_TAG_E__PC0,  ADC_DEVICES_123, ADC_CHANNEL_10, 10 },
    { DEFIO_TAG_E__PC1,  ADC_DEVICES_123, ADC_CHANNEL_11, 11 },
    { DEFIO_TAG_E__PC2,  ADC_DEVICES_3,   ADC_CHANNEL_0,   0 },
    { DEFIO_TAG_E__PC3,  ADC_DEVICES_3,   ADC_CHANNEL_1,   1 },
#endif
    { DEFIO_TAG_E__PC4,  ADC_DEVICES_12,  ADC_CHANNEL_4,   4 },
    { DEFIO_TAG_E__PC5,  ADC_DEVICES_12,  ADC_CHANNEL_8,   8 },
    { DEFIO_TAG_E__PB0,  ADC_DEVICES_12,  ADC_CHANNEL_9,   9 },
    { DEFIO_TAG_E__PB1,  ADC_DEVICES_12,  ADC_CHANNEL_5,   5 },
    { DEFIO_TAG_E__PA0,  ADC_DEVICES_1,   ADC_CHANNEL_16, 16 },
    { DEFIO_TAG_E__PA1,  ADC_DEVICES_1,   ADC_CHANNEL_17, 17 },
    { DEFIO_TAG_E__PA2,  ADC_DEVICES_12,  ADC_CHANNEL_14, 14 },
    { DEFIO_TAG_E__PA3,  ADC_DEVICES_12,  ADC_CHANNEL_15, 15 },
    { DEFIO_TAG_E__PA4,  ADC_DEVICES_12,  ADC_CHANNEL_18, 18 },
    { DEFIO_TAG_E__PA5,  ADC_DEVICES_12,  ADC_CHANNEL_19, 19 },
    { DEFIO_TAG_E__PA6,  ADC_DEVICES_12,  ADC_CHANNEL_3,   3 },
    { DEFIO_TAG_E__PA7,  ADC_DEVICES_12,  ADC_CHANNEL_7,   7 },

#if 0
    // Inputs available for packages larger than LQFP144
    { DEFIO_TAG_E__PF3,  ADC_DEVICES_3,   ADC_CHANNEL_5,   5 },
    { DEFIO_TAG_E__PF4,  ADC_DEVICES_3,   ADC_CHANNEL_9,   9 },
    { DEFIO_TAG_E__PF5,  ADC_DEVICES_3,   ADC_CHANNEL_4,   4 },
    { DEFIO_TAG_E__PF6,  ADC_DEVICES_3,   ADC_CHANNEL_8,   8 },
    { DEFIO_TAG_E__PF7,  ADC_DEVICES_3,   ADC_CHANNEL_3,   3 },
    { DEFIO_TAG_E__PF8,  ADC_DEVICES_3,   ADC_CHANNEL_7,   7 },
    { DEFIO_TAG_E__PF9,  ADC_DEVICES_3,   ADC_CHANNEL_2,   2 },
    { DEFIO_TAG_E__PF10, ADC_DEVICES_3,   ADC_CHANNEL_6,   6 },
    { DEFIO_TAG_E__PF11, ADC_DEVICES_1,   ADC_CHANNEL_2,   2 },
    { DEFIO_TAG_E__PF12, ADC_DEVICES_1,   ADC_CHANNEL_6,   6 },
    { DEFIO_TAG_E__PF13, ADC_DEVICES_2,   ADC_CHANNEL_2,   2 },
    { DEFIO_TAG_E__PF14, ADC_DEVICES_2,   ADC_CHANNEL_6,   6 },
#endif
};

// Translate rank number x to ADC_REGULAR_RANK_x (Note that array index is 0-origin)

#define RANK(n) ADC_REGULAR_RANK_ ## n

static uint32_t adcRegularRankMap[] = {
    RANK(1),
    RANK(2),
    RANK(3),
    RANK(4),
    RANK(5),
    RANK(6),
    RANK(7),
    RANK(8),
    RANK(9),
    RANK(10),
    RANK(11),
    RANK(12),
    RANK(13),
    RANK(14),
    RANK(15),
    RANK(16),
};

#undef RANK

static void errorHandler(void) { while (1) { } }

// Note on sampling time.
// Temperature sensor has minimum sample time of 9us.
// With prescaler = 4 at 200MHz (AHB1), fADC = 50MHz (tcycle = 0.02us), 9us = 450cycles < 810

void adcInitDevice(adcDevice_t *adcdev, int channelCount)
{
    ADC_HandleTypeDef *hadc = &adcdev->ADCHandle; // For clarity

    hadc->Instance = adcdev->ADCx;

    // DeInit is done in adcInit().

    hadc->Init.ClockPrescaler           = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc->Init.Resolution               = ADC_RESOLUTION_12B;
    hadc->Init.ScanConvMode             = ENABLE;                        // Works with single channel, too
    hadc->Init.EOCSelection             = ADC_EOC_SINGLE_CONV;
    hadc->Init.LowPowerAutoWait         = DISABLE;
    hadc->Init.ContinuousConvMode       = ENABLE;
    hadc->Init.NbrOfConversion          = channelCount;
    hadc->Init.DiscontinuousConvMode    = DISABLE;
    hadc->Init.NbrOfDiscConversion      = 1;                             // Don't care
    hadc->Init.ExternalTrigConv         = ADC_SOFTWARE_START;
    hadc->Init.ExternalTrigConvEdge     = ADC_EXTERNALTRIGCONVEDGE_NONE; // Don't care

    // Enable circular DMA.
    // ADC3 of H72X and H73X has a special way of doing this.
#if defined(STM32H723xx) || defined(STM32H725xx)
    if (adcdev->ADCx == ADC3) {
        hadc->Init.DMAContinuousRequests = ENABLE;
    } else
#else
    {
        hadc->Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
    }
#endif

    hadc->Init.Overrun                  = ADC_OVR_DATA_OVERWRITTEN;
    hadc->Init.OversamplingMode         = DISABLE;

    // Initialize this ADC peripheral

    if (HAL_ADC_Init(hadc) != HAL_OK) {
      errorHandler();
    }

    // Execute calibration

    if (HAL_ADCEx_Calibration_Start(hadc, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK) {
      errorHandler();
    }
}

int adcFindTagMapEntry(ioTag_t tag)
{
    for (int i = 0; i < ADC_TAG_MAP_COUNT; i++) {
        if (adcTagMap[i].tag == tag) {
            return i;
        }
    }
    return -1;
}

// H743, H750 and H7A3 seems to use 16-bit precision value,
// while H723 and H725, H730 seems to use 12-bit precision value.
#if defined(STM32H743xx) || defined(STM32H750xx) || defined(STM32H7A3xx) || defined(STM32H7A3xxQ)
#define VREFINT_CAL_SHIFT 4
#elif defined(STM32H723xx) || defined(STM32H725xx) || defined(STM32H730xx)
#define VREFINT_CAL_SHIFT 0
#else
#error Unknown MCU
#endif

void adcInitCalibrationValues(void)
{
    adcVREFINTCAL = *VREFINT_CAL_ADDR >> VREFINT_CAL_SHIFT;
    adcTSCAL1 = *TEMPSENSOR_CAL1_ADDR >> VREFINT_CAL_SHIFT;
    adcTSCAL2 = *TEMPSENSOR_CAL2_ADDR >> VREFINT_CAL_SHIFT;
    adcTSSlopeK = (TEMPSENSOR_CAL2_TEMP - TEMPSENSOR_CAL1_TEMP) * 1000 / (adcTSCAL2 - adcTSCAL1);
}

// ADC conversion result DMA buffer
// Need this separate from the main adcValue[] array, because channels are numbered
// by ADC instance order that is different from ADC_xxx numbering.

static volatile DMA_RAM uint16_t adcConversionBuffer[ADC_CHANNEL_COUNT] __attribute__((aligned(32)));

void adcInit(const adcConfig_t *config)
{
    memset(adcOperatingConfig, 0, sizeof(adcOperatingConfig));
    memcpy(adcDevice, adcHardware, sizeof(adcDevice));

    if (config->vbat.enabled) {
        adcOperatingConfig[ADC_BATTERY].tag = config->vbat.ioTag;
    }

    if (config->rssi.enabled) {
        adcOperatingConfig[ADC_RSSI].tag = config->rssi.ioTag;  //RSSI_ADC_CHANNEL;
    }

    if (config->external1.enabled) {
        adcOperatingConfig[ADC_EXTERNAL1].tag = config->external1.ioTag; //EXTERNAL1_ADC_CHANNEL;
    }

    if (config->current.enabled) {
        adcOperatingConfig[ADC_CURRENT].tag = config->current.ioTag;  //CURRENT_METER_ADC_CHANNEL;
    }

#ifdef USE_ADC_INTERNAL
    adcInitCalibrationValues();
#endif

    for (int i = 0; i < ADC_CHANNEL_COUNT; i++) {
        int map;
        int dev;

        if (i == ADC_TEMPSENSOR) {
            map = ADC_TAG_MAP_TEMPSENSOR;
            dev = ffs(adcTagMap[map].devices) - 1;
        } else if (i == ADC_VREFINT) {
            map = ADC_TAG_MAP_VREFINT;
            dev = ffs(adcTagMap[map].devices) - 1;
        } else {
            if (!adcOperatingConfig[i].tag) {
                continue;
            }

            map = adcFindTagMapEntry(adcOperatingConfig[i].tag);
            if (map < 0) {
                continue;
            }

            // Found a tag map entry for this input pin
            // Find an ADC device that can handle this input pin

            for (dev = 0; dev < ADCDEV_COUNT; dev++) {
                if (!adcDevice[dev].ADCx
#ifndef USE_DMA_SPEC
                     || !adcDevice[dev].dmaResource
#endif
                   ) {
                    // Instance not activated
                    continue;
                }
                if (adcTagMap[map].devices & (1 << dev)) {
                    // Found an activated ADC instance for this input pin
                    break;
                }
            }

            if (dev == ADCDEV_COUNT) {
                // No valid device found, go next channel.
                continue;
            }
        }

        // At this point, map is an entry for the input pin and dev is a valid ADCx for the pin for input i

        adcOperatingConfig[i].adcDevice = dev;
        adcOperatingConfig[i].adcChannel = adcTagMap[map].channel;
        adcOperatingConfig[i].sampleTime = ADC_SAMPLETIME_810CYCLES_5;
        adcOperatingConfig[i].enabled = true;

        adcDevice[dev].channelBits |= (1 << adcTagMap[map].channelOrdinal);

        // Configure a pin for ADC
        if (adcOperatingConfig[i].tag) {
            IOInit(IOGetByTag(adcOperatingConfig[i].tag), OWNER_ADC_BATT + i, 0);
            IOConfigGPIO(IOGetByTag(adcOperatingConfig[i].tag), IO_CONFIG(GPIO_MODE_ANALOG, 0, GPIO_NOPULL));
        }
    }

    // DeInit ADCx with inputs
    // We have to batch call DeInit() for all devices as DeInit() initializes ADCx_COMMON register.

    for (int dev = 0; dev < ADCDEV_COUNT; dev++) {
        adcDevice_t *adc = &adcDevice[dev];

        if (!(adc->ADCx && adc->channelBits)) {
            continue;
        }

        adc->ADCHandle.Instance = adc->ADCx;

        if (HAL_ADC_DeInit(&adc->ADCHandle) != HAL_OK) { 
            // ADC de-initialization Error
            errorHandler();
        }
    }

    // Configure ADCx with inputs

    int dmaBufferIndex = 0;

    for (int dev = 0; dev < ADCDEV_COUNT; dev++) {
        adcDevice_t *adc = &adcDevice[dev];

        if (!(adc->ADCx && adc->channelBits)) {
            continue;
        }

        RCC_ClockCmd(adc->rccADC, ENABLE);

#ifdef USE_ADC3_DIRECT_HAL_INIT
        // XXX (Only) ADC3 (sometimes) fails to self calibrate without these? Need to verify

        // ADC Periph clock enable
        __HAL_RCC_ADC3_CLK_ENABLE();

        // ADC Periph interface clock configuration
        __HAL_RCC_ADC_CONFIG(RCC_ADCCLKSOURCE_CLKP);
#endif

        int configuredAdcChannels = BITCOUNT(adc->channelBits);

        adcInitDevice(adc, configuredAdcChannels);

        // Configure channels

        int rank = 0;

        for (int adcChan = 0; adcChan < ADC_CHANNEL_COUNT; adcChan++) {

            if (!adcOperatingConfig[adcChan].enabled) {
                continue;
            }

            if (adcOperatingConfig[adcChan].adcDevice != dev) {
                continue;
            }

            adcOperatingConfig[adcChan].dmaIndex = dmaBufferIndex++;

            ADC_ChannelConfTypeDef sConfig;

            sConfig.Channel      = adcOperatingConfig[adcChan].adcChannel; /* Sampled channel number */
            sConfig.Rank         = adcRegularRankMap[rank++];   /* Rank of sampled channel number ADCx_CHANNEL */
            sConfig.SamplingTime = ADC_SAMPLETIME_387CYCLES_5;  /* Sampling time (number of clock cycles unit) */
            sConfig.SingleDiff   = ADC_SINGLE_ENDED;            /* Single-ended input channel */
            sConfig.OffsetNumber = ADC_OFFSET_NONE;             /* No offset subtraction */
            sConfig.Offset = 0;                                 /* Parameter discarded because offset correction is disabled */

            if (HAL_ADC_ConfigChannel(&adc->ADCHandle, &sConfig) != HAL_OK) {
                errorHandler();
            }
        }

        // Configure DMA for this ADC peripheral

        dmaIdentifier_e dmaIdentifier;
#ifdef USE_DMA_SPEC
        const dmaChannelSpec_t *dmaSpec = dmaGetChannelSpecByPeripheral(DMA_PERIPH_ADC, dev, config->dmaopt[dev]);

        if (!dmaSpec) {
            return;
        }

        adc->DmaHandle.Instance                 = dmaSpec->ref;
        adc->DmaHandle.Init.Request             = dmaSpec->channel;
        dmaIdentifier = dmaGetIdentifier(dmaSpec->ref);
#else
        dmaIdentifier = dmaGetIdentifier(adc->dmaResource);
        adc->DmaHandle.Instance                 = (DMA_ARCH_TYPE *)adc->dmaResource;
        adc->DmaHandle.Init.Request             = adc->channel;
#endif
        adc->DmaHandle.Init.Direction           = DMA_PERIPH_TO_MEMORY;
        adc->DmaHandle.Init.PeriphInc           = DMA_PINC_DISABLE;
        adc->DmaHandle.Init.MemInc              = DMA_MINC_ENABLE;
        adc->DmaHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
        adc->DmaHandle.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
        adc->DmaHandle.Init.Mode                = DMA_CIRCULAR;
        adc->DmaHandle.Init.Priority            = DMA_PRIORITY_MEDIUM;

        // Deinitialize  & Initialize the DMA for new transfer

        // dmaInit must be called before calling HAL_DMA_Init,
        // to enable clock for associated DMA if not already done so.
        dmaInit(dmaIdentifier, OWNER_ADC, RESOURCE_INDEX(dev));

        HAL_DMA_DeInit(&adc->DmaHandle);
        HAL_DMA_Init(&adc->DmaHandle);

        // Associate the DMA handle

        __HAL_LINKDMA(&adc->ADCHandle, DMA_Handle, adc->DmaHandle);

#ifdef USE_ADC_INTERRUPT
        // XXX No interrupt used, so we can skip this.
        // If interrupt is needed in any case, use dmaXXX facility instead,
        // using dmaIdentifier obtained above.

        // NVIC configuration for DMA Input data interrupt

        HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 1, 0);
        HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
#endif
    }

    // Start channels.
    // This must be done after channel configuration is complete, as HAL_ADC_ConfigChannel
    // throws an error when configuring internal channels if ADC1 or ADC2 are already enabled.

    dmaBufferIndex = 0;

    for (int dev = 0; dev < ADCDEV_COUNT; dev++) {

        adcDevice_t *adc = &adcDevice[dev];

        if (!(adc->ADCx && adc->channelBits)) {
            continue;
        }

        // Start conversion in DMA mode

        if (HAL_ADC_Start_DMA(&adc->ADCHandle, (uint32_t *)&adcConversionBuffer[dmaBufferIndex], BITCOUNT(adc->channelBits)) != HAL_OK) {
            errorHandler();
        }

        dmaBufferIndex += BITCOUNT(adc->channelBits);
    }
}

void adcGetChannelValues(void)
{
    // Transfer values in conversion buffer into adcValues[]
    // Cache coherency should be maintained by MPU facility

    for (int i = 0; i < ADC_CHANNEL_INTERNAL; i++) {
        if (adcOperatingConfig[i].enabled) {
            adcValues[adcOperatingConfig[i].dmaIndex] = adcConversionBuffer[adcOperatingConfig[i].dmaIndex];
        }
    }
}

#ifdef USE_ADC_INTERNAL

bool adcInternalIsBusy(void)
{
    return false;
}

void adcInternalStartConversion(void)
{
    return;
}

uint16_t adcInternalRead(int channel)
{
    int dmaIndex = adcOperatingConfig[channel].dmaIndex;

    return adcConversionBuffer[dmaIndex];
}

uint16_t adcInternalReadVrefint(void)
{
    uint16_t value = adcInternalRead(ADC_VREFINT);
    return value;
}

uint16_t adcInternalReadTempsensor(void)
{
    uint16_t value = adcInternalRead(ADC_TEMPSENSOR);
    return value;
}
#endif // USE_ADC_INTERNAL

#endif // USE_ADC
