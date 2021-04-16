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

#ifndef ADC1_DMA_CHANNEL
#define ADC1_DMA_CHANNEL NULL
#endif
#ifndef ADC2_DMA_CHANNEL
#define ADC2_DMA_CHANNEL NULL
#endif
#ifndef ADC3_DMA_CHANNEL
#define ADC3_DMA_CHANNEL NULL
#endif
#ifndef ADC4_DMA_CHANNEL
#define ADC4_DMA_CHANNEL NULL
#endif
#ifndef ADC5_DMA_CHANNEL
#define ADC5_DMA_CHANNEL NULL
#endif

const adcDevice_t adcHardware[ADCDEV_COUNT] = {
    {
        .ADCx = ADC1,
        .rccADC = RCC_AHB2(ADC12),
#if !defined(USE_DMA_SPEC)
        .dmaResource = (dmaResource_t *)ADC1_DMA_CHANNEL,
        .channel = DMA_REQUEST_ADC1,
#endif
    },
    {
        .ADCx = ADC2,
        .rccADC = RCC_AHB2(ADC12),
#if !defined(USE_DMA_SPEC)
        .dmaResource = (dmaResource_t *)ADC2_DMA_CHANNEL,
        .channel = DMA_REQUEST_ADC2,
#endif
    },
    {
        .ADCx = ADC3,
        .rccADC = RCC_AHB2(ADC345),
#if !defined(USE_DMA_SPEC)
        .dmaResource = (dmaResource_t *)ADC3_DMA_CHANNEL,
        .channel = DMA_REQUEST_ADC3,
#endif
    },
    {
        .ADCx = ADC4,
        .rccADC = RCC_AHB2(ADC345),
#if !defined(USE_DMA_SPEC)
        .dmaResource = (dmaResource_t *)ADC3_DMA_CHANNEL,
        .channel = DMA_REQUEST_ADC4,
#endif
    },
    {
        .ADCx = ADC5,
        .rccADC = RCC_AHB2(ADC345),
#if !defined(USE_DMA_SPEC)
        .dmaResource = (dmaResource_t *)ADC5_DMA_CHANNEL,
        .channel = DMA_REQUEST_ADC5,
#endif
    }
};

adcDevice_t adcDevice[ADCDEV_COUNT];

/* note these could be packed up for saving space */
const adcTagMap_t adcTagMap[] = {
#ifdef USE_ADC_INTERNAL
    // Pseudo entries for internal sensor.
    // Keep these at the beginning for easy indexing by ADC_TAG_MAP_{VREFINT,TEMPSENSOR}
#define ADC_TAG_MAP_VREFINT    0
#define ADC_TAG_MAP_TEMPSENSOR 1
    // VREFINT is available on all ADC instances except ADC2
    // Here, for simplicity, we force VREFINT on ADC1.
    { DEFIO_TAG_E__NONE, ADC_DEVICES_1,   ADC_CHANNEL_VREFINT,         18 },
    // TEMPSENSOR is available on ADC1 or ADC5
    // Here, for simplicity, we force TEMPSENSOR on ADC1.
    { DEFIO_TAG_E__NONE, ADC_DEVICES_1,   ADC_CHANNEL_TEMPSENSOR_ADC1, 16 },
#endif
    // Inputs available for all packages under 100 pin or smaller

    { DEFIO_TAG_E__PA0,  ADC_DEVICES_12,  ADC_CHANNEL_1,   1 },
    { DEFIO_TAG_E__PA1,  ADC_DEVICES_12,  ADC_CHANNEL_2,   2 },
    { DEFIO_TAG_E__PA2,  ADC_DEVICES_1,   ADC_CHANNEL_3,   3 },
    { DEFIO_TAG_E__PA3,  ADC_DEVICES_1,   ADC_CHANNEL_4,   4 },
    { DEFIO_TAG_E__PA4,  ADC_DEVICES_2,   ADC_CHANNEL_17, 17 },
    { DEFIO_TAG_E__PA5,  ADC_DEVICES_2,   ADC_CHANNEL_13, 13 },
    { DEFIO_TAG_E__PA6,  ADC_DEVICES_2,   ADC_CHANNEL_3,   3 },
    { DEFIO_TAG_E__PA7,  ADC_DEVICES_2,   ADC_CHANNEL_4,   4 },
    { DEFIO_TAG_E__PA8,  ADC_DEVICES_5,   ADC_CHANNEL_1,   1 },
    { DEFIO_TAG_E__PA9,  ADC_DEVICES_5,   ADC_CHANNEL_2,   2 },

    { DEFIO_TAG_E__PB0,  ADC_DEVICES_1,   ADC_CHANNEL_15, 15 },
    { DEFIO_TAG_E__PB0,  ADC_DEVICES_3,   ADC_CHANNEL_12, 12 }, // 2nd entry
    { DEFIO_TAG_E__PB1,  ADC_DEVICES_1,   ADC_CHANNEL_12, 12 },
    { DEFIO_TAG_E__PB1,  ADC_DEVICES_3,   ADC_CHANNEL_1,   1 }, // 2nd entry
    { DEFIO_TAG_E__PB2,  ADC_DEVICES_2,   ADC_CHANNEL_12, 12 },
    { DEFIO_TAG_E__PB11, ADC_DEVICES_12,  ADC_CHANNEL_14, 14 },
    { DEFIO_TAG_E__PB12, ADC_DEVICES_1,   ADC_CHANNEL_11, 11 },
    { DEFIO_TAG_E__PB12, ADC_DEVICES_4,   ADC_CHANNEL_3,   3 }, // 2nd entry
    { DEFIO_TAG_E__PB13, ADC_DEVICES_5,   ADC_CHANNEL_3,   3 },
    { DEFIO_TAG_E__PB14, ADC_DEVICES_1,   ADC_CHANNEL_5,   5 },
    { DEFIO_TAG_E__PB14, ADC_DEVICES_4,   ADC_CHANNEL_4,   4 }, // 2nd entry
    { DEFIO_TAG_E__PB15, ADC_DEVICES_2,   ADC_CHANNEL_15, 15 },
    { DEFIO_TAG_E__PB15, ADC_DEVICES_4,   ADC_CHANNEL_5,   5 }, // 2nd entry

    { DEFIO_TAG_E__PC0,  ADC_DEVICES_12,  ADC_CHANNEL_6,   6 },
    { DEFIO_TAG_E__PC1,  ADC_DEVICES_12,  ADC_CHANNEL_7,   7 },
    { DEFIO_TAG_E__PC2,  ADC_DEVICES_12,  ADC_CHANNEL_8,   8 },
    { DEFIO_TAG_E__PC3,  ADC_DEVICES_12,  ADC_CHANNEL_9,   9 },
    { DEFIO_TAG_E__PC4,  ADC_DEVICES_2,   ADC_CHANNEL_5,   5 },
    { DEFIO_TAG_E__PC5,  ADC_DEVICES_2,   ADC_CHANNEL_11, 11 },

    { DEFIO_TAG_E__PD8,  ADC_DEVICES_2,   ADC_CHANNEL_15, 15 },
    { DEFIO_TAG_E__PD8,  ADC_DEVICES_4,   ADC_CHANNEL_12, 12 }, // 2nd entry
    { DEFIO_TAG_E__PD9,  ADC_DEVICES_4,   ADC_CHANNEL_12, 12 },
    { DEFIO_TAG_E__PD9,  ADC_DEVICES_5,   ADC_CHANNEL_13, 13 }, // 2nd entry
    { DEFIO_TAG_E__PD10, ADC_DEVICES_345, ADC_CHANNEL_7,   7 },
    { DEFIO_TAG_E__PD11, ADC_DEVICES_345, ADC_CHANNEL_8,   8 },
    { DEFIO_TAG_E__PD12, ADC_DEVICES_345, ADC_CHANNEL_9,   9 },
    { DEFIO_TAG_E__PD13, ADC_DEVICES_345, ADC_CHANNEL_10, 10 },
    { DEFIO_TAG_E__PD14, ADC_DEVICES_345, ADC_CHANNEL_11, 11 },

    { DEFIO_TAG_E__PE7,  ADC_DEVICES_3,   ADC_CHANNEL_4,   4 },
    { DEFIO_TAG_E__PE8,  ADC_DEVICES_345, ADC_CHANNEL_6,   6 },
    { DEFIO_TAG_E__PE9,  ADC_DEVICES_3,   ADC_CHANNEL_2,   2 },
    { DEFIO_TAG_E__PE10, ADC_DEVICES_345, ADC_CHANNEL_14, 14,},
    { DEFIO_TAG_E__PE11, ADC_DEVICES_345, ADC_CHANNEL_15, 15,},
    { DEFIO_TAG_E__PE12, ADC_DEVICES_345, ADC_CHANNEL_16, 16,},
    { DEFIO_TAG_E__PE13, ADC_DEVICES_3,   ADC_CHANNEL_3,   3,},
    { DEFIO_TAG_E__PE14, ADC_DEVICES_4,   ADC_CHANNEL_1,   1,},
    { DEFIO_TAG_E__PE15, ADC_DEVICES_4,   ADC_CHANNEL_2,   2,},

    // Inputs available for packages larger than 100-pin are not listed
};

// An array to convert rank number to encoded rank code for HAL.
// Note that the table only list possible values, as rank for any single conversion
// will not exceed maximum number of input sources (ADC_CHANNEL_COUNT).
static uint32_t adcRegularRank[] = {
    0,                     // ranks is counted by 1-origin; dodge zero.
    ADC_REGULAR_RANK_1,
    ADC_REGULAR_RANK_2,
    ADC_REGULAR_RANK_3,
    ADC_REGULAR_RANK_4,
#ifdef USE_ADC_INTERNAL
    ADC_REGULAR_RANK_5,
    ADC_REGULAR_RANK_6,
#endif
};

static void handleError(void)
{
    while (true) {
    }
}

// Note on sampling time.
// Temperature sensor has minimum sample time of 9us.
// With prescaler = 4 at 200MHz (AHB1), fADC = 50MHz (tcycle = 0.02us), 9us = 450cycles < 810

void adcInitDevice(adcDevice_t *adcdev, int channelCount)
{
    ADC_HandleTypeDef *hadc = &adcdev->ADCHandle; // For clarity

    hadc->Instance = adcdev->ADCx;

    // DeInit is done in adcInit().

    hadc->Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
    hadc->Init.Resolution = ADC_RESOLUTION_12B;
    hadc->Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc->Init.GainCompensation = 0;
    hadc->Init.ScanConvMode = ADC_SCAN_ENABLE;
    hadc->Init.EOCSelection = ADC_EOC_SEQ_CONV;
    hadc->Init.LowPowerAutoWait = DISABLE;
    hadc->Init.ContinuousConvMode = ENABLE;
    hadc->Init.NbrOfConversion = channelCount;
    hadc->Init.DiscontinuousConvMode = DISABLE;
    hadc->Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc->Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc->Init.DMAContinuousRequests = ENABLE;
    hadc->Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
    hadc->Init.OversamplingMode = DISABLE;

    if (HAL_ADC_Init(hadc) != HAL_OK) {
        handleError();
    }

    if (HAL_ADCEx_Calibration_Start(hadc, ADC_SINGLE_ENDED) != HAL_OK) {
      handleError();
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

void adcInitCalibrationValues(void)
{
    adcVREFINTCAL = *(uint16_t *)VREFINT_CAL_ADDR;
    adcTSCAL1 = *TEMPSENSOR_CAL1_ADDR;
    adcTSCAL2 = *TEMPSENSOR_CAL2_ADDR;
    adcTSSlopeK = (TEMPSENSOR_CAL2_TEMP - TEMPSENSOR_CAL1_TEMP) * 1000 / (adcTSCAL2 - adcTSCAL1);
}

// ADC conversion result DMA buffer
// Need this separate from the main adcValue[] array, because channels are numbered
// by ADC instance order that is different from ADC_xxx numbering.

volatile DMA_RAM_R uint16_t adcConversionBuffer[ADC_CHANNEL_COUNT] __attribute__((aligned(32)));

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
            dev = ADCDEV_1;
        } else if (i == ADC_VREFINT) {
            map = ADC_TAG_MAP_VREFINT;
            dev = ADCDEV_1;
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
        adcOperatingConfig[i].sampleTime = ADC_SAMPLETIME_640CYCLES_5;
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
            handleError();
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

        int configuredAdcChannels = BITCOUNT(adc->channelBits);

        adcInitDevice(adc, configuredAdcChannels);

        // Configure channels

        int rank = 1;

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
            sConfig.Rank         = adcRegularRank[rank++];      /* Rank of sampled channel number ADCx_CHANNEL */
            sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;  /* Sampling time (number of clock cycles unit) */
            sConfig.SingleDiff   = ADC_SINGLE_ENDED;            /* Single-ended input channel */
            sConfig.OffsetNumber = ADC_OFFSET_NONE;             /* No offset subtraction */
            sConfig.Offset = 0;                                 /* Parameter discarded because offset correction is disabled */

            if (HAL_ADC_ConfigChannel(&adc->ADCHandle, &sConfig) != HAL_OK) {
                handleError();
            }
        }

        // Configure DMA for this ADC peripheral

        dmaIdentifier_e dmaIdentifier;
#ifdef USE_DMA_SPEC
        const dmaChannelSpec_t *dmaSpec = dmaGetChannelSpecByPeripheral(DMA_PERIPH_ADC, dev, config->dmaopt[dev]);

        if (!dmaSpec) {
            return;
        }

        adc->DmaHandle.Instance                 = (DMA_ARCH_TYPE *)dmaSpec->ref;
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
            handleError();
        }

        dmaBufferIndex += BITCOUNT(adc->channelBits);
    }
}

void adcGetChannelValues(void)
{
    // Transfer values in conversion buffer into adcValues[]
    // Cache coherency should be maintained by MPU facility

    for (int i = 0; i < ADC_CHANNEL_INTERNAL_FIRST_ID; i++) {
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

int adcPrivateVref = -1;
int adcPrivateTemp = -1;

uint16_t adcInternalReadVrefint(void)
{
    uint16_t value = adcInternalRead(ADC_VREFINT);
adcPrivateVref = __HAL_ADC_CALC_VREFANALOG_VOLTAGE(value, ADC_RESOLUTION_12B);
    return value;
}

uint16_t adcInternalReadTempsensor(void)
{
    uint16_t value = adcInternalRead(ADC_TEMPSENSOR);
adcPrivateTemp = __HAL_ADC_CALC_TEMPERATURE(adcPrivateVref, value, ADC_RESOLUTION_12B);
    return value;
}
#endif // USE_ADC_INTERNAL
#endif // USE_ADC
