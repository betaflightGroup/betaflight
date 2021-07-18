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

#include "platform.h"

#ifdef USE_ADC

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "drivers/adc.h"
#include "drivers/adc_impl.h"
#include "drivers/io.h"

#include "pg/adc.h"


PG_REGISTER_WITH_RESET_FN(adcConfig_t, adcConfig, PG_ADC_CONFIG, 0);

void pgResetFn_adcConfig(adcConfig_t *adcConfig)
{
    adcConfig->device = ADC_DEV_TO_CFG(adcDeviceByInstance(ADC_INSTANCE));
    adcConfig->dmaopt[ADCDEV_1] = ADC1_DMA_OPT;
// These conditionals need to match the ones used in 'src/main/drivers/adc.h'.
#if defined(ADC2)
    adcConfig->dmaopt[ADCDEV_2] = ADC2_DMA_OPT;
#endif
#if defined(ADC3)
    adcConfig->dmaopt[ADCDEV_3] = ADC3_DMA_OPT;
#endif
#if defined(ADC4)
    adcConfig->dmaopt[ADCDEV_4] = ADC4_DMA_OPT;
#endif
#if defined(ADC5)
    adcConfig->dmaopt[ADCDEV_5] = ADC5_DMA_OPT;
#endif

#ifdef VBAT_ADC_PIN
    adcConfig->vbat.enabled = true;
    adcConfig->vbat.ioTag = IO_TAG(VBAT_ADC_PIN);
#if defined(STM32H7)
#ifdef VBAT_ADC_INSTANCE
    adcConfig->vbat.device = ADC_DEV_TO_CFG(adcDeviceByInstance(VBAT_ADC_INSTANCE));;
#else
    adcConfig->vbat.device = adcConfig->device;
#endif
#endif
#endif

#ifdef EXTERNAL1_ADC_PIN
    adcConfig->external1.enabled = true;
    adcConfig->external1.ioTag = IO_TAG(EXTERNAL1_ADC_PIN);
#if defined(STM32H7)
#ifdef EXTERNAL1_ADC_INSTANCE
    adcConfig->external.device = ADC_DEV_TO_CFG(adcDeviceByInstance(EXTERNAL1_ADC_INSTANCE));;
#else
    adcConfig->external1.device = adcConfig->device;
#endif
#endif
#endif

#ifdef CURRENT_METER_ADC_PIN
    adcConfig->current.enabled = true;
    adcConfig->current.ioTag = IO_TAG(CURRENT_METER_ADC_PIN);
#if defined(STM32H7)
#ifdef CURRENT_METER_ADC_INSTANCE
    adcConfig->current.device = ADC_DEV_TO_CFG(adcDeviceByInstance(CURRENT_METER_ADC_INSTANCE));;
#else
    adcConfig->current.device = adcConfig->device;
#endif
#endif
#endif

#ifdef RSSI_ADC_PIN
    adcConfig->rssi.enabled = true;
    adcConfig->rssi.ioTag = IO_TAG(RSSI_ADC_PIN);
#if defined(STM32H7)
#ifdef RSSI_ADC_INSTANCE
    adcConfig->rssi.device = ADC_DEV_TO_CFG(adcDeviceByInstance(RSSI_ADC_INSTANCE));;
#else
    adcConfig->rssi.device = adcConfig->device;
#endif
#endif
#endif

    adcConfig->vrefIntCalibration = 0;
    adcConfig->tempSensorCalibration1 = 0;
    adcConfig->tempSensorCalibration2 = 0;
}
#endif // USE_ADC
