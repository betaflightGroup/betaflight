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

#include "platform.h"

/*
 * Compute SCLDEL, SDADEL, SCLH and SCLL for TIMINGR register according to reference manuals.
 */
static void i2cClockComputeRaw(uint32_t pclkFreq, int i2cFreqKhz, int presc, int dfcoeff,
                            uint8_t *scldel, uint8_t *sdadel, uint16_t *sclh, uint16_t *scll)
{
    // Values from I2C-SMBus specification
    uint16_t trmax;      // Rise time (max)
    uint16_t tfmax;      // Fall time (max)
    uint8_t tsuDATmin;   // SDA setup time (min)
    uint8_t thdDATmin;   // SDA hold time (min)
    uint16_t tHIGHmin;   // High period of SCL clock (min)
    uint16_t tLOWmin;    // Low period of SCL clock (min)

    // Silicon specific values, from datasheet
    uint8_t tAFmin = 50; // Analog filter delay (min)

    // Actual (estimated) values
    uint8_t tr = 100;   // Rise time
    uint8_t tf = 10;    // Fall time

    if (i2cFreqKhz > 400) {
        // Fm+ (Fast mode plus)
        trmax = 120;
        tfmax = 120;
        tsuDATmin = 50;
        thdDATmin = 0;
        tHIGHmin = 260;
        tLOWmin = 500;
    } else {
        // Fm (Fast mode)
        trmax = 300;
        tfmax = 300;
        tsuDATmin = 100;
        thdDATmin = 0;
        tHIGHmin = 600;
        tLOWmin = 1300;
    }

    // Convert pclkFreq into nsec
    float tI2cclk = 1000000000.0f / pclkFreq;

    // Convert target i2cFreq into cycle time (nsec)
    float tSCL = 1000000.0f / i2cFreqKhz;

    uint32_t SCLDELmin = (trmax + tsuDATmin) / ((presc + 1) * tI2cclk) - 1;
    uint32_t SDADELmin = (tfmax + thdDATmin - tAFmin - ((dfcoeff + 3) * tI2cclk)) / ((presc + 1) * tI2cclk);

    float tsync1 = tf + tAFmin + dfcoeff * tI2cclk + 2 * tI2cclk;
    float tsync2 = tr + tAFmin + dfcoeff * tI2cclk + 2 * tI2cclk;

    float tSCLH = tHIGHmin * tSCL / (tHIGHmin + tLOWmin) - tsync2;
    float tSCLL = tSCL - tSCLH - tsync1 - tsync2;

    uint32_t SCLH = tSCLH / ((presc + 1) * tI2cclk) - 1;
    uint32_t SCLL = tSCLL / ((presc + 1) * tI2cclk) - 1;

    while (tsync1 + tsync2 + ((SCLH + 1) + (SCLL + 1)) * ((presc + 1) * tI2cclk) < tSCL) {
        SCLH++;
    }

    *scldel = SCLDELmin;
    *sdadel = SDADELmin;
    *sclh = SCLH;
    *scll = SCLL;
}

uint32_t i2cClockTIMINGR(uint32_t pclkFreq, int i2cFreqKhz, int dfcoeff)
{
#define TIMINGR(presc, scldel, sdadel, sclh, scll) \
    ((presc << 28)|(scldel << 20)|(sdadel << 16)|(sclh << 8)|(scll << 0))

    uint8_t scldel;
    uint8_t sdadel;
    uint16_t sclh;
    uint16_t scll;

    for (int presc = 0; presc < 15; presc++) {
        i2cClockComputeRaw(pclkFreq, i2cFreqKhz, presc, dfcoeff, &scldel, &sdadel, &sclh, &scll);

        // If all fields are not overflowing, return TIMINGR.
        // Otherwise, increase prescaler and try again.
        if ((scldel < 16) && (sdadel < 16) && (sclh < 256) && (scll < 256)) {
            return TIMINGR(presc, scldel, sdadel, sclh, scll);
        }
    }
    return 0; // Shouldn't reach here
}
