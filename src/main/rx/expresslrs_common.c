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

/*
 * Based on https://github.com/ExpressLRS/ExpressLRS
 * Thanks to AlessandroAU, original creator of the ExpressLRS project.
 */

#include <string.h>
#include "platform.h"

#ifdef USE_RX_EXPRESSLRS

#include "build/build_config.h"
#include "common/utils.h"
#include "rx/expresslrs_common.h"
#include "drivers/rx/rx_sx127x.h"
#include "drivers/rx/rx_sx1280.h"

STATIC_UNIT_TESTED uint16_t crc14tab[ELRS_CRC_LEN] = {0};

static uint8_t volatile FHSSptr = 0;
STATIC_UNIT_TESTED uint8_t FHSSsequence[ELRS_NR_SEQUENCE_ENTRIES] = {0};
static const uint32_t *FHSSfreqs;
static uint8_t numEntries = 0; // The number of FHSS frequencies in the table

elrs_mod_settings_t air_rate_config[][ELRS_RATE_MAX] = {
#ifdef USE_RX_SX127X
    {
        {0, RATE_200HZ, SX127x_BW_500_00_KHZ, SX127x_SF_6, SX127x_CR_4_7, 5000, TLM_RATIO_1_64, 4, 8, -112},
        {1, RATE_100HZ, SX127x_BW_500_00_KHZ, SX127x_SF_7, SX127x_CR_4_7, 10000, TLM_RATIO_1_64, 4, 8, -117},
        {2, RATE_50HZ, SX127x_BW_500_00_KHZ, SX127x_SF_8, SX127x_CR_4_7, 20000, TLM_RATIO_NO_TLM, 4, 10, -120},
        {3, RATE_25HZ, SX127x_BW_500_00_KHZ, SX127x_SF_9, SX127x_CR_4_7, 40000, TLM_RATIO_NO_TLM, 4, 10, -123}
    },
#endif
#ifdef USE_RX_SX1280
    {
        {0, RATE_500HZ, SX1280_LORA_BW_0800, SX1280_LORA_SF5, SX1280_LORA_CR_LI_4_6, 2000, TLM_RATIO_1_128, 4, 12, -105},
        {1, RATE_250HZ, SX1280_LORA_BW_0800, SX1280_LORA_SF6, SX1280_LORA_CR_LI_4_7, 4000, TLM_RATIO_1_64, 4, 14, -108},
        {2, RATE_150HZ, SX1280_LORA_BW_0800, SX1280_LORA_SF7, SX1280_LORA_CR_LI_4_7, 6666, TLM_RATIO_1_32, 4, 12, -112},
        {3, RATE_50HZ, SX1280_LORA_BW_0800, SX1280_LORA_SF9, SX1280_LORA_CR_LI_4_6, 20000, TLM_RATIO_NO_TLM, 4, 12, -117}
    },
#endif
#if !defined(USE_RX_SX127X) && !defined(USE_RX_SX1280)
    {{0}},
#endif
};

#ifdef USE_RX_SX127X
const uint32_t FHSSfreqsAU433[] = {
    FREQ_HZ_TO_REG_VAL_900(433420000),
    FREQ_HZ_TO_REG_VAL_900(433920000),
    FREQ_HZ_TO_REG_VAL_900(434420000)};

const uint32_t FHSSfreqsAU915[] = {
    FREQ_HZ_TO_REG_VAL_900(915500000),
    FREQ_HZ_TO_REG_VAL_900(916100000),
    FREQ_HZ_TO_REG_VAL_900(916700000),
    FREQ_HZ_TO_REG_VAL_900(917300000),

    FREQ_HZ_TO_REG_VAL_900(917900000),
    FREQ_HZ_TO_REG_VAL_900(918500000),
    FREQ_HZ_TO_REG_VAL_900(919100000),
    FREQ_HZ_TO_REG_VAL_900(919700000),

    FREQ_HZ_TO_REG_VAL_900(920300000),
    FREQ_HZ_TO_REG_VAL_900(920900000),
    FREQ_HZ_TO_REG_VAL_900(921500000),
    FREQ_HZ_TO_REG_VAL_900(922100000),

    FREQ_HZ_TO_REG_VAL_900(922700000),
    FREQ_HZ_TO_REG_VAL_900(923300000),
    FREQ_HZ_TO_REG_VAL_900(923900000),
    FREQ_HZ_TO_REG_VAL_900(924500000),

    FREQ_HZ_TO_REG_VAL_900(925100000),
    FREQ_HZ_TO_REG_VAL_900(925700000),
    FREQ_HZ_TO_REG_VAL_900(926300000),
    FREQ_HZ_TO_REG_VAL_900(926900000)};

/* Frequency bands taken from https://wetten.overheid.nl/BWBR0036378/2016-12-28#Bijlagen
 * Note: these frequencies fall in the license free H-band, but in combination with 500kHz 
 * LoRa modem bandwidth used by ExpressLRS (EU allows up to 125kHz modulation BW only) they
 * will never pass RED certification and they are ILLEGAL to use. 
 * 
 * Therefore we simply maximize the usage of available spectrum so laboratory testing of the software won't disturb existing
 * 868MHz ISM band traffic too much.
 */
const uint32_t FHSSfreqsEU868[] = {
    FREQ_HZ_TO_REG_VAL_900(863275000), // band H1, 863 - 865MHz, 0.1% duty cycle or CSMA techniques, 25mW EIRP
    FREQ_HZ_TO_REG_VAL_900(863800000),
    FREQ_HZ_TO_REG_VAL_900(864325000),
    FREQ_HZ_TO_REG_VAL_900(864850000),
    FREQ_HZ_TO_REG_VAL_900(865375000), // Band H2, 865 - 868.6MHz, 1.0% dutycycle or CSMA, 25mW EIRP
    FREQ_HZ_TO_REG_VAL_900(865900000),
    FREQ_HZ_TO_REG_VAL_900(866425000),
    FREQ_HZ_TO_REG_VAL_900(866950000),
    FREQ_HZ_TO_REG_VAL_900(867475000),
    FREQ_HZ_TO_REG_VAL_900(868000000),
    FREQ_HZ_TO_REG_VAL_900(868525000), // Band H3, 868.7-869.2MHz, 0.1% dutycycle or CSMA, 25mW EIRP
    FREQ_HZ_TO_REG_VAL_900(869050000),
    FREQ_HZ_TO_REG_VAL_900(869575000)};

/* Frequency band G, taken from https://wetten.overheid.nl/BWBR0036378/2016-12-28#Bijlagen
 * Note: As is the case with the 868Mhz band, these frequencies only comply to the license free portion
 * of the spectrum, nothing else. As such, these are likely illegal to use. 
 */
const uint32_t FHSSfreqsEU433[] = {
    FREQ_HZ_TO_REG_VAL_900(433100000),
    FREQ_HZ_TO_REG_VAL_900(433925000),
    FREQ_HZ_TO_REG_VAL_900(434450000)};

/* Very definitely not fully checked. An initial pass at increasing the hops
*/
const uint32_t FHSSfreqsFCC915[] = {
    FREQ_HZ_TO_REG_VAL_900(903500000),
    FREQ_HZ_TO_REG_VAL_900(904100000),
    FREQ_HZ_TO_REG_VAL_900(904700000),
    FREQ_HZ_TO_REG_VAL_900(905300000),

    FREQ_HZ_TO_REG_VAL_900(905900000),
    FREQ_HZ_TO_REG_VAL_900(906500000),
    FREQ_HZ_TO_REG_VAL_900(907100000),
    FREQ_HZ_TO_REG_VAL_900(907700000),

    FREQ_HZ_TO_REG_VAL_900(908300000),
    FREQ_HZ_TO_REG_VAL_900(908900000),
    FREQ_HZ_TO_REG_VAL_900(909500000),
    FREQ_HZ_TO_REG_VAL_900(910100000),

    FREQ_HZ_TO_REG_VAL_900(910700000),
    FREQ_HZ_TO_REG_VAL_900(911300000),
    FREQ_HZ_TO_REG_VAL_900(911900000),
    FREQ_HZ_TO_REG_VAL_900(912500000),

    FREQ_HZ_TO_REG_VAL_900(913100000),
    FREQ_HZ_TO_REG_VAL_900(913700000),
    FREQ_HZ_TO_REG_VAL_900(914300000),
    FREQ_HZ_TO_REG_VAL_900(914900000),

    FREQ_HZ_TO_REG_VAL_900(915500000), // as per AU..
    FREQ_HZ_TO_REG_VAL_900(916100000),
    FREQ_HZ_TO_REG_VAL_900(916700000),
    FREQ_HZ_TO_REG_VAL_900(917300000),

    FREQ_HZ_TO_REG_VAL_900(917900000),
    FREQ_HZ_TO_REG_VAL_900(918500000),
    FREQ_HZ_TO_REG_VAL_900(919100000),
    FREQ_HZ_TO_REG_VAL_900(919700000),

    FREQ_HZ_TO_REG_VAL_900(920300000),
    FREQ_HZ_TO_REG_VAL_900(920900000),
    FREQ_HZ_TO_REG_VAL_900(921500000),
    FREQ_HZ_TO_REG_VAL_900(922100000),

    FREQ_HZ_TO_REG_VAL_900(922700000),
    FREQ_HZ_TO_REG_VAL_900(923300000),
    FREQ_HZ_TO_REG_VAL_900(923900000),
    FREQ_HZ_TO_REG_VAL_900(924500000),

    FREQ_HZ_TO_REG_VAL_900(925100000),
    FREQ_HZ_TO_REG_VAL_900(925700000),
    FREQ_HZ_TO_REG_VAL_900(926300000),
    FREQ_HZ_TO_REG_VAL_900(926900000)};
#endif
#ifdef USE_RX_SX1280
const uint32_t FHSSfreqsISM2400[] = {
    FREQ_HZ_TO_REG_VAL_24(2400400000),
    FREQ_HZ_TO_REG_VAL_24(2401400000),
    FREQ_HZ_TO_REG_VAL_24(2402400000),
    FREQ_HZ_TO_REG_VAL_24(2403400000),
    FREQ_HZ_TO_REG_VAL_24(2404400000),

    FREQ_HZ_TO_REG_VAL_24(2405400000),
    FREQ_HZ_TO_REG_VAL_24(2406400000),
    FREQ_HZ_TO_REG_VAL_24(2407400000),
    FREQ_HZ_TO_REG_VAL_24(2408400000),
    FREQ_HZ_TO_REG_VAL_24(2409400000),

    FREQ_HZ_TO_REG_VAL_24(2410400000),
    FREQ_HZ_TO_REG_VAL_24(2411400000),
    FREQ_HZ_TO_REG_VAL_24(2412400000),
    FREQ_HZ_TO_REG_VAL_24(2413400000),
    FREQ_HZ_TO_REG_VAL_24(2414400000),

    FREQ_HZ_TO_REG_VAL_24(2415400000),
    FREQ_HZ_TO_REG_VAL_24(2416400000),
    FREQ_HZ_TO_REG_VAL_24(2417400000),
    FREQ_HZ_TO_REG_VAL_24(2418400000),
    FREQ_HZ_TO_REG_VAL_24(2419400000),

    FREQ_HZ_TO_REG_VAL_24(2420400000),
    FREQ_HZ_TO_REG_VAL_24(2421400000),
    FREQ_HZ_TO_REG_VAL_24(2422400000),
    FREQ_HZ_TO_REG_VAL_24(2423400000),
    FREQ_HZ_TO_REG_VAL_24(2424400000),

    FREQ_HZ_TO_REG_VAL_24(2425400000),
    FREQ_HZ_TO_REG_VAL_24(2426400000),
    FREQ_HZ_TO_REG_VAL_24(2427400000),
    FREQ_HZ_TO_REG_VAL_24(2428400000),
    FREQ_HZ_TO_REG_VAL_24(2429400000),

    FREQ_HZ_TO_REG_VAL_24(2430400000),
    FREQ_HZ_TO_REG_VAL_24(2431400000),
    FREQ_HZ_TO_REG_VAL_24(2432400000),
    FREQ_HZ_TO_REG_VAL_24(2433400000),
    FREQ_HZ_TO_REG_VAL_24(2434400000),

    FREQ_HZ_TO_REG_VAL_24(2435400000),
    FREQ_HZ_TO_REG_VAL_24(2436400000),
    FREQ_HZ_TO_REG_VAL_24(2437400000),
    FREQ_HZ_TO_REG_VAL_24(2438400000),
    FREQ_HZ_TO_REG_VAL_24(2439400000),

    FREQ_HZ_TO_REG_VAL_24(2440400000),
    FREQ_HZ_TO_REG_VAL_24(2441400000),
    FREQ_HZ_TO_REG_VAL_24(2442400000),
    FREQ_HZ_TO_REG_VAL_24(2443400000),
    FREQ_HZ_TO_REG_VAL_24(2444400000),

    FREQ_HZ_TO_REG_VAL_24(2445400000),
    FREQ_HZ_TO_REG_VAL_24(2446400000),
    FREQ_HZ_TO_REG_VAL_24(2447400000),
    FREQ_HZ_TO_REG_VAL_24(2448400000),
    FREQ_HZ_TO_REG_VAL_24(2449400000),

    FREQ_HZ_TO_REG_VAL_24(2450400000),
    FREQ_HZ_TO_REG_VAL_24(2451400000),
    FREQ_HZ_TO_REG_VAL_24(2452400000),
    FREQ_HZ_TO_REG_VAL_24(2453400000),
    FREQ_HZ_TO_REG_VAL_24(2454400000),

    FREQ_HZ_TO_REG_VAL_24(2455400000),
    FREQ_HZ_TO_REG_VAL_24(2456400000),
    FREQ_HZ_TO_REG_VAL_24(2457400000),
    FREQ_HZ_TO_REG_VAL_24(2458400000),
    FREQ_HZ_TO_REG_VAL_24(2459400000),

    FREQ_HZ_TO_REG_VAL_24(2460400000),
    FREQ_HZ_TO_REG_VAL_24(2461400000),
    FREQ_HZ_TO_REG_VAL_24(2462400000),
    FREQ_HZ_TO_REG_VAL_24(2463400000),
    FREQ_HZ_TO_REG_VAL_24(2464400000),

    FREQ_HZ_TO_REG_VAL_24(2465400000),
    FREQ_HZ_TO_REG_VAL_24(2466400000),
    FREQ_HZ_TO_REG_VAL_24(2467400000),
    FREQ_HZ_TO_REG_VAL_24(2468400000),
    FREQ_HZ_TO_REG_VAL_24(2469400000),

    FREQ_HZ_TO_REG_VAL_24(2470400000),
    FREQ_HZ_TO_REG_VAL_24(2471400000),
    FREQ_HZ_TO_REG_VAL_24(2472400000),
    FREQ_HZ_TO_REG_VAL_24(2473400000),
    FREQ_HZ_TO_REG_VAL_24(2474400000),

    FREQ_HZ_TO_REG_VAL_24(2475400000),
    FREQ_HZ_TO_REG_VAL_24(2476400000),
    FREQ_HZ_TO_REG_VAL_24(2477400000),
    FREQ_HZ_TO_REG_VAL_24(2478400000),
    FREQ_HZ_TO_REG_VAL_24(2479400000)};
#endif

void generateCrc14Table(void)
{
    uint16_t crc;
    for (uint16_t i = 0; i < ELRS_CRC_LEN; i++) {
        crc = i << (14 - 8);
        for (uint8_t j = 0; j < 8; j++) {
            crc = (crc << 1) ^ ((crc & 0x2000) ? ELRS_CRC14_POLY : 0);
        }
        crc14tab[i] = crc;
    }
}

uint16_t calcCrc14(uint8_t *data, uint8_t len, uint16_t crc)
{
    while (len--) {
        crc = (crc << 8) ^ crc14tab[((crc >> 6) ^ (uint16_t) *data++) & 0x00FF];
    }
    return crc & 0x3FFF;
}

static void initializeFHSSFrequencies(const elrs_freq_domain_e dom) {
    switch (dom) {
#ifdef USE_RX_SX127X
        case AU433:
            FHSSfreqs = FHSSfreqsAU433;
            numEntries = sizeof(FHSSfreqsAU433) / sizeof(uint32_t);
            break;
        case AU915:
            FHSSfreqs = FHSSfreqsAU915;
            numEntries = sizeof(FHSSfreqsAU915) / sizeof(uint32_t);
            break;
        case EU433:
            FHSSfreqs = FHSSfreqsEU433;
            numEntries = sizeof(FHSSfreqsEU433) / sizeof(uint32_t);
            break;
        case EU868:
            FHSSfreqs = FHSSfreqsEU868;
            numEntries = sizeof(FHSSfreqsEU868) / sizeof(uint32_t);
            break;
        case FCC915:
            FHSSfreqs = FHSSfreqsFCC915;
            numEntries = sizeof(FHSSfreqsFCC915) / sizeof(uint32_t);
            break;
#endif
#ifdef USE_RX_SX1280
        case ISM2400:
            FHSSfreqs = FHSSfreqsISM2400;
            numEntries = sizeof(FHSSfreqsISM2400) / sizeof(uint32_t);
            break;
#endif
        default:
            FHSSfreqs = NULL;
            numEntries = 0;
    }
}

uint32_t getInitialFreq(const int32_t freqCorrection)
{
    return FHSSfreqs[0] - freqCorrection;
}

uint8_t getFHSSNumEntries(void)
{
    return numEntries;
}

uint8_t FHSSgetCurrIndex(void) {
    return FHSSptr;
}

void FHSSsetCurrIndex(const uint8_t value) {
    FHSSptr = value;
}

uint32_t FHSSgetNextFreq(const int32_t freqCorrection)
{
    return FHSSfreqs[FHSSsequence[FHSSptr++]] - freqCorrection;
}

static unsigned long seed = 0;

// returns 0 <= x < max where max <= 256
// (actual upper limit is higher, but there is one and I haven't
//  thought carefully about what it is)
static unsigned int rngN(unsigned int max)
{
    unsigned long m = 2147483648;
    long a = 214013;
    long c = 2531011;
    seed = (a * seed + c) % m;
    unsigned int result = ((seed >> 16) * max) / ELRS_RNG_MAX;
    return result;
}

// Set all of the flags in the array to true, except for the first one
// which corresponds to the sync channel and is never available for normal
// allocation.
static void resetIsAvailable(uint8_t *array, const uint8_t size)
{
    // channel 0 is the sync channel and is never considered available
    array[0] = 0;

    // all other entires to 1
    for (unsigned int i = 1; i < size; i++) {
        array[i] = 1;
    }
}

/**
Requirements:
1. 0 every n hops
2. No two repeated channels
3. Equal occurance of each (or as even as possible) of each channel
4. Pesudorandom

Approach:
  Initialise an array of flags indicating which channels have not yet been assigned and a counter of how many channels are available
  Iterate over the FHSSsequence array using index
    if index is a multiple of numEntries assign the sync channel index (0)
    otherwise, generate a random number between 0 and the number of channels left to be assigned
    find the index of the nth remaining channel
    if the index is a repeat, generate a new random number
    if the index is not a repeat, assing it to the FHSSsequence array, clear the availability flag and decrement the available count
    if there are no available channels left, reset the flags array and the count
*/
void FHSSrandomiseFHSSsequence(const uint8_t UID[], const elrs_freq_domain_e dom)
{
    seed = ((long)UID[2] << 24) + ((long)UID[3] << 16) + ((long)UID[4] << 8) + UID[5];

    initializeFHSSFrequencies(dom);

    uint8_t isAvailable[numEntries];

    resetIsAvailable(isAvailable, numEntries);

    // Fill the FHSSsequence with channel indices
    // The 0 index is special - the 'sync' channel. The sync channel appears every
    // syncInterval hops. The other channels are randomly distributed between the
    // sync channels
    int nLeft = numEntries - 1; // how many channels are left to be allocated. Does not include the sync channel
    unsigned int prev = 0;           // needed to prevent repeats of the same index

    // for each slot in the sequence table
    for (int i = 0; i < ELRS_NR_SEQUENCE_ENTRIES; i++)
    {
        if (i % numEntries == 0)
        {
            // assign sync channel 0
            FHSSsequence[i] = 0;
            prev = 0;
        }
        else
        {
            // pick one of the available channels. May need to loop to avoid repeats
            unsigned int index;
            do
            {
                int c = rngN(nLeft); // returnc 0<c<nLeft
                // find the c'th entry in the isAvailable array
                // can skip 0 as that's the sync channel and is never available for normal allocation
                index = 1;
                int found = 0;
                while (index < numEntries)
                {
                    if (isAvailable[index])
                    {
                        if (found == c)
                            break;
                        found++;
                    }
                    index++;
                }
                if (index == numEntries)
                {
                    // This should never happen
                    // Use the sync channel
                    index = 0;
                    break;
                }
            } while (index == prev); // can't use index if it repeats the previous value

            FHSSsequence[i] = index; // assign the value to the sequence array
            isAvailable[index] = 0;  // clear the flag
            prev = index;            // remember for next iteration
            nLeft--;                 // reduce the count of available channels
            if (nLeft == 0)
            {
                // we've assigned all of the channels, so reset for next cycle
                resetIsAvailable(isAvailable, numEntries);
                nLeft = numEntries - 1;
            }
        }
    } // for each element in FHSSsequence
}

uint8_t tlmRatioEnumToValue(const elrs_tlm_ratio_e enumval)
{
    switch (enumval)
    {
    case TLM_RATIO_NO_TLM:
        return 1;
        break;
    case TLM_RATIO_1_2:
        return 2;
        break;
    case TLM_RATIO_1_4:
        return 4;
        break;
    case TLM_RATIO_1_8:
        return 8;
        break;
    case TLM_RATIO_1_16:
        return 16;
        break;
    case TLM_RATIO_1_32:
        return 32;
        break;
    case TLM_RATIO_1_64:
        return 64;
        break;
    case TLM_RATIO_1_128:
        return 128;
        break;
    default:
        return 0;
    }
}

#define ELRS_LQ_DEPTH 4 //100 % 32
static uint32_t lqArray[ELRS_LQ_DEPTH] = {0};
static uint8_t lq = 0;
static uint8_t lqByte = 0;
static uint32_t lqMask = 0;

uint8_t getLQ(const bool addLQ)
{
    lqMask <<= 1;
    if (lqMask == 0) {
        lqMask = (1 << 0);
        lqByte += 1;
    }

    // At idx N / 32 and bit N % 32, wrap back to idx=0, bit=0
    if ((lqByte == 3) && (lqMask & (1 << ELRS_LQ_DEPTH))) {
        lqByte = 0;
        lqMask = (1 << 0);
    }

    if ((lqArray[lqByte] & lqMask) != 0) {
        lqArray[lqByte] &= ~lqMask;
        lq -= 1;
    }

    if (addLQ) {
        lqArray[lqByte] |= lqMask;
        lq += 1;
    }

    return lq;
}

void resetLQ(void)
{
    lq = 0;
    lqByte = 0;
    lqMask = 0;
    memset(lqArray, 0, sizeof(lqArray));
}

inline uint16_t convertSwitch1b(const uint16_t val)
{
    return val ? 2000 : 1000;
}

// 3b to decode 7 pos switches
inline uint16_t convertSwitch3b(const uint16_t val) 
{
    switch (val) {
        case 0:
            return 1000;
        case 1:
            return 1275;
        case 2:
            return 1425;
        case 3:
            return 1575;
        case 4:
            return 1725;
        case 5:
            return 2000;
        default:
            return 1500;
    }
}

// 4b to decode 16 pos switches
inline uint16_t convertSwitch4b(const uint16_t val) 
{
    return (val > 15) ? 1500 : val * 66.67f + 1000;
}

inline uint16_t convertAnalog(const uint16_t val)
{
    return 0.62477120195241f * val + 881;
}

#endif /* USE_RX_EXPRESSLRS */