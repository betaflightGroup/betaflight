/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * Based on https://github.com/ExpressLRS/ExpressLRS
 * Thanks to AlessandroAU, original creator of the ExpressLRS project.
 */

#include <stdint.h>
#include <stdbool.h>

#include <limits.h>

extern "C" {
    #include "platform.h"

    #include "drivers/io.h"

    #include "pg/pg.h"
    #include "pg/pg_ids.h"
    #include "pg/rx_spi.h"
    #include "pg/rx_spi_expresslrs.h"

    #include "rx/rx_spi.h"
    #include "rx/expresslrs.h"

    #include "drivers/rx/rx_sx127x.h"
    #include "drivers/rx/rx_sx1280.h"

    extern uint8_t FHSSsequence[ELRS_NR_SEQUENCE_ENTRIES];
    extern uint16_t crc14tab[ELRS_CRC_LEN];

    extern elrsReceiver_t receiver;
    static const elrsReceiver_t empty = elrsReceiver_t();

    static rxRuntimeState_t config = rxRuntimeState_t();
    static rxSpiExtiConfig_t extiConfig;
    static const rxSpiConfig_t injectedConfig = {
        .extiIoTag = IO_TAG(PA0),
    };
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

//make clean test_rx_spi_expresslrs_unittest
TEST(RxSpiExpressLrsUnitTest, TestCrc14)
{
    uint16_t expectedCrc14Tab[ELRS_CRC_LEN] = {
        0,28247,62201,40110,52133,42482,14684,22283,
        38730,63773,26035,3044,23791,12984,44566,49217,
        11924,16579,56429,45626,58673,35686,6088,31135,
        47582,55177,19239,9584,29307,7212,32898,61141,
        29567,7464,33158,61393,47322,54925,18979,9332,
        58421,35426,5836,30875,12176,16839,56681,45886,
        24043,13244,44818,49477,38478,63513,25783,2784,
        51873,42230,14424,22031,260,28499,62461,40362,
        51369,42750,14928,21511,780,27995,61941,40866,
        24547,12724,44314,49997,37958,64017,26303,2280,
        58941,34922,5316,31379,11672,17359,57185,45366,
        29047,7968,33678,60889,47826,54405,18475,9852,
        48086,54657,18735,10104,28787,7716,33418,60637,
        11420,17099,56933,45106,59193,35182,5568,31639,
        38210,64277,26555,2540,24295,12464,44062,49737,
        520,27743,61681,40614,51629,43002,15188,21763,
        37202,65285,25515,3580,23287,13472,43022,50777,
        1560,26703,62689,39606,52669,41962,16196,20755,
        49094,53649,19775,9064,29795,6708,34458,59597,
        10380,18139,55925,46114,58153,36222,4560,32647,
        57901,35962,4308,32387,10632,18399,56177,46374,
        30055,6960,34718,59849,48834,53397,19515,8812,
        52409,41710,15936,20503,1820,26955,62949,39858,
        23539,13732,43274,51037,36950,65025,25263,3320,
        23035,14252,43778,50517,37470,64521,24743,3824,
        52913,41190,15432,21023,1300,27459,63469,39354,
        30575,6456,34198,60353,48330,53917,20019,8292,
        57381,36466,4828,31883,11136,17879,55673,46894,
        10884,17619,55421,46634,57633,36726,5080,32143,
        48590,54169,20279,8544,30315,6204,33938,60101,
        1040,27207,63209,39102,53173,41442,15692,21275,
        37722,64781,24995,4084,22783,13992,43526,50257
    };

    generateCrc14Table();
    for(int i=0; i < ELRS_CRC_LEN; i++) {
        EXPECT_EQ(expectedCrc14Tab[i], crc14tab[i]);
    }
}

TEST(RxSpiExpressLrsUnitTest, TestFHSSTable)
{
    const uint8_t UID[6] = {1, 2, 3, 4, 5, 6};

    const uint8_t expectedSequence[2][ELRS_NR_SEQUENCE_ENTRIES] = {
        {
            0,15,42,38,69,77,30,60,53,57,
            5,19,48,75,65,27,13,10,33,45,
            35,40,51,63,9,52,61,11,23,59,
            71,29,39,76,78,24,20,54,3,74,
            25,31,17,8,62,37,47,7,66,6,
            32,26,67,46,70,28,22,43,56,58,
            34,14,73,36,79,44,49,50,16,4,
            72,55,21,41,18,12,64,1,2,68,
            0,30,14,37,69,45,25,22,67,56,
            34,3,23,15,26,1,77,55,48,70,
            20,5,4,58,44,2,68,41,54,65,
            49,73,63,16,51,66,64,76,27,29,
            74,31,7,17,59,72,21,75,43,71,
            11,50,46,60,53,42,18,33,61,24,
            13,12,8,10,78,47,35,9,52,62,
            28,39,38,57,32,6,79,36,40,19,
            0,21,25,71,31,16,77,10,33,78,
            68,4,30,35,59,47,11,65,44,69,
            26,52,66,46,17,72,36,28,56,38,
            32,13,29,7,41,1,53,5,23,55,
            2,12,37,42,73,3,60,8,6,43,
            48,9,70,63,18,20,54,58,79,76,
            74,49,14,22,27,57,34,24,51,19,
            45,15,67,40,39,62,64,75,50,61,
            0,67,60,28,22,40,68,34,44,59,
            32,24,3,57,30,14
        },
        {
            0,8,21,19,35,39,15,29,26,28,
            3,10,24,37,32,13,6,5,17,22,
            16,20,27,33,4,25,31,2,11,34,
            36,12,18,38,30,7,9,23,1,14,
            0,13,17,9,6,31,21,25,5,33,
            4,16,15,34,24,36,14,18,22,29,
            30,12,8,39,20,38,23,26,28,10,
            2,37,27,11,19,32,3,35,1,7,
            0,38,15,7,19,34,22,12,10,33,
            27,17,2,11,6,14,1,39,28,25,
            35,13,3,4,30,23,5,32,21,29,
            36,24,37,26,9,20,31,18,16,8,
            0,15,36,16,1,9,31,35,12,37,
            24,34,5,26,25,30,29,23,8,17,
            32,13,7,10,3,4,38,27,18,6,
            22,33,11,20,21,28,14,2,39,19,
            0,30,19,10,12,36,15,8,38,4,
            17,39,33,2,16,20,28,24,5,32,
            22,34,11,26,31,21,7,35,14,13,
            27,18,9,3,23,1,25,6,37,29,
            0,12,26,1,5,16,19,35,2,29,
            4,3,20,22,6,36,32,9,10,25,
            28,39,38,37,21,7,13,14,27,17,
            15,30,8,23,11,33,24,18,31,34,
            0,32,6,24,34,29,14,11,20,35,
            17,21,30,15,10,2
        }
    };

    FHSSrandomiseFHSSsequence(UID, ISM2400);
    for(int i=0; i < ELRS_NR_SEQUENCE_ENTRIES; i++) {
        EXPECT_EQ(expectedSequence[0][i], FHSSsequence[i]);
    }

    FHSSrandomiseFHSSsequence(UID, FCC915);
    for(int i=0; i < ELRS_NR_SEQUENCE_ENTRIES; i++) {
        EXPECT_EQ(expectedSequence[1][i], FHSSsequence[i]);
    }
}

TEST(RxSpiExpressLrsUnitTest, TestInitUnbound)
{
    const uint8_t bindUID[6] = {0, 1, 2, 3, 4, 5};

    receiver = empty;
    expressLrsSpiInit(&injectedConfig, &config, &extiConfig);

    //check initialization of elrsReceiver_t
    EXPECT_FALSE(receiver.bound);
    EXPECT_EQ(IO_NONE, receiver.resetPin);
    EXPECT_EQ(IO_NONE, receiver.busyPin);
    for (int i = 0; i < 6; i++) {
        EXPECT_EQ(bindUID[i], receiver.UID[i]);
    }
    EXPECT_EQ(0, receiver.nonceRX);
    EXPECT_EQ(0, receiver.freqOffset);
    EXPECT_EQ(0, receiver.lastValidPacket);

    const uint32_t initialFrequencies[6] = {
        FREQ_HZ_TO_REG_VAL_900(433420000), 
        FREQ_HZ_TO_REG_VAL_900(915500000), 
        FREQ_HZ_TO_REG_VAL_900(433100000), 
        FREQ_HZ_TO_REG_VAL_900(863275000), 
        FREQ_HZ_TO_REG_VAL_900(903500000), 
        FREQ_HZ_TO_REG_VAL_24(2400400000)
    };

    for (int i=0; i < 6; i++) {
        receiver = empty;
        rxExpressLrsSpiConfigMutable()->domain = i;
        expressLrsSpiInit(&injectedConfig, &config, &extiConfig);
        EXPECT_EQ(initialFrequencies[i], receiver.currentFreq);
    }

    // for unbound we need to initialize in rate index 0
    receiver = empty;
    rxExpressLrsSpiConfigMutable()->rateIndex = 2;
    rxExpressLrsSpiConfigMutable()->domain = FCC915;
    expressLrsSpiInit(&injectedConfig, &config, &extiConfig);
    EXPECT_EQ(air_rate_config[0][0].index, receiver.mod_params->index);
    EXPECT_EQ(air_rate_config[0][0].enumRate, receiver.mod_params->enumRate);
    EXPECT_EQ(air_rate_config[0][0].bw, receiver.mod_params->bw);
    EXPECT_EQ(air_rate_config[0][0].sf, receiver.mod_params->sf);
    EXPECT_EQ(air_rate_config[0][0].cr, receiver.mod_params->cr);
    EXPECT_EQ(air_rate_config[0][0].interval, receiver.mod_params->interval);
    EXPECT_EQ(air_rate_config[0][0].tlmInterval, receiver.mod_params->tlmInterval);
    EXPECT_EQ(air_rate_config[0][0].fhssHopInterval, receiver.mod_params->fhssHopInterval);
    EXPECT_EQ(air_rate_config[0][0].preambleLen, receiver.mod_params->preambleLen);

    receiver = empty;
    rxExpressLrsSpiConfigMutable()->rateIndex = 3;
    rxExpressLrsSpiConfigMutable()->domain = ISM2400;
    expressLrsSpiInit(&injectedConfig, &config, &extiConfig);
    EXPECT_EQ(air_rate_config[1][0].index, receiver.mod_params->index);
    EXPECT_EQ(air_rate_config[1][0].enumRate, receiver.mod_params->enumRate);
    EXPECT_EQ(air_rate_config[1][0].bw, receiver.mod_params->bw);
    EXPECT_EQ(air_rate_config[1][0].sf, receiver.mod_params->sf);
    EXPECT_EQ(air_rate_config[1][0].cr, receiver.mod_params->cr);
    EXPECT_EQ(air_rate_config[1][0].interval, receiver.mod_params->interval);
    EXPECT_EQ(air_rate_config[1][0].tlmInterval, receiver.mod_params->tlmInterval);
    EXPECT_EQ(air_rate_config[1][0].fhssHopInterval, receiver.mod_params->fhssHopInterval);
    EXPECT_EQ(air_rate_config[1][0].preambleLen, receiver.mod_params->preambleLen);

    //check num channels based on hybrid switches
    receiver = empty;
    rxExpressLrsSpiConfigMutable()->hybridSwitches = false;
    expressLrsSpiInit(&injectedConfig, &config, &extiConfig);
    EXPECT_EQ(12, config.channelCount);
    receiver = empty;
    rxExpressLrsSpiConfigMutable()->hybridSwitches = true;
    expressLrsSpiInit(&injectedConfig, &config, &extiConfig);
    EXPECT_EQ(12, config.channelCount);
}

TEST(RxSpiExpressLrsUnitTest, TestInitBound)
{
    const uint8_t validUID[6] = {0, 0, 1, 2, 3, 4};
    receiver = empty;
    memcpy(rxExpressLrsSpiConfigMutable()->UID, validUID, 6);
    
    // check mod params
    for (int i=0; i < ELRS_RATE_MAX; i++) {
        receiver = empty;
        rxExpressLrsSpiConfigMutable()->rateIndex = i;
        rxExpressLrsSpiConfigMutable()->domain = FCC915;
        expressLrsSpiInit(&injectedConfig, &config, &extiConfig);
        EXPECT_EQ(air_rate_config[0][i].index, receiver.mod_params->index);
        EXPECT_EQ(air_rate_config[0][i].enumRate, receiver.mod_params->enumRate);
        EXPECT_EQ(air_rate_config[0][i].bw, receiver.mod_params->bw);
        EXPECT_EQ(air_rate_config[0][i].sf, receiver.mod_params->sf);
        EXPECT_EQ(air_rate_config[0][i].cr, receiver.mod_params->cr);
        EXPECT_EQ(air_rate_config[0][i].interval, receiver.mod_params->interval);
        EXPECT_EQ(air_rate_config[0][i].tlmInterval, receiver.mod_params->tlmInterval);
        EXPECT_EQ(air_rate_config[0][i].fhssHopInterval, receiver.mod_params->fhssHopInterval);
        EXPECT_EQ(air_rate_config[0][i].preambleLen, receiver.mod_params->preambleLen);

        receiver = empty;
        rxExpressLrsSpiConfigMutable()->rateIndex = i;
        rxExpressLrsSpiConfigMutable()->domain = ISM2400;
        expressLrsSpiInit(&injectedConfig, &config, &extiConfig);
        EXPECT_EQ(air_rate_config[1][i].index, receiver.mod_params->index);
        EXPECT_EQ(air_rate_config[1][i].enumRate, receiver.mod_params->enumRate);
        EXPECT_EQ(air_rate_config[1][i].bw, receiver.mod_params->bw);
        EXPECT_EQ(air_rate_config[1][i].sf, receiver.mod_params->sf);
        EXPECT_EQ(air_rate_config[1][i].cr, receiver.mod_params->cr);
        EXPECT_EQ(air_rate_config[1][i].interval, receiver.mod_params->interval);
        EXPECT_EQ(air_rate_config[1][i].tlmInterval, receiver.mod_params->tlmInterval);
        EXPECT_EQ(air_rate_config[1][i].fhssHopInterval, receiver.mod_params->fhssHopInterval);
        EXPECT_EQ(air_rate_config[1][i].preambleLen, receiver.mod_params->preambleLen);
    }

    expressLrsSpiInit(&injectedConfig, &config, &extiConfig);
    EXPECT_TRUE(receiver.bound);
    for (int i = 0; i < 6; i++) {
        EXPECT_EQ(validUID[i], receiver.UID[i]);
    }
}

TEST(RxSpiExpressLrsUnitTest, TestLQCalc)
{
    resetLQ();
    getLQ(true);
    getLQ(false);
    EXPECT_EQ(1, getLQ(false));
    for (int i = 2; i < 98; i++) {
        EXPECT_EQ(i, getLQ(true));
    }
    EXPECT_EQ(98, getLQ(true));
    EXPECT_EQ(98, getLQ(true)); //bug
    EXPECT_EQ(99, getLQ(true));
    EXPECT_EQ(100, getLQ(true));
    EXPECT_EQ(100, getLQ(true));
    EXPECT_EQ(100, getLQ(true));
    EXPECT_EQ(100, getLQ(true));
    EXPECT_EQ(99, getLQ(false));
    EXPECT_EQ(98, getLQ(false));
    for (int i = 97; i > 0; i--) {
        EXPECT_EQ(i, getLQ(false));
    }
    EXPECT_EQ(0, getLQ(false));
    EXPECT_EQ(0, getLQ(false));
    EXPECT_EQ(0, getLQ(false));
    EXPECT_EQ(1, getLQ(true));
    resetLQ();
    EXPECT_EQ(0, getLQ(false));
    EXPECT_EQ(1, getLQ(true));
}

TEST(RxSpiExpressLrsUnitTest, Test1bSwitchDecode)
{
    EXPECT_EQ(1000, convertSwitch1b(0));
    EXPECT_EQ(2000, convertSwitch1b(1));
    EXPECT_EQ(2000, convertSwitch1b(2));
    EXPECT_EQ(2000, convertSwitch1b(255));
}

TEST(RxSpiExpressLrsUnitTest, Test3bSwitchDecode)
{
    EXPECT_EQ(1000, convertSwitch3b(0));
    EXPECT_EQ(1275, convertSwitch3b(1));
    EXPECT_EQ(1425, convertSwitch3b(2));
    EXPECT_EQ(1575, convertSwitch3b(3));
    EXPECT_EQ(1725, convertSwitch3b(4));
    EXPECT_EQ(2000, convertSwitch3b(5));
    EXPECT_EQ(1500, convertSwitch3b(6));
    EXPECT_EQ(1500, convertSwitch3b(7));
    EXPECT_EQ(1500, convertSwitch3b(8));
    EXPECT_EQ(1500, convertSwitch3b(123));
    EXPECT_EQ(1500, convertSwitch3b(255));
}

TEST(RxSpiExpressLrsUnitTest, Test4bSwitchDecode)
{
    EXPECT_EQ(1000, convertSwitch4b(0));
    EXPECT_EQ(1066, convertSwitch4b(1));
    EXPECT_EQ(1133, convertSwitch4b(2));
    EXPECT_EQ(1200, convertSwitch4b(3));
    EXPECT_EQ(1266, convertSwitch4b(4));
    EXPECT_EQ(1333, convertSwitch4b(5));
    EXPECT_EQ(1400, convertSwitch4b(6));
    EXPECT_EQ(1466, convertSwitch4b(7));
    EXPECT_EQ(1533, convertSwitch4b(8));
    EXPECT_EQ(1600, convertSwitch4b(9));
    EXPECT_EQ(1666, convertSwitch4b(10));
    EXPECT_EQ(1733, convertSwitch4b(11));
    EXPECT_EQ(1800, convertSwitch4b(12));
    EXPECT_EQ(1866, convertSwitch4b(13));
    EXPECT_EQ(1933, convertSwitch4b(14));
    EXPECT_EQ(2000, convertSwitch4b(15));
    EXPECT_EQ(1500, convertSwitch4b(16));
    EXPECT_EQ(1500, convertSwitch4b(255));
}

// STUBS

extern "C" {

    int16_t *debug;
    uint8_t debugMode;

    rssiSource_e rssiSource;
    linkQualitySource_e linkQualitySource;
    void setRssi(uint16_t , rssiSource_e ) {}
    void setRssiDirect(uint16_t , rssiSource_e ) {}

    uint32_t micros(void) { return 0; }
    uint32_t millis(void) { return 0; }

    bool IORead(IO_t ) { return true; }
    IO_t IOGetByTag(ioTag_t ) { return (IO_t)1; }
    void IOHi(IO_t ) {}
    void IOLo(IO_t ) {}
    void writeEEPROM(void) {}

    void rxSpiCommonIOInit(const rxSpiConfig_t *) {}
    void rxSpiLedBlinkRxLoss(rx_spi_received_e ) {}
    void rxSpiLedBlinkBind(void) {}
    bool rxSpiCheckBindRequested(bool)
    {
        return false;
    }
    bool rxSpiExtiConfigured(void) { return true; }

    void sx1280Config(const sx1280_lora_bandwidths_e , const sx1280_lora_spreading_factors_e , const sx1280_lora_coding_rates_e , const uint32_t , const uint8_t , const bool ) {}
    void sx1280StartReceiving(void) {}
    bool sx1280ISR(uint32_t *timestamp)
    {
        *timestamp = 0;
        return true;
    }
    void sx1280TransmitData(const uint8_t *, const uint8_t ) {}
    void sx1280ReceiveData(uint8_t *, const uint8_t ) {}
    void sx1280SetFrequencyReg(const uint32_t ) {}
    void sx1280GetLastPacketStats(int8_t *rssi, int8_t *snr) 
    {
        *rssi = 0;
        *snr = 0;
    }
    void sx1280AdjustFrequency(int32_t , const uint32_t ) {}
    bool sx1280Init(IO_t , IO_t ) { return true; }

    void sx127xConfig(const sx127x_bandwidth_e , const sx127x_spreading_factor_e , const sx127x_coding_rate_e , const uint32_t , const uint8_t , const bool ) {}
    void sx127xStartReceiving(void) {}
    bool sx127xISR(uint32_t *timestamp)
    {
        *timestamp = 0;
        return true;
    }
    void sx127xTransmitData(const uint8_t *, const uint8_t ) {}
    void sx127xReceiveData(uint8_t *, const uint8_t ) {}
    void sx127xSetFrequencyReg(const uint32_t ) {}
    void sx127xGetLastPacketStats(int8_t *rssi, int8_t *snr) 
    {
        *rssi = 0;
        *snr = 0;
    }
    void sx127xAdjustFrequency(int32_t , const uint32_t ) {}
    bool sx127xInit(IO_t , IO_t ) { return true; };

    int scaleRange(int x, int srcFrom, int srcTo, int destFrom, int destTo) {
        long int a = ((long int) destTo - (long int) destFrom) * ((long int) x - (long int) srcFrom);
        long int b = (long int) srcTo - (long int) srcFrom;
        return (a / b) + destFrom;
    }
}
