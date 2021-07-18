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

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#ifdef USE_RX_SX127X

#include "drivers/bus_spi.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/rx/rx_sx127x.h"
#include "drivers/rx/rx_spi.h"
#include "drivers/time.h"

static const uint8_t sx127xAllowedSyncwords[105] =
    {0, 5, 6, 7, 11, 12, 13, 15, 18,
     21, 23, 26, 29, 30, 31, 33, 34,
     37, 38, 39, 40, 42, 44, 50, 51,
     54, 55, 57, 58, 59, 61, 63, 65,
     67, 68, 71, 77, 78, 79, 80, 82,
     84, 86, 89, 92, 94, 96, 97, 99,
     101, 102, 105, 106, 109, 111, 113, 115,
     117, 118, 119, 121, 122, 124, 126, 127,
     129, 130, 138, 143, 161, 170, 172, 173,
     175, 180, 181, 182, 187, 190, 191, 192,
     193, 196, 199, 201, 204, 205, 208, 209,
     212, 213, 219, 220, 221, 223, 227, 229,
     235, 239, 240, 242, 243, 246, 247, 255};

static bool sx127xDetectChip(void)
{
    uint8_t i = 0;
    bool flagFound = false;
    while ((i < 3) && !flagFound) {
        uint8_t version = sx127xReadRegister(SX127X_REG_VERSION);
        if (version == 0x12) {
            flagFound = true;
        } else {
            delay(50);
            i++;
        }
    }

    sx127xSetRegisterValue(SX127X_REG_OP_MODE, SX127x_OPMODE_SLEEP, 2, 0);
    return flagFound;
}

bool sx127xISR(timeUs_t *timeStamp)
{
    if (rxSpiPollExti()) {
        if (rxSpiGetLastExtiTimeUs()) {
            *timeStamp = rxSpiGetLastExtiTimeUs();
        }

        rxSpiResetExti();

        return true;
    }
    return false;
}

bool sx127xInit(IO_t resetPin, IO_t busyPin)
{
    UNUSED(busyPin);

    if (!rxSpiExtiConfigured()) {
        return false;
    }

    if (resetPin) {
        IOInit(resetPin, OWNER_RX_SPI_EXPRESSLRS_RESET, 0);
        IOConfigGPIO(resetPin, IOCFG_OUT_PP);
    } else {
        resetPin = IO_NONE;
    }

    IOLo(resetPin);
    delay(50);
    IOConfigGPIO(resetPin, IOCFG_IN_FLOATING); // leave floating
    
    return sx127xDetectChip();
}

void sx127xWriteRegister(const uint8_t address, const uint8_t data)
{
    rxSpiWriteCommand(address | SX127x_SPI_WRITE, data);
}

void sx127xWriteRegisterBurst(const uint8_t address, const uint8_t *data, const uint8_t length)
{
    rxSpiWriteCommandMulti(address | SX127x_SPI_WRITE, &data[0], length);
}

uint8_t sx127xReadRegister(const uint8_t address)
{
    return rxSpiReadCommand(address | SX127x_SPI_READ, 0xFF);
}

void sx127xReadRegisterBurst(const uint8_t address, uint8_t data[], const uint8_t length)
{
    rxSpiReadCommandMulti(address | SX127x_SPI_READ, 0xFF, &data[0], length);
}

uint8_t sx127xGetRegisterValue(const uint8_t reg, const uint8_t msb, const uint8_t lsb)
{
    if ((msb > 7) || (lsb > 7) || (lsb > msb)) {
        return (SX127x_ERR_INVALID_BIT_RANGE);
    }
    uint8_t rawValue = sx127xReadRegister(reg);
    return rawValue & ((0xFF << lsb) & (0xFF >> (7 - msb)));
}

uint8_t sx127xSetRegisterValue(const uint8_t reg, const uint8_t value, const uint8_t msb, const uint8_t lsb)
{
    if ((msb > 7) || (lsb > 7) || (lsb > msb)) {
        return (SX127x_ERR_INVALID_BIT_RANGE);
    }

    uint8_t currentValue = sx127xReadRegister(reg);
    uint8_t mask = ~((0xFF << (msb + 1)) | (0xFF >> (8 - lsb)));
    uint8_t newValue = (currentValue & ~mask) | (value & mask);
    sx127xWriteRegister(reg, newValue);
    return (SX127x_ERR_NONE);
}

void sx127xReadRegisterFIFO(uint8_t data[], const uint8_t length)
{
    sx127xReadRegisterBurst(SX127X_REG_FIFO, data, length);
}

void sx127xWriteRegisterFIFO(const uint8_t *data, const uint8_t length)
{
    sx127xWriteRegisterBurst(SX127X_REG_FIFO, data, length);
}

void sx127xSetBandwidthCodingRate(const sx127x_bandwidth_e bw, const sx127x_coding_rate_e cr, const sx127x_spreading_factor_e sf, const bool headerExplMode, const bool crcEnabled)
{
    if (sf == SX127x_SF_6) { // set SF6 optimizations
        sx127xWriteRegister(SX127X_REG_MODEM_CONFIG_1, bw | cr | SX1278_HEADER_IMPL_MODE);
        sx127xSetRegisterValue(SX127X_REG_MODEM_CONFIG_2, SX1278_RX_CRC_MODE_OFF, 2, 2);
    } else {
        if (headerExplMode) {
            sx127xWriteRegister(SX127X_REG_MODEM_CONFIG_1, bw | cr | SX1278_HEADER_EXPL_MODE);
        } else {
            sx127xWriteRegister(SX127X_REG_MODEM_CONFIG_1, bw | cr | SX1278_HEADER_IMPL_MODE);
        }

        if (crcEnabled) {
            sx127xSetRegisterValue(SX127X_REG_MODEM_CONFIG_2, SX1278_RX_CRC_MODE_ON, 2, 2);
        } else {
            sx127xSetRegisterValue(SX127X_REG_MODEM_CONFIG_2, SX1278_RX_CRC_MODE_OFF, 2, 2);
        }
    }

    if (bw == SX127x_BW_500_00_KHZ) {
        //datasheet errata reconmendation http://caxapa.ru/thumbs/972894/SX1276_77_8_ErrataNote_1.1_STD.pdf
        sx127xWriteRegister(0x36, 0x02);
        sx127xWriteRegister(0x3a, 0x64);
    } else {
        sx127xWriteRegister(0x36, 0x03);
    }
}

static bool sx127xSyncWordOk(const uint8_t syncWord)
{
    for (unsigned int i = 0; i < sizeof(sx127xAllowedSyncwords); i++) {
        if (syncWord == sx127xAllowedSyncwords[i]) {
            return true;
        }
    }
    return false;
}

void sx127xSetSyncWord(uint8_t syncWord)
{
    while (sx127xSyncWordOk(syncWord) == false) {
        syncWord++;
    }

    sx127xWriteRegister(SX127X_REG_SYNC_WORD, syncWord); //TODO: possible bug in original code
}

void sx127xSetMode(const sx127x_radio_op_mode_e mode)
{
    sx127xWriteRegister(SX127x_OPMODE_LORA | SX127X_REG_OP_MODE, mode);
}

void sx127xSetOutputPower(const uint8_t power)
{
    sx127xSetMode(SX127x_OPMODE_STANDBY);
    sx127xWriteRegister(SX127X_REG_PA_CONFIG, SX127X_PA_SELECT_BOOST | SX127X_MAX_OUTPUT_POWER | power);
}

void sx127xSetPreambleLength(const uint8_t preambleLen)
{
    sx127xWriteRegister(SX127X_REG_PREAMBLE_LSB, preambleLen);
}

void sx127xSetSpreadingFactor(const sx127x_spreading_factor_e sf)
{
    sx127xSetRegisterValue(SX127X_REG_MODEM_CONFIG_2, sf | SX127X_TX_MODE_SINGLE, 7, 3);
    if (sf == SX127x_SF_6) {
        sx127xSetRegisterValue(SX127X_REG_DETECT_OPTIMIZE, SX127X_DETECT_OPTIMIZE_SF_6, 2, 0);
        sx127xWriteRegister(SX127X_REG_DETECTION_THRESHOLD, SX127X_DETECTION_THRESHOLD_SF_6);
    } else {
        sx127xSetRegisterValue(SX127X_REG_DETECT_OPTIMIZE, SX127X_DETECT_OPTIMIZE_SF_7_12, 2, 0);
        sx127xWriteRegister(SX127X_REG_DETECTION_THRESHOLD, SX127X_DETECTION_THRESHOLD_SF_7_12);
    }
}

void sx127xSetFrequencyHZ(const uint32_t freq)
{
    sx127xSetMode(SX127x_OPMODE_STANDBY);

    int32_t FRQ = ((uint32_t)((double)freq / (double)SX127x_FREQ_STEP));

    uint8_t FRQ_MSB = (uint8_t)((FRQ >> 16) & 0xFF);
    uint8_t FRQ_MID = (uint8_t)((FRQ >> 8) & 0xFF);
    uint8_t FRQ_LSB = (uint8_t)(FRQ & 0xFF);

    uint8_t outbuff[3] = {FRQ_MSB, FRQ_MID, FRQ_LSB};

    sx127xWriteRegisterBurst(SX127X_REG_FRF_MSB, outbuff, sizeof(outbuff));
}

void sx127xSetFrequencyReg(const uint32_t freq)
{
    sx127xSetMode(SX127x_OPMODE_STANDBY);

    uint8_t FRQ_MSB = (uint8_t)((freq >> 16) & 0xFF);
    uint8_t FRQ_MID = (uint8_t)((freq >> 8) & 0xFF);
    uint8_t FRQ_LSB = (uint8_t)(freq & 0xFF);

    uint8_t outbuff[3] = {FRQ_MSB, FRQ_MID, FRQ_LSB}; //check speedup

    sx127xWriteRegisterBurst(SX127X_REG_FRF_MSB, outbuff, sizeof(outbuff));
}

void sx127xTransmitData(const uint8_t *data, const uint8_t length)
{
    sx127xClearIRQFlags();
    sx127xSetMode(SX127x_OPMODE_STANDBY);

    sx127xWriteRegister(SX127X_REG_FIFO_ADDR_PTR, SX127X_FIFO_TX_BASE_ADDR_MAX);
    sx127xWriteRegisterFIFO(data, length);

    sx127xSetMode(SX127x_OPMODE_TX);
}

void sx127xReceiveData(uint8_t *data, const uint8_t length)
{
    sx127xClearIRQFlags();
    sx127xReadRegisterFIFO(data, length);
}

void sx127xStartReceiving(void)
{
    sx127xClearIRQFlags();
    sx127xSetMode(SX127x_OPMODE_STANDBY);
    sx127xWriteRegister(SX127X_REG_FIFO_ADDR_PTR, SX127X_FIFO_RX_BASE_ADDR_MAX);
    sx127xSetMode(SX127x_OPMODE_RXCONTINUOUS);
}

void sx127xConfig(const sx127x_bandwidth_e bw, const sx127x_spreading_factor_e sf, const sx127x_coding_rate_e cr, const uint32_t freq, const uint8_t preambleLen, const bool iqInverted)
{
    sx127xConfigLoraDefaults(iqInverted);
    sx127xSetPreambleLength(preambleLen);
    sx127xSetOutputPower(SX127x_MAX_POWER);
    sx127xSetSpreadingFactor(sf);
    sx127xSetBandwidthCodingRate(bw, cr, sf, false, false);
    sx127xSetFrequencyReg(freq);
}

uint32_t sx127xGetCurrBandwidth(const sx127x_bandwidth_e bw)
{
    switch (bw) {
    case SX127x_BW_7_80_KHZ:
        return 7.8E3;
    case SX127x_BW_10_40_KHZ:
        return 10.4E3;
    case SX127x_BW_15_60_KHZ:
        return 15.6E3;
    case SX127x_BW_20_80_KHZ:
        return 20.8E3;
    case SX127x_BW_31_25_KHZ:
        return 31.25E3;
    case SX127x_BW_41_70_KHZ:
        return 41.7E3;
    case SX127x_BW_62_50_KHZ:
        return 62.5E3;
    case SX127x_BW_125_00_KHZ:
        return 125E3;
    case SX127x_BW_250_00_KHZ:
        return 250E3;
    case SX127x_BW_500_00_KHZ:
        return 500E3;
    }
    return -1;
}

uint32_t sx127xGetCurrBandwidthNormalisedShifted(const sx127x_bandwidth_e bw) // this is basically just used for speedier calc of the freq offset, pre compiled for 32mhz xtal
{
    switch (bw) {
    case SX127x_BW_7_80_KHZ:
        return 1026;
    case SX127x_BW_10_40_KHZ:
        return 769;
    case SX127x_BW_15_60_KHZ:
        return 513;
    case SX127x_BW_20_80_KHZ:
        return 385;
    case SX127x_BW_31_25_KHZ:
        return 256;
    case SX127x_BW_41_70_KHZ:
        return 192;
    case SX127x_BW_62_50_KHZ:
        return 128;
    case SX127x_BW_125_00_KHZ:
        return 64;
    case SX127x_BW_250_00_KHZ:
        return 32;
    case SX127x_BW_500_00_KHZ:
        return 16;
    }
    return -1;
}

void sx127xSetPPMoffsetReg(const int32_t offset, const uint32_t freq)
{
    int8_t offsetPPM = (offset * 1e6 / freq) * 95 / 100;
    sx127xWriteRegister(SX127x_PPMOFFSET, (uint8_t) offsetPPM);
}

static bool sx127xGetFrequencyErrorbool(void)
{
    return (sx127xReadRegister(SX127X_REG_FEI_MSB) & 0x08) >> 3; // returns true if pos freq error, neg if false
}

int32_t sx127xGetFrequencyError(const sx127x_bandwidth_e bw)
{
    uint8_t reg[3] = {0x0, 0x0, 0x0};
    sx127xReadRegisterBurst(SX127X_REG_FEI_MSB, reg, sizeof(reg));

    uint32_t regFei = ((reg[0] & 0x07) << 16) + (reg[1] << 8) + reg[2];

    int32_t intFreqError = regFei;

    if ((reg[0] & 0x08) >> 3) {
        intFreqError -= 524288; // Sign bit is on
    }

    int32_t fErrorHZ = (intFreqError >> 3) * (sx127xGetCurrBandwidthNormalisedShifted(bw)); // bit shift hackery so we don't have to use floaty bois; the >> 3 is intentional and is a simplification of the formula on page 114 of sx1276 datasheet
    fErrorHZ >>= 4;

    return fErrorHZ;
}

void sx127xAdjustFrequency(int32_t offset, const uint32_t freq)
{
    if (sx127xGetFrequencyErrorbool()) { //logic is flipped compared to original code
        if (offset > SX127x_FREQ_CORRECTION_MIN) {
            offset -= 1;
        } else {
            offset = 0; //reset because something went wrong
        }
    } else {
        if (offset < SX127x_FREQ_CORRECTION_MAX) {
            offset += 1;
        } else {
            offset = 0; //reset because something went wrong
        }
    }
    sx127xSetPPMoffsetReg(offset, freq); //as above but corrects a different PPM offset based on freq error
}

uint8_t sx127xUnsignedGetLastPacketRSSI(void)
{
    return sx127xGetRegisterValue(SX127X_REG_PKT_RSSI_VALUE, 7, 0);
}

int8_t sx127xGetLastPacketRSSI(void)
{
    return (-157 + sx127xGetRegisterValue(SX127X_REG_PKT_RSSI_VALUE, 7, 0));
}

int8_t sx127xGetCurrRSSI(void)
{
    return (-157 + sx127xGetRegisterValue(SX127X_REG_RSSI_VALUE, 7, 0));
}

int8_t sx127xGetLastPacketSNR(void)
{
    int8_t rawSNR = (int8_t)sx127xGetRegisterValue(SX127X_REG_PKT_SNR_VALUE, 7, 0);
    return (rawSNR / 4);
}

void sx127xGetLastPacketStats(int8_t *rssi, int8_t *snr)
{
    *rssi = sx127xGetLastPacketRSSI();
    *snr = sx127xGetLastPacketSNR();
}

void sx127xClearIRQFlags(void)
{
    sx127xWriteRegister(SX127X_REG_IRQ_FLAGS, 0xFF);
}

void sx127xConfigLoraDefaults(const bool iqInverted)
{
    sx127xWriteRegister(SX127X_REG_OP_MODE, SX127x_OPMODE_SLEEP);
    sx127xWriteRegister(SX127X_REG_OP_MODE, SX127x_OPMODE_LORA); //must be written in sleep mode
    sx127xSetMode(SX127x_OPMODE_STANDBY);

    sx127xWriteRegister(SX127X_REG_PAYLOAD_LENGTH, 8);
    sx127xSetSyncWord(SX127X_SYNC_WORD);
    sx127xWriteRegister(SX127X_REG_FIFO_TX_BASE_ADDR, SX127X_FIFO_TX_BASE_ADDR_MAX);
    sx127xWriteRegister(SX127X_REG_FIFO_RX_BASE_ADDR, SX127X_FIFO_RX_BASE_ADDR_MAX);
    sx127xSetRegisterValue(SX127X_REG_DIO_MAPPING_1, 0xC0, 7, 6); //undocumented "hack", looking at Table 18 from datasheet SX127X_REG_DIO_MAPPING_1 = 11 appears to be unspported by infact it generates an intterupt on both RXdone and TXdone, this saves switching modes.
    sx127xWriteRegister(SX127X_REG_LNA, SX127X_LNA_BOOST_ON);
    sx127xWriteRegister(SX1278_REG_MODEM_CONFIG_3, SX1278_AGC_AUTO_ON | SX1278_LOW_DATA_RATE_OPT_OFF);
    sx127xSetRegisterValue(SX127X_REG_OCP, SX127X_OCP_ON | SX127X_OCP_150MA, 5, 0); //150ma max current
    sx127xSetPreambleLength(SX127X_PREAMBLE_LENGTH_LSB);
    sx127xSetRegisterValue(SX127X_REG_INVERT_IQ, (uint8_t)iqInverted, 6, 6);
}

#endif /* USE_RX_SX127X */