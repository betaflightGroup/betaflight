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

#pragma once

//SX127x_ModulationModes
typedef enum {
    SX127x_OPMODE_FSK_OOK = 0x00,
    SX127x_OPMODE_LORA = 0x80,
    SX127X_ACCESS_SHARED_REG_OFF = 0x00,
    SX127X_ACCESS_SHARED_REG_ON = 0x40,
} sx127x_modulation_mode_e;

//SX127x_RadioOPmodes
typedef enum {
    SX127x_OPMODE_SLEEP = 0x00,
    SX127x_OPMODE_STANDBY = 0x01,
    SX127x_OPMODE_FSTX = 0x02,
    SX127x_OPMODE_TX = 0x03,
    SX127x_OPMODE_FSRX = 0x04,
    SX127x_OPMODE_RXCONTINUOUS = 0x05,
    SX127x_OPMODE_RXSINGLE = 0x06,
    SX127x_OPMODE_CAD = 0x07,
} sx127x_radio_op_mode_e;

//SX127x_Bandwidth
typedef enum {
    SX127x_BW_7_80_KHZ = 0x00,
    SX127x_BW_10_40_KHZ = 0x10,
    SX127x_BW_15_60_KHZ = 0x20,
    SX127x_BW_20_80_KHZ = 0x30,
    SX127x_BW_31_25_KHZ = 0x40,
    SX127x_BW_41_70_KHZ = 0x50,
    SX127x_BW_62_50_KHZ = 0x60,
    SX127x_BW_125_00_KHZ = 0x70,
    SX127x_BW_250_00_KHZ = 0x80,
    SX127x_BW_500_00_KHZ = 0x90,
} sx127x_bandwidth_e;

//SX127x_SpreadingFactor
typedef enum {
    SX127x_SF_6 = 0x60,
    SX127x_SF_7 = 0x70,
    SX127x_SF_8 = 0x80,
    SX127x_SF_9 = 0x90,
    SX127x_SF_10 = 0xA0,
    SX127x_SF_11 = 0xB0,
    SX127x_SF_12 = 0xC0,
} sx127x_spreading_factor_e;

//SX127x_CodingRate
typedef enum {
    SX127x_CR_4_5 = 0x02,
    SX127x_CR_4_6 = 0x04,
    SX127x_CR_4_7 = 0x06,
    SX127x_CR_4_8 = 0x08,
} sx127x_coding_rate_e;

// SX127x series common registers
#define SX127X_REG_FIFO 0x00
#define SX127X_REG_OP_MODE 0x01
#define SX127X_REG_FRF_MSB 0x06
#define SX127X_REG_FRF_MID 0x07
#define SX127X_REG_FRF_LSB 0x08
#define SX127X_REG_PA_CONFIG 0x09
#define SX127X_REG_PA_RAMP 0x0A
#define SX127X_REG_OCP 0x0B
#define SX127X_REG_LNA 0x0C
#define SX127X_REG_FIFO_ADDR_PTR 0x0D
#define SX127X_REG_FIFO_TX_BASE_ADDR 0x0E
#define SX127X_REG_FIFO_RX_BASE_ADDR 0x0F
#define SX127X_REG_FIFO_RX_CURRENT_ADDR 0x10
#define SX127X_REG_IRQ_FLAGS_MASK 0x11
#define SX127X_REG_IRQ_FLAGS 0x12
#define SX127X_REG_RX_NB_BYTES 0x13
#define SX127X_REG_RX_HEADER_CNT_VALUE_MSB 0x14
#define SX127X_REG_RX_HEADER_CNT_VALUE_LSB 0x15
#define SX127X_REG_RX_PACKET_CNT_VALUE_MSB 0x16
#define SX127X_REG_RX_PACKET_CNT_VALUE_LSB 0x17
#define SX127X_REG_MODEM_STAT 0x18
#define SX127X_REG_PKT_SNR_VALUE 0x19
#define SX127X_REG_PKT_RSSI_VALUE 0x1A
#define SX127X_REG_RSSI_VALUE 0x1B
#define SX127X_REG_HOP_CHANNEL 0x1C
#define SX127X_REG_MODEM_CONFIG_1 0x1D
#define SX127X_REG_MODEM_CONFIG_2 0x1E
#define SX127X_REG_SYMB_TIMEOUT_LSB 0x1F
#define SX127X_REG_PREAMBLE_MSB 0x20
#define SX127X_REG_PREAMBLE_LSB 0x21
#define SX127X_REG_PAYLOAD_LENGTH 0x22
#define SX127X_REG_MAX_PAYLOAD_LENGTH 0x23
#define SX127X_REG_HOP_PERIOD 0x24
#define SX127X_REG_FIFO_RX_BYTE_ADDR 0x25
#define SX127X_REG_FEI_MSB 0x28
#define SX127X_REG_FEI_MID 0x29
#define SX127X_REG_FEI_LSB 0x2A
#define SX127X_REG_RSSI_WIDEBAND 0x2C
#define SX127X_REG_DETECT_OPTIMIZE 0x31
#define SX127X_REG_INVERT_IQ 0x33
#define SX127X_REG_DETECTION_THRESHOLD 0x37
#define SX127X_REG_SYNC_WORD 0x39
#define SX127X_REG_DIO_MAPPING_1 0x40
#define SX127X_REG_DIO_MAPPING_2 0x41
#define SX127X_REG_VERSION 0x42

// SX127X_REG_PA_CONFIG
#define SX127X_PA_SELECT_RFO 0x00    //  7     7     RFO pin output, power limited to +14 dBm
#define SX127X_PA_SELECT_BOOST 0x80  //  7     7     PA_BOOST pin output, power limited to +20 dBm
#define SX127X_OUTPUT_POWER 0x0F     //  3     0     output power: P_out = 17 - (15 - OUTPUT_POWER) [dBm] for PA_SELECT_BOOST
#define SX127X_MAX_OUTPUT_POWER 0x70 //              Enable max output power

// SX127X_REG_OCP
#define SX127X_OCP_OFF 0x00   //  5     5     PA overload current protection disabled
#define SX127X_OCP_ON 0x20    //  5     5     PA overload current protection enabled
#define SX127X_OCP_TRIM 0x0B  //  4     0     OCP current: I_max(OCP_TRIM = 0b1011) = 100 mA
#define SX127X_OCP_150MA 0x12 //  4     0     OCP current: I_max(OCP_TRIM = 10010) = 150 mA

// SX127X_REG_LNA
#define SX127X_LNA_GAIN_0 0x00    //  7     5     LNA gain setting:   not used
#define SX127X_LNA_GAIN_1 0x20    //  7     5                         max gain
#define SX127X_LNA_GAIN_2 0x40    //  7     5                         .
#define SX127X_LNA_GAIN_3 0x60    //  7     5                         .
#define SX127X_LNA_GAIN_4 0x80    //  7     5                         .
#define SX127X_LNA_GAIN_5 0xA0    //  7     5                         .
#define SX127X_LNA_GAIN_6 0xC0    //  7     5                         min gain
#define SX127X_LNA_GAIN_7 0xE0    //  7     5                         not used
#define SX127X_LNA_BOOST_OFF 0x00 //  1     0     default LNA current
#define SX127X_LNA_BOOST_ON 0x03  //  1     0     150% LNA current

#define SX127X_TX_MODE_SINGLE 0x00 //  3     3     single TX
#define SX127X_TX_MODE_CONT 0x08   //  3     3     continuous TX
#define SX127X_RX_TIMEOUT_MSB 0x00 //  1     0

// SX127X_REG_SYMB_TIMEOUT_LSB
#define SX127X_RX_TIMEOUT_LSB 0x64 //  7     0     10 bit RX operation timeout

// SX127X_REG_PREAMBLE_MSB + REG_PREAMBLE_LSB
#define SX127X_PREAMBLE_LENGTH_MSB 0x00 //  7     0     2 byte preamble length setting: l_P = PREAMBLE_LENGTH + 4.25
#define SX127X_PREAMBLE_LENGTH_LSB 0x08 //  7     0         where l_p = preamble length
//#define SX127X_PREAMBLE_LENGTH_LSB                    0x04  //  7     0         where l_p = preamble length  //CHANGED

// SX127X_REG_DETECT_OPTIMIZE
#define SX127X_DETECT_OPTIMIZE_SF_6 0x05    //  2     0     SF6 detection optimization
#define SX127X_DETECT_OPTIMIZE_SF_7_12 0x03 //  2     0     SF7 to SF12 detection optimization

// SX127X_REG_DETECTION_THRESHOLD
#define SX127X_DETECTION_THRESHOLD_SF_6 0x0C    //  7     0     SF6 detection threshold
#define SX127X_DETECTION_THRESHOLD_SF_7_12 0x0A //  7     0     SF7 to SF12 detection threshold

// SX127X_REG_PA_DAC
#define SX127X_PA_BOOST_OFF 0x04 //  2     0     PA_BOOST disabled
#define SX127X_PA_BOOST_ON 0x07  //  2     0     +20 dBm on PA_BOOST when OUTPUT_POWER = 0b1111

// SX127X_REG_HOP_PERIOD
#define SX127X_HOP_PERIOD_OFF 0x00 //  7     0     number of periods between frequency hops; 0 = disabled
#define SX127X_HOP_PERIOD_MAX 0xFF //  7     0

// SX127X_REG_DIO_MAPPING_1
#define SX127X_DIO0_RX_DONE 0x00             //  7     6
#define SX127X_DIO0_TX_DONE 0x40             //  7     6
#define SX127X_DIO0_CAD_DONE 0x80            //  7     6
#define SX127X_DIO1_RX_TIMEOUT 0x00          //  5     4
#define SX127X_DIO1_FHSS_CHANGE_CHANNEL 0x10 //  5     4
#define SX127X_DIO1_CAD_DETECTED 0x20        //  5     4

// SX127X_REG_IRQ_FLAGS
#define SX127X_CLEAR_IRQ_FLAG_RX_TIMEOUT 0x80          //  7     7     timeout
#define SX127X_CLEAR_IRQ_FLAG_RX_DONE 0x40             //  6     6     packet reception complete
#define SX127X_CLEAR_IRQ_FLAG_PAYLOAD_CRC_ERROR 0x20   //  5     5     payload CRC error
#define SX127X_CLEAR_IRQ_FLAG_VALID_HEADER 0x10        //  4     4     valid header received
#define SX127X_CLEAR_IRQ_FLAG_TX_DONE 0x08             //  3     3     payload transmission complete
#define SX127X_CLEAR_IRQ_FLAG_CAD_DONE 0x04            //  2     2     CAD complete
#define SX127X_CLEAR_IRQ_FLAG_FHSS_CHANGE_CHANNEL 0x02 //  1     1     FHSS change channel
#define SX127X_CLEAR_IRQ_FLAG_CAD_DETECTED 0x01        //  0     0     valid LoRa signal detected during CAD operation

// SX127X_REG_IRQ_FLAGS_MASK
#define SX127X_MASK_IRQ_FLAG_RX_TIMEOUT 0x7F          //  7     7     timeout
#define SX127X_MASK_IRQ_FLAG_RX_DONE 0xBF             //  6     6     packet reception complete
#define SX127X_MASK_IRQ_FLAG_PAYLOAD_CRC_ERROR 0xDF   //  5     5     payload CRC error
#define SX127X_MASK_IRQ_FLAG_VALID_HEADER 0xEF        //  4     4     valid header received
#define SX127X_MASK_IRQ_FLAG_TX_DONE 0xF7             //  3     3     payload transmission complete
#define SX127X_MASK_IRQ_FLAG_CAD_DONE 0xFB            //  2     2     CAD complete
#define SX127X_MASK_IRQ_FLAG_FHSS_CHANGE_CHANNEL 0xFD //  1     1     FHSS change channel
#define SX127X_MASK_IRQ_FLAG_CAD_DETECTED 0xFE        //  0     0     valid LoRa signal detected during CAD operation

// SX127X_REG_FIFO_TX_BASE_ADDR
#define SX127X_FIFO_TX_BASE_ADDR_MAX 0x00 //  7     0     allocate the entire FIFO buffer for TX only

// SX127X_REG_FIFO_RX_BASE_ADDR
#define SX127X_FIFO_RX_BASE_ADDR_MAX 0x00 //  7     0     allocate the entire FIFO buffer for RX only

// SX127X_REG_SYNC_WORD
//#define SX127X_SYNC_WORD 0xC8 //  200   0     default ExpressLRS sync word - 200Hz
#define SX127X_SYNC_WORD                              0x12        //  18    0     default LoRa sync word
#define SX127X_SYNC_WORD_LORAWAN 0x34 //  52    0     sync word reserved for LoRaWAN networks

///Added by Sandro
#define SX127x_TXCONTINUOUSMODE_MASK 0xF7
#define SX127x_TXCONTINUOUSMODE_ON 0x08
#define SX127x_TXCONTINUOUSMODE_OFF 0x00
#define SX127x_PPMOFFSET 0x27

///// SX1278 Regs /////
//SX1278 specific register map
#define SX1278_REG_MODEM_CONFIG_3 0x26
#define SX1278_REG_TCXO 0x4B
#define SX1278_REG_PA_DAC 0x4D
#define SX1278_REG_FORMER_TEMP 0x5D
#define SX1278_REG_AGC_REF 0x61
#define SX1278_REG_AGC_THRESH_1 0x62
#define SX1278_REG_AGC_THRESH_2 0x63
#define SX1278_REG_AGC_THRESH_3 0x64
#define SX1278_REG_PLL 0x70

//SX1278 LoRa modem settings
//SX1278_REG_OP_MODE                                                  MSB   LSB   DESCRIPTION
#define SX1278_HIGH_FREQ 0x00 //  3     3     access HF test registers
#define SX1278_LOW_FREQ 0x08  //  3     3     access LF test registers

//SX1278_REG_FRF_MSB + REG_FRF_MID + REG_FRF_LSB
#define SX1278_FRF_MSB 0x6C //  7     0     carrier frequency setting: f_RF = (F(XOSC) * FRF)/2^19
#define SX1278_FRF_MID 0x80 //  7     0         where F(XOSC) = 32 MHz
#define SX1278_FRF_LSB 0x00 //  7     0               FRF = 3 byte value of FRF registers

//SX1278_REG_PA_CONFIG
#define SX1278_MAX_POWER 0x70 //  6     4     max power: P_max = 10.8 + 0.6*MAX_POWER [dBm]; P_max(MAX_POWER = 0b111) = 15 dBm
//#define SX1278_MAX_POWER                              0x10  //  6     4     changed

//SX1278_REG_LNA
#define SX1278_LNA_BOOST_LF_OFF 0x00 //  4     3     default LNA current

#define SX1278_HEADER_EXPL_MODE 0x00 //  0     0     explicit header mode
#define SX1278_HEADER_IMPL_MODE 0x01 //  0     0     implicit header mode

//SX1278_REG_MODEM_CONFIG_2
#define SX1278_RX_CRC_MODE_OFF 0x00 //  2     2     CRC disabled
#define SX1278_RX_CRC_MODE_ON 0x04  //  2     2     CRC enabled

//SX1278_REG_MODEM_CONFIG_3
#define SX1278_LOW_DATA_RATE_OPT_OFF 0x00 //  3     3     low data rate optimization disabled
#define SX1278_LOW_DATA_RATE_OPT_ON 0x08  //  3     3     low data rate optimization enabled
#define SX1278_AGC_AUTO_OFF 0x00          //  2     2     LNA gain set by REG_LNA
#define SX1278_AGC_AUTO_ON 0x04           //  2     2     LNA gain set by internal AGC loop

#define SX1276_HEADER_EXPL_MODE 0x00 //  0     0     explicit header mode
#define SX1276_HEADER_IMPL_MODE 0x01 //  0     0     implicit header mode

#define SX127x_ERR_NONE 0x00
#define SX127x_ERR_CHIP_NOT_FOUND 0x01
#define SX127x_ERR_EEPROM_NOT_INITIALIZED 0x02

#define SX127x_ERR_PACKET_TOO_LONG 0x10
#define SX127x_ERR_TX_TIMEOUT 0x11

#define SX127x_ERR_RX_TIMEOUT 0x20
#define SX127x_ERR_CRC_MISMATCH 0x21

#define SX127x_ERR_INVALID_BANDWIDTH 0x30
#define SX127x_ERR_INVALID_SPREADING_FACTOR 0x31
#define SX127x_ERR_INVALID_CODING_RATE 0x32
#define SX127x_ERR_INVALID_FREQUENCY 0x33

#define SX127x_ERR_INVALID_BIT_RANGE 0x40

#define SX127x_CHANNEL_FREE 0x50
#define SX127x_PREAMBLE_DETECTED 0x51

#define SX127x_SPI_READ 0x00
#define SX127x_SPI_WRITE 0x80

#define SX127x_MAX_POWER 0x0F //17dBm

#define SX127x_FREQ_STEP 61.03515625

#define SX127x_FREQ_CORRECTION_MAX ((int32_t)(100000 / SX127x_FREQ_STEP)) 
#define SX127x_FREQ_CORRECTION_MIN ((int32_t)(-100000 / SX127x_FREQ_STEP))

bool sx127xInit(IO_t resetPin, IO_t busyPin);
bool sx127xISR(uint32_t *timeStamp);
void sx127xWriteRegister(const uint8_t address, const uint8_t data);
void sx127xWriteRegisterBurst(const uint8_t address, const uint8_t *data, const uint8_t length);
uint8_t sx127xReadRegister(const uint8_t address);
void sx127xReadRegisterBurst(const uint8_t address, uint8_t *data, const uint8_t length);
uint8_t sx127xGetRegisterValue(const uint8_t reg, const uint8_t msb, const uint8_t lsb);
uint8_t sx127xSetRegisterValue(const uint8_t reg, const uint8_t value, const uint8_t msb, const uint8_t lsb);
void sx127xReadRegisterFIFO(uint8_t *data, const uint8_t length);
void sx127xWriteRegisterFIFO(const uint8_t *data, const uint8_t length);
void sx127xSetBandwidthCodingRate(const sx127x_bandwidth_e bw, const sx127x_coding_rate_e cr, const sx127x_spreading_factor_e sf, const bool headerExplMode, const bool crcEnabled);
void sx127xSetSyncWord(uint8_t syncWord);
void sx127xSetMode(const sx127x_radio_op_mode_e mode);
void sx127xSetOutputPower(const uint8_t power);
void sx127xSetPreambleLength(const uint8_t preambleLen);
void sx127xSetSpreadingFactor(const sx127x_spreading_factor_e sf);
void sx127xSetFrequencyHZ(const uint32_t freq);
void sx127xSetFrequencyReg(const uint32_t freq);

void sx127xTransmitData(const uint8_t *data, const uint8_t length);
void sx127xReceiveData(uint8_t *data, const uint8_t length);
void sx127xStartReceiving(void);
void sx127xConfig(const sx127x_bandwidth_e bw, const sx127x_spreading_factor_e sf, const sx127x_coding_rate_e cr, const uint32_t freq, const uint8_t preambleLen, const bool iqInverted);

uint32_t sx127xGetCurrBandwidth(const sx127x_bandwidth_e bw);
uint32_t sx127xGetCurrBandwidthNormalisedShifted(const sx127x_bandwidth_e bw);
void sx127xSetPPMoffsetReg(const int32_t offset, const uint32_t freq);
int32_t sx127xGetFrequencyError(const sx127x_bandwidth_e bw);
void sx127xAdjustFrequency(int32_t offset, const uint32_t freq);
uint8_t sx127xUnsignedGetLastPacketRSSI(void);
int8_t sx127xGetLastPacketRSSI(void);
int8_t sx127xGetCurrRSSI(void);
int8_t sx127xGetLastPacketSNR(void);
void sx127xClearIRQFlags(void);
void sx127xGetLastPacketStats(int8_t *rssi, int8_t *snr);

void sx127xConfigLoraDefaults(const bool iqInverted);