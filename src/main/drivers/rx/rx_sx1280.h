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

#define REG_LR_FIRMWARE_VERSION_MSB 0x0153 //The address of the register holding the firmware version MSB
#define SX1280_REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB 0x0954
#define SX1280_REG_LR_ESTIMATED_FREQUENCY_ERROR_MASK 0x0FFFFF

#define SX1280_XTAL_FREQ 52000000
#define SX1280_FREQ_STEP ((double)(SX1280_XTAL_FREQ / 262144.0))

typedef enum
{
    SX1280_RF_IDLE = 0x00, //!< The radio is idle
    SX1280_RF_RX_RUNNING,  //!< The radio is in reception state
    SX1280_RF_TX_RUNNING,  //!< The radio is in transmission state
    SX1280_RF_CAD,         //!< The radio is doing channel activity detection
} sxx1280_states_e;

/*!
 * \brief Represents the operating mode the radio is actually running
 */
typedef enum
{
    SX1280_MODE_SLEEP = 0x00, //! The radio is in sleep mode
    SX1280_MODE_CALIBRATION,  //! The radio is in calibration mode
    SX1280_MODE_STDBY_RC,     //! The radio is in standby mode with RC oscillator
    SX1280_MODE_STDBY_XOSC,   //! The radio is in standby mode with XOSC oscillator
    SX1280_MODE_FS,           //! The radio is in frequency synthesis mode
    SX1280_MODE_RX,           //! The radio is in receive mode
    SX1280_MODE_TX,           //! The radio is in transmit mode
    SX1280_MODE_CAD           //! The radio is in channel activity detection mode
} sx1280_operating_modes_e;

/*!
 * \brief Declares the oscillator in use while in standby mode
 *
 * Using the STDBY_RC standby mode allow to reduce the energy consumption
 * STDBY_XOSC should be used for time critical applications
 */
typedef enum
{
    SX1280_STDBY_RC = 0x00,
    SX1280_STDBY_XOSC = 0x01,
} sx1280_standby_modes_e;

/*!
 * \brief Declares the power regulation used to power the device
 *
 * This command allows the user to specify if DC-DC or LDO is used for power regulation.
 * Using only LDO implies that the Rx or Tx current is doubled
 */
typedef enum
{
    SX1280_USE_LDO = 0x00,  //! Use LDO (default value)
    SX1280_USE_DCDC = 0x01, //! Use DCDC
} sx1280_regulator_modes_e;

/*!
 * \brief Represents the possible packet type (i.e. modem) used
 */
typedef enum
{
    SX1280_PACKET_TYPE_GFSK = 0x00,
    SX1280_PACKET_TYPE_LORA,
    SX1280_PACKET_TYPE_RANGING,
    SX1280_PACKET_TYPE_FLRC,
    SX1280_PACKET_TYPE_BLE,
    SX1280_PACKET_TYPE_NONE = 0x0F,
} sx1280_packet_types_e;

typedef enum
{
    SX1280_LORA_IQ_NORMAL = 0x40,
    SX1280_LORA_IQ_INVERTED = 0x00,
} sx1280_lora_iq_modes_e;

typedef enum
{
    SX1280_RADIO_CRC_OFF = 0x00, //!< No CRC in use
    SX1280_RADIO_CRC_1_BYTES = 0x10,
    SX1280_RADIO_CRC_2_BYTES = 0x20,
    SX1280_RADIO_CRC_3_BYTES = 0x30,
} sx1280_crc_types_e;

/*!
 * \brief Represents the ramping time for power amplifier
 */
typedef enum
{
    SX1280_RADIO_RAMP_02_US = 0x00,
    SX1280_RADIO_RAMP_04_US = 0x20,
    SX1280_RADIO_RAMP_06_US = 0x40,
    SX1280_RADIO_RAMP_08_US = 0x60,
    SX1280_RADIO_RAMP_10_US = 0x80,
    SX1280_RADIO_RAMP_12_US = 0xA0,
    SX1280_RADIO_RAMP_16_US = 0xC0,
    SX1280_RADIO_RAMP_20_US = 0xE0,
} sx1280_ramp_times_e;

/*!
 * \brief Represents the number of symbols to be used for channel activity detection operation
 */
typedef enum
{
    SX1280_LORA_CAD_01_SYMBOL = 0x00,
    SX1280_LORA_CAD_02_SYMBOLS = 0x20,
    SX1280_LORA_CAD_04_SYMBOLS = 0x40,
    SX1280_LORA_CAD_08_SYMBOLS = 0x60,
    SX1280_LORA_CAD_16_SYMBOLS = 0x80,
} sx1280_lora_cad_symbols_e;

/*!
 * \brief Represents the possible spreading factor values in LORA packet types
 */
typedef enum
{
    SX1280_LORA_SF5 = 0x50,
    SX1280_LORA_SF6 = 0x60,
    SX1280_LORA_SF7 = 0x70,
    SX1280_LORA_SF8 = 0x80,
    SX1280_LORA_SF9 = 0x90,
    SX1280_LORA_SF10 = 0xA0,
    SX1280_LORA_SF11 = 0xB0,
    SX1280_LORA_SF12 = 0xC0,
} sx1280_lora_spreading_factors_e;

/*!
 * \brief Represents the bandwidth values for LORA packet type
 */
typedef enum
{
    SX1280_LORA_BW_0200 = 0x34,
    SX1280_LORA_BW_0400 = 0x26,
    SX1280_LORA_BW_0800 = 0x18,
    SX1280_LORA_BW_1600 = 0x0A,
} sx1280_lora_bandwidths_e;

/*!
 * \brief Represents the coding rate values for LORA packet type
 */
typedef enum
{
    SX1280_LORA_CR_4_5 = 0x01,
    SX1280_LORA_CR_4_6 = 0x02,
    SX1280_LORA_CR_4_7 = 0x03,
    SX1280_LORA_CR_4_8 = 0x04,
    SX1280_LORA_CR_LI_4_5 = 0x05,
    SX1280_LORA_CR_LI_4_6 = 0x06,
    SX1280_LORA_CR_LI_4_7 = 0x07,
} sx1280_lora_coding_rates_e;

typedef enum
{
    SX1280_LORA_PACKET_VARIABLE_LENGTH = 0x00, //!< The packet is on variable size, header included
    SX1280_LORA_PACKET_FIXED_LENGTH = 0x80,    //!< The packet is known on both sides, no header included in the packet
    SX1280_LORA_PACKET_EXPLICIT = SX1280_LORA_PACKET_VARIABLE_LENGTH,
    SX1280_LORA_PACKET_IMPLICIT = SX1280_LORA_PACKET_FIXED_LENGTH,
} sx1280_lora_packet_lengths_modes_e;

typedef enum
{
    SX1280_LORA_CRC_ON = 0x20,  //!< CRC activated
    SX1280_LORA_CRC_OFF = 0x00, //!< CRC not used
} sx1280_lora_crc_modes_e;

typedef enum
{
    SX1280_RADIO_GET_STATUS = 0xC0,
    SX1280_RADIO_WRITE_REGISTER = 0x18,
    SX1280_RADIO_READ_REGISTER = 0x19,
    SX1280_RADIO_WRITE_BUFFER = 0x1A,
    SX1280_RADIO_READ_BUFFER = 0x1B,
    SX1280_RADIO_SET_SLEEP = 0x84,
    SX1280_RADIO_SET_STANDBY = 0x80,
    SX1280_RADIO_SET_FS = 0xC1,
    SX1280_RADIO_SET_TX = 0x83,
    SX1280_RADIO_SET_RX = 0x82,
    SX1280_RADIO_SET_RXDUTYCYCLE = 0x94,
    SX1280_RADIO_SET_CAD = 0xC5,
    SX1280_RADIO_SET_TXCONTINUOUSWAVE = 0xD1,
    SX1280_RADIO_SET_TXCONTINUOUSPREAMBLE = 0xD2,
    SX1280_RADIO_SET_PACKETTYPE = 0x8A,
    SX1280_RADIO_GET_PACKETTYPE = 0x03,
    SX1280_RADIO_SET_RFFREQUENCY = 0x86,
    SX1280_RADIO_SET_TXPARAMS = 0x8E,
    SX1280_RADIO_SET_CADPARAMS = 0x88,
    SX1280_RADIO_SET_BUFFERBASEADDRESS = 0x8F,
    SX1280_RADIO_SET_MODULATIONPARAMS = 0x8B,
    SX1280_RADIO_SET_PACKETPARAMS = 0x8C,
    SX1280_RADIO_GET_RXBUFFERSTATUS = 0x17,
    SX1280_RADIO_GET_PACKETSTATUS = 0x1D,
    SX1280_RADIO_GET_RSSIINST = 0x1F,
    SX1280_RADIO_SET_DIOIRQPARAMS = 0x8D,
    SX1280_RADIO_GET_IRQSTATUS = 0x15,
    SX1280_RADIO_CLR_IRQSTATUS = 0x97,
    SX1280_RADIO_CALIBRATE = 0x89,
    SX1280_RADIO_SET_REGULATORMODE = 0x96,
    SX1280_RADIO_SET_SAVECONTEXT = 0xD5,
    SX1280_RADIO_SET_AUTOTX = 0x98,
    SX1280_RADIO_SET_AUTOFS = 0x9E,
    SX1280_RADIO_SET_LONGPREAMBLE = 0x9B,
    SX1280_RADIO_SET_UARTSPEED = 0x9D,
    SX1280_RADIO_SET_RANGING_ROLE = 0xA3,
} sx1280_commands_e;

typedef enum
{
    SX1280_IRQ_RADIO_NONE = 0x0000,
    SX1280_IRQ_TX_DONE = 0x0001,
    SX1280_IRQ_RX_DONE = 0x0002,
    SX1280_IRQ_SYNCWORD_VALID = 0x0004,
    SX1280_IRQ_SYNCWORD_ERROR = 0x0008,
    SX1280_IRQ_HEADER_VALID = 0x0010,
    SX1280_IRQ_HEADER_ERROR = 0x0020,
    SX1280_IRQ_CRC_ERROR = 0x0040,
    SX1280_IRQ_RANGING_SLAVE_RESPONSE_DONE = 0x0080,
    SX1280_IRQ_RANGING_SLAVE_REQUEST_DISCARDED = 0x0100,
    SX1280_IRQ_RANGING_MASTER_RESULT_VALID = 0x0200,
    SX1280_IRQ_RANGING_MASTER_TIMEOUT = 0x0400,
    SX1280_IRQ_RANGING_SLAVE_REQUEST_VALID = 0x0800,
    SX1280_IRQ_CAD_DONE = 0x1000,
    SX1280_IRQ_CAD_DETECTED = 0x2000,
    SX1280_IRQ_RX_TX_TIMEOUT = 0x4000,
    SX1280_IRQ_PREAMBLE_DETECTED = 0x8000,
    SX1280_IRQ_RADIO_ALL = 0xFFFF,
} sx1280_irq_masks_e;

typedef enum
{
    SX1280_RADIO_DIO1 = 0x02,
    SX1280_RADIO_DIO2 = 0x04,
    SX1280_RADIO_DIO3 = 0x08,
} sx1280_dios_e;

typedef enum
{
    SX1280_RADIO_TICK_SIZE_0015_US = 0x00,
    SX1280_RADIO_TICK_SIZE_0062_US = 0x01,
    SX1280_RADIO_TICK_SIZE_1000_US = 0x02,
    SX1280_RADIO_TICK_SIZE_4000_US = 0x03,
} sx1280_tick_sizes_e;

bool sx1280Init(IO_t resetPin, IO_t busyPin);
bool sx1280ISR(uint32_t *timeStamp);
void sx1280WriteCommand(const uint8_t address, const uint8_t data);
void sx1280WriteCommandBurst(const uint8_t address, const uint8_t *data, const uint8_t length);
void sx1280ReadCommandBurst(const uint8_t address, uint8_t *data, const uint8_t length);
void sx1280WriteRegisterBurst(const uint16_t address, const uint8_t *buffer, const uint8_t size);
void sx1280WriteRegister(const uint16_t address, const uint8_t value);
void sx1280ReadRegisterBurst(const uint16_t address, uint8_t *buffer, const uint8_t size);
uint8_t sx1280ReadRegister(const uint16_t address);
void sx1280WriteBuffer(const uint8_t offset, const uint8_t *buffer, const uint8_t size);
void sx1280ReadBuffer(const uint8_t offset, uint8_t *buffer, const uint8_t size);

uint8_t sx1280GetStatus(void);
void sx1280ConfigLoraDefaults(void);
void sx1280Config(const sx1280_lora_bandwidths_e bw, const sx1280_lora_spreading_factors_e sf, const sx1280_lora_coding_rates_e cr, const uint32_t freq, const uint8_t preambleLength, const bool iqInverted);
void sx1280SetOutputPower(const int8_t power);
void sx1280SetPacketParams(const uint8_t preambleLength, const sx1280_lora_packet_lengths_modes_e headerType, const uint8_t payloadLength, const sx1280_lora_crc_modes_e crc, const sx1280_lora_iq_modes_e invertIQ);
void sx1280SetMode(const sx1280_operating_modes_e opMode);
void sx1280ConfigLoraModParams(const sx1280_lora_bandwidths_e bw, const sx1280_lora_spreading_factors_e sf, const sx1280_lora_coding_rates_e cr);
void sx1280SetFrequencyHZ(const uint32_t reqFreq);
void sx1280SetFrequencyReg(const uint32_t freq);
int32_t sx1280GetFrequencyError(void);
void sx1280AdjustFrequency(int32_t offset, const uint32_t freq);
void sx1280SetFIFOaddr(const uint8_t txBaseAddr, const uint8_t rxBaseAddr);
void sx1280SetDioIrqParams(const uint16_t irqMask, const uint16_t dio1Mask, const uint16_t dio2Mask, const uint16_t dio3Mask);
void sx1280ClearIrqStatus(const uint16_t irqMask);

void sx1280TransmitData(const uint8_t *data, const uint8_t length);
void sx1280ReceiveData(uint8_t *data, const uint8_t length);
void sx1280StartReceiving(void);

bool sx1280GetFrequencyErrorbool(void);
void sx1280GetLastPacketStats(int8_t *rssi, int8_t *snr);