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
#include <string.h>

#include "platform.h"

#ifdef USE_RX_SX1280

#include "drivers/bus_spi.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/rx/rx_sx1280.h"
#include "drivers/rx/rx_spi.h"
#include "drivers/time.h"

static IO_t busy;

static bool sx1280PollBusy(void)
{
    uint32_t startTime = micros();
    while (IORead(busy)) {
        if ((micros() - startTime) > 1000) {
            return false;
        } else {
            __asm__("nop");
        }
    }
    return true;
}

bool sx1280Init(IO_t resetPin, IO_t busyPin)
{

    if (!rxSpiExtiConfigured()) {
        return false;
    }

    if (resetPin) {
        IOInit(resetPin, OWNER_RX_SPI_EXPRESSLRS_RESET, 0);
        IOConfigGPIO(resetPin, IOCFG_OUT_PP);
    } else {
        resetPin = IO_NONE;
    }

    if (busyPin) {
        IOInit(busyPin, OWNER_RX_SPI_EXPRESSLRS_BUSY, 0);
        IOConfigGPIO(busyPin, IOCFG_IPU);
    } else {
        busyPin = IO_NONE;
    }

    busy = busyPin;

    IOLo(resetPin);
    delay(50);
    IOConfigGPIO(resetPin, IOCFG_IN_FLOATING); // leave floating, internal pullup on sx1280 side
    delay(20);

    uint16_t firmwareRev = (((sx1280ReadRegister(REG_LR_FIRMWARE_VERSION_MSB)) << 8) | (sx1280ReadRegister(REG_LR_FIRMWARE_VERSION_MSB + 1)));

    if ((firmwareRev == 0) || (firmwareRev == 65535)) {
        return false;
    }

    return true;
}

bool sx1280ISR(timeUs_t *timeStamp)
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

void sx1280WriteCommand(const uint8_t address, const uint8_t data)
{
    sx1280PollBusy();
    rxSpiWriteCommand(address, data);
}

void sx1280WriteCommandBurst(const uint8_t address, const uint8_t *data, const uint8_t length)
{
    uint8_t outBuffer[length + 1];

    outBuffer[0] = address;

    memcpy(outBuffer + 1, data, length);

    sx1280PollBusy();
    rxSpiTransferCommandMulti(&outBuffer[0], length + 1);
}

void sx1280ReadCommandBurst(const uint8_t address, uint8_t *data, const uint8_t length)
{
    uint8_t outBuffer[length + 2];

    outBuffer[0] = address;
    outBuffer[1] = 0x00;

    memcpy(outBuffer + 2, data, length);

    sx1280PollBusy();
    rxSpiTransferCommandMulti(&outBuffer[0], length + 2);
    memcpy(data, outBuffer + 2, length);
}

void sx1280WriteRegisterBurst(const uint16_t address, const uint8_t *buffer, const uint8_t size)
{
    uint8_t outBuffer[size + 3];

    outBuffer[0] = (uint8_t) SX1280_RADIO_WRITE_REGISTER;
    outBuffer[1] = ((address & 0xFF00) >> 8);
    outBuffer[2] = (address & 0x00FF);

    memcpy(outBuffer + 3, buffer, size);

    sx1280PollBusy();
    rxSpiTransferCommandMulti(&outBuffer[0], size + 3);
}

void sx1280WriteRegister(const uint16_t address, const uint8_t value)
{
    sx1280WriteRegisterBurst(address, &value, 1);
}

void sx1280ReadRegisterBurst(const uint16_t address, uint8_t *buffer, const uint8_t size)
{
    uint8_t outBuffer[size + 4];

    outBuffer[0] = (uint8_t) SX1280_RADIO_READ_REGISTER;
    outBuffer[1] = ((address & 0xFF00) >> 8);
    outBuffer[2] = (address & 0x00FF);
    outBuffer[3] = 0x00;

    sx1280PollBusy();
    rxSpiTransferCommandMulti(&outBuffer[0], size + 4);
    memcpy(buffer, outBuffer + 4, size);
}

uint8_t sx1280ReadRegister(const uint16_t address)
{
    uint8_t data;
    sx1280ReadRegisterBurst(address, &data, 1);
    return data;
}

void sx1280WriteBuffer(const uint8_t offset, const uint8_t *buffer, const uint8_t size)
{
    uint8_t outBuffer[size + 2];

    outBuffer[0] = (uint8_t) SX1280_RADIO_WRITE_BUFFER;
    outBuffer[1] = offset;

    memcpy(outBuffer + 2, buffer, size);

    sx1280PollBusy();
    rxSpiTransferCommandMulti(&outBuffer[0], size + 2);
}

void sx1280ReadBuffer(const uint8_t offset, uint8_t *buffer, const uint8_t size)
{
    uint8_t outBuffer[size + 3];

    outBuffer[0] = (uint8_t) SX1280_RADIO_READ_BUFFER;
    outBuffer[1] = offset;
    outBuffer[2] = 0x00;

    sx1280PollBusy();
    rxSpiTransferCommandMulti(&outBuffer[0], size + 3);
    memcpy(buffer, outBuffer + 3, size);
}

uint8_t sx1280GetStatus(void)
{
    uint8_t buffer[3] = {(uint8_t) SX1280_RADIO_GET_STATUS, 0, 0};
    sx1280PollBusy();
    rxSpiTransferCommandMulti(&buffer[0], 3);
    return buffer[0];
}

void sx1280ConfigLoraDefaults(void)
{
    sx1280SetMode(SX1280_MODE_STDBY_RC);                                      //step 1 put in STDBY_RC mode
    sx1280WriteCommand(SX1280_RADIO_SET_PACKETTYPE, SX1280_PACKET_TYPE_LORA); //Step 2: set packet type to LoRa
    sx1280ConfigLoraModParams(SX1280_LORA_BW_0800, SX1280_LORA_SF6, SX1280_LORA_CR_4_7); //Step 5: Configure Modulation Params
    sx1280WriteCommand(SX1280_RADIO_SET_AUTOFS, 0x01);                        //enable auto FS
    sx1280WriteRegister(0x0891, (sx1280ReadRegister(0x0891) | 0xC0));         //default is low power mode, switch to high sensitivity instead
    sx1280SetPacketParams(12, SX1280_LORA_PACKET_IMPLICIT, 8, SX1280_LORA_CRC_OFF, SX1280_LORA_IQ_NORMAL); //default params
    sx1280SetFrequencyHZ(2400000000);                                         //Step 3: Set Freq
    sx1280SetFIFOaddr(0x00, 0x00);                                            //Step 4: Config FIFO addr
    sx1280SetDioIrqParams(SX1280_IRQ_RADIO_ALL, SX1280_IRQ_TX_DONE | SX1280_IRQ_RX_DONE, SX1280_IRQ_RADIO_NONE, SX1280_IRQ_RADIO_NONE); //set IRQ to both RXdone/TXdone on DIO1
}

void sx1280Config(const sx1280_lora_bandwidths_e bw, const sx1280_lora_spreading_factors_e sf, const sx1280_lora_coding_rates_e cr, const uint32_t freq, const uint8_t preambleLength, const bool iqInverted)
{
    sx1280ConfigLoraDefaults();
    sx1280SetOutputPower(13); //default is max power (12.5dBm for SX1280 RX)
    sx1280SetMode(SX1280_MODE_STDBY_XOSC); 
    sx1280ClearIrqStatus(SX1280_IRQ_RADIO_ALL);
    sx1280ConfigLoraModParams(bw, sf, cr);
    sx1280SetPacketParams(preambleLength, SX1280_LORA_PACKET_IMPLICIT, 8, SX1280_LORA_CRC_OFF, (sx1280_lora_iq_modes_e)((uint8_t)!iqInverted << 6)); // TODO don't make static etc.
    sx1280SetFrequencyReg(freq);
}

void sx1280SetOutputPower(const int8_t power)
{
    uint8_t buf[2];
    buf[0] = power + 18;
    buf[1] = (uint8_t) SX1280_RADIO_RAMP_04_US;
    sx1280WriteCommandBurst(SX1280_RADIO_SET_TXPARAMS, buf, 2);
}

void sx1280SetPacketParams(const uint8_t preambleLength, const sx1280_lora_packet_lengths_modes_e headerType, const uint8_t payloadLength, const sx1280_lora_crc_modes_e crc, const sx1280_lora_iq_modes_e invertIQ)
{
    uint8_t buf[7];

    buf[0] = preambleLength;
    buf[1] = headerType;
    buf[2] = payloadLength;
    buf[3] = crc;
    buf[4] = invertIQ;
    buf[5] = 0x00;
    buf[6] = 0x00;

    sx1280WriteCommandBurst(SX1280_RADIO_SET_PACKETPARAMS, buf, 7);
}

void sx1280SetMode(const sx1280_operating_modes_e opMode)
{
    uint8_t buf[3];

    switch (opMode)
    {

    case SX1280_MODE_SLEEP:
        sx1280WriteCommand(SX1280_RADIO_SET_SLEEP, 0x01);
        break;

    case SX1280_MODE_CALIBRATION:
        break;

    case SX1280_MODE_STDBY_RC:
        sx1280WriteCommand(SX1280_RADIO_SET_STANDBY, SX1280_STDBY_RC);
        break;
        
    case SX1280_MODE_STDBY_XOSC:
        sx1280WriteCommand(SX1280_RADIO_SET_STANDBY, SX1280_STDBY_XOSC);
        break;

    case SX1280_MODE_FS:
        sx1280WriteCommand(SX1280_RADIO_SET_FS, 0x00);
        break;

    case SX1280_MODE_RX:
        buf[0] = 0x00; // periodBase = 1ms, page 71 datasheet, set to FF for cont RX
        buf[1] = 0xFF;
        buf[2] = 0xFF;
        sx1280WriteCommandBurst(SX1280_RADIO_SET_RX, buf, 3);
        break;

    case SX1280_MODE_TX:
        //uses timeout Time-out duration = periodBase * periodBaseCount
        buf[0] = 0x00; // periodBase = 1ms, page 71 datasheet
        buf[1] = 0xFF; // no timeout set for now
        buf[2] = 0xFF; // TODO dynamic timeout based on expected onairtime
        sx1280WriteCommandBurst(SX1280_RADIO_SET_TX, buf, 3);
        break;

    case SX1280_MODE_CAD:
        break;

    default:
        break;
    }
}

void sx1280ConfigLoraModParams(const sx1280_lora_bandwidths_e bw, const sx1280_lora_spreading_factors_e sf, const sx1280_lora_coding_rates_e cr)
{
    // Care must therefore be taken to ensure that modulation parameters are set using the command
    // SetModulationParam() only after defining the packet type SetPacketType() to be used

    uint8_t rfparams[3] = {0};

    rfparams[0] = (uint8_t)sf;
    rfparams[1] = (uint8_t)bw;
    rfparams[2] = (uint8_t)cr;

    sx1280WriteCommandBurst(SX1280_RADIO_SET_MODULATIONPARAMS, rfparams, 3);

    switch (sf)
    {
    case SX1280_LORA_SF5:
    case SX1280_LORA_SF6:
        sx1280WriteRegister(0x925, 0x1E); // for SF5 or SF6
        break;
    case SX1280_LORA_SF7:
    case SX1280_LORA_SF8:
        sx1280WriteRegister(0x925, 0x37); // for SF7 or SF8
        break;
    default:
        sx1280WriteRegister(0x925, 0x32); // for SF9, SF10, SF11, SF12
    }
}

void sx1280SetFrequencyHZ(const uint32_t reqFreq)
{
    uint8_t buf[3] = {0};

    uint32_t freq = (uint32_t)((double)reqFreq / (double)SX1280_FREQ_STEP);
    buf[0] = (uint8_t)((freq >> 16) & 0xFF);
    buf[1] = (uint8_t)((freq >> 8) & 0xFF);
    buf[2] = (uint8_t)(freq & 0xFF);

    sx1280WriteCommandBurst(SX1280_RADIO_SET_RFFREQUENCY, buf, 3);
}

void sx1280SetFrequencyReg(const uint32_t freq)
{
    uint8_t buf[3] = {0};

    buf[0] = (uint8_t)((freq >> 16) & 0xFF);
    buf[1] = (uint8_t)((freq >> 8) & 0xFF);
    buf[2] = (uint8_t)(freq & 0xFF);

    sx1280WriteCommandBurst(SX1280_RADIO_SET_RFFREQUENCY, buf, 3);
}

int32_t sx1280GetFrequencyError(void)
{
    uint8_t efeRaw[3] = {0};
    uint32_t efe = 0;
    double efeHz = 0.0;

    efeRaw[0] = sx1280ReadRegister(SX1280_REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB);
    efeRaw[1] = sx1280ReadRegister(SX1280_REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB + 1);
    efeRaw[2] = sx1280ReadRegister(SX1280_REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB + 2);
    efe = (efeRaw[0] << 16) | (efeRaw[1] << 8) | efeRaw[2];

    efe &= SX1280_REG_LR_ESTIMATED_FREQUENCY_ERROR_MASK;

    //efeHz = 1.55 * (double)complement2(efe, 20) / (1600.0 / (double)GetLoRaBandwidth() * 1000.0);
    return efeHz;
}

void sx1280AdjustFrequency(int32_t offset, const uint32_t freq)
{
    //Not implemented
    UNUSED(offset);
    UNUSED(freq);
}

void sx1280SetFIFOaddr(const uint8_t txBaseAddr, const uint8_t rxBaseAddr)
{
    uint8_t buf[2];

    buf[0] = txBaseAddr;
    buf[1] = rxBaseAddr;
    sx1280WriteCommandBurst(SX1280_RADIO_SET_BUFFERBASEADDRESS, buf, 2);
}

void sx1280SetDioIrqParams(const uint16_t irqMask, const uint16_t dio1Mask, const uint16_t dio2Mask, const uint16_t dio3Mask)
{
    uint8_t buf[8];

    buf[0] = (uint8_t)((irqMask >> 8) & 0x00FF);
    buf[1] = (uint8_t)(irqMask & 0x00FF);
    buf[2] = (uint8_t)((dio1Mask >> 8) & 0x00FF);
    buf[3] = (uint8_t)(dio1Mask & 0x00FF);
    buf[4] = (uint8_t)((dio2Mask >> 8) & 0x00FF);
    buf[5] = (uint8_t)(dio2Mask & 0x00FF);
    buf[6] = (uint8_t)((dio3Mask >> 8) & 0x00FF);
    buf[7] = (uint8_t)(dio3Mask & 0x00FF);

    sx1280WriteCommandBurst(SX1280_RADIO_SET_DIOIRQPARAMS, buf, 8);
}

void sx1280ClearIrqStatus(const uint16_t irqMask)
{
    uint8_t buf[2];

    buf[0] = (uint8_t)(((uint16_t)irqMask >> 8) & 0x00FF);
    buf[1] = (uint8_t)((uint16_t)irqMask & 0x00FF);

    sx1280WriteCommandBurst(SX1280_RADIO_CLR_IRQSTATUS, buf, 2);
}

void sx1280TransmitData(const uint8_t *data, const uint8_t length)
{
    /*if (instance->currOpmode == SX1280_MODE_TX) //catch TX timeout
    {
        instance->ClearIrqStatus(SX1280_IRQ_RADIO_ALL);
        instance->SetMode(SX1280_MODE_FS);
        TXnbISR();
        return;
    }*/
    sx1280ClearIrqStatus(SX1280_IRQ_RADIO_ALL);
    sx1280WriteBuffer(0x00, data, length);
    sx1280SetMode(SX1280_MODE_TX);
}

static uint8_t sx1280GetRxBufferAddr(void)
{
    uint8_t status[2] = {0};
    sx1280ReadCommandBurst(SX1280_RADIO_GET_RXBUFFERSTATUS, status, 2);
    return status[1];
}

void sx1280ReceiveData(uint8_t *data, const uint8_t length)
{
    sx1280ClearIrqStatus(SX1280_IRQ_RADIO_ALL);
    uint8_t FIFOaddr = sx1280GetRxBufferAddr();
    sx1280ReadBuffer(FIFOaddr, data, length);
}

void sx1280StartReceiving(void)
{
    sx1280ClearIrqStatus(SX1280_IRQ_RADIO_ALL);
    sx1280SetMode(SX1280_MODE_RX);
}

bool sx1280GetFrequencyErrorbool(void)
{
    uint8_t regEFI[3];

    sx1280ReadRegisterBurst(SX1280_REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB, regEFI, 3);

    //bool result = (val & 0b00001000) >> 3;
    //return result; // returns true if pos freq error, neg if false
    return 0;
}

void sx1280GetLastPacketStats(int8_t *rssi, int8_t *snr)
{
    uint8_t status[2];

    sx1280ReadCommandBurst(SX1280_RADIO_GET_PACKETSTATUS, status, 2);
    *rssi = -(int8_t)(status[0] / 2);
    *snr = ((int8_t) status[1]) / 4;
}

#endif /* USE_RX_SX1280 */