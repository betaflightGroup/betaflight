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
* CC2500 SPI drivers
*/
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#ifdef USE_RX_CC2500

#include "build/build_config.h"

#include "pg/rx.h"
#include "pg/rx_spi.h"

#include "drivers/io.h"
#include "drivers/rx/rx_spi.h"
#include "drivers/system.h"
#include "drivers/time.h"

#include "rx_cc2500.h"

#define NOP 0xFF

void cc2500ReadFifo(uint8_t *dpbuffer, uint8_t len)
{
    rxSpiReadCommandMulti(CC2500_3F_RXFIFO | CC2500_READ_BURST, NOP, dpbuffer, len);
}

void cc2500WriteCommand(uint8_t command, uint8_t data)
{
    // Burst writes require an interbyte gap, see fig. 7, pg. 22 in https://www.ti.com/lit/ds/symlink/cc2500.pdf
    // As such gaps can't be inserted if DMA is being used, inhibit DMA on this bus for the duration of this call
    bool dmaMode = rxSpiInhibitDma();
    rxSpiWriteCommand(command, data);
    rxSpiRestoreDma(dmaMode);
}

void cc2500WriteCommandMulti(uint8_t command, const uint8_t *data, uint8_t length)
{
    // Burst writes require an interbyte gap, see fig. 7, pg. 22 in https://www.ti.com/lit/ds/symlink/cc2500.pdf
    // As such gaps can't be inserted if DMA is being used, inhibit DMA on this bus for the duration of this call
    bool dmaMode = rxSpiInhibitDma();
    rxSpiWriteCommandMulti(command, data, length);
    rxSpiRestoreDma(dmaMode);
}

void cc2500WriteFifo(uint8_t *dpbuffer, uint8_t len)
{
    cc2500Strobe(CC2500_SFTX); // 0x3B SFTX
    cc2500WriteCommandMulti(CC2500_3F_TXFIFO | CC2500_WRITE_BURST,
                                 dpbuffer, len);
    cc2500Strobe(CC2500_STX); // 0x35
}

void cc2500ReadRegisterMulti(uint8_t address, uint8_t *data, uint8_t length)
{
    rxSpiReadCommandMulti(address, NOP, data, length);
}

void cc2500WriteRegisterMulti(uint8_t address, uint8_t *data,
                                  uint8_t length)
{
    cc2500WriteCommandMulti(address, data, length);
}

uint8_t cc2500ReadReg(uint8_t reg)
{
    return rxSpiReadCommand(reg | 0x80, NOP);
}

void cc2500Strobe(uint8_t address) { rxSpiWriteByte(address); }

void cc2500WriteReg(uint8_t address, uint8_t data)
{
    cc2500WriteCommand(address, data);
}

void cc2500SetPower(uint8_t power)
{
    const uint8_t patable[8] = {
        0xC5, // -12dbm
        0x97, // -10dbm
        0x6E, // -8dbm
        0x7F, // -6dbm
        0xA9, // -4dbm
        0xBB, // -2dbm
        0xFE, // 0dbm
        0xFF  // 1.5dbm
    };
    if (power > 7)
        power = 7;
    cc2500WriteReg(CC2500_3E_PATABLE, patable[power]);
}

uint8_t cc2500Reset(void)
{
    cc2500Strobe(CC2500_SRES);
    delayMicroseconds(1000); // 1000us
    // CC2500_SetTxRxMode(TXRX_OFF);
    // RX_EN_off;//off tx
    // TX_EN_off;//off rx
    return cc2500ReadReg(CC2500_0E_FREQ1) == 0xC4; // check if reset
}
#endif
