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
#include <ctype.h>

#include "platform.h"

#ifdef USE_MSP_DISPLAYPORT

#include "common/utils.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "drivers/display.h"

#include "io/displayport_msp.h"

#include "msp/msp.h"
#include "msp/msp_protocol.h"
#include "msp/msp_serial.h"

// no template required since defaults are zero
PG_REGISTER(displayPortProfile_t, displayPortProfileMsp, PG_DISPLAY_PORT_MSP_CONFIG, 0);

static displayPort_t mspDisplayPort;

#ifdef USE_CLI
extern uint8_t cliMode;
#endif

static int output(displayPort_t *displayPort, uint8_t cmd, uint8_t *buf, int len)
{
    UNUSED(displayPort);

#ifdef USE_CLI
    // FIXME There should be no dependency on the CLI but mspSerialPush doesn't check for cli mode, and can't because it also shouldn't have a dependency on the CLI.
    if (cliMode) {
        return 0;
    }
#endif
    return mspSerialPush(cmd, buf, len, MSP_DIRECTION_REPLY);
}

static int heartbeat(displayPort_t *displayPort)
{
    uint8_t subcmd[] = { 0 };

    // heartbeat is used to:
    // a) ensure display is not released by MW OSD software
    // b) prevent OSD Slave boards from displaying a 'disconnected' status.
    return output(displayPort, MSP_DISPLAYPORT, subcmd, sizeof(subcmd));
}

static int grab(displayPort_t *displayPort)
{
    return heartbeat(displayPort);
}

static int release(displayPort_t *displayPort)
{
    uint8_t subcmd[] = { 1 };

    return output(displayPort, MSP_DISPLAYPORT, subcmd, sizeof(subcmd));
}

static int clearScreen(displayPort_t *displayPort)
{
    uint8_t subcmd[] = { 2 };

    return output(displayPort, MSP_DISPLAYPORT, subcmd, sizeof(subcmd));
}

static int drawScreen(displayPort_t *displayPort)
{
    uint8_t subcmd[] = { 4 };
    return output(displayPort, MSP_DISPLAYPORT, subcmd, sizeof(subcmd));
}

static int screenSize(const displayPort_t *displayPort)
{
    return displayPort->rows * displayPort->cols;
}

static int writeString(displayPort_t *displayPort, uint8_t col, uint8_t row, const char *string)
{
#define MSP_OSD_MAX_STRING_LENGTH 30 // FIXME move this
    uint8_t buf[MSP_OSD_MAX_STRING_LENGTH + 4];

    int len = strlen(string);
    if (len >= MSP_OSD_MAX_STRING_LENGTH) {
        len = MSP_OSD_MAX_STRING_LENGTH;
    }

    buf[0] = 3;
    buf[1] = row;
    buf[2] = col;
    buf[3] = 0;
    memcpy(&buf[4], string, len);

    return output(displayPort, MSP_DISPLAYPORT, buf, len + 4);
}

static int writeChar(displayPort_t *displayPort, uint8_t col, uint8_t row, uint8_t c)
{
    char buf[2];

    buf[0] = c;
    buf[1] = 0;
    return writeString(displayPort, col, row, buf); //!!TODO - check if there is a direct MSP command to do this
}
#ifdef USE_MAX7456_EXTENDED
static bool isExtended(void) { return false; }
#endif

static bool isTransferInProgress(const displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return false;
}

static bool isSynced(const displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return true;
}

static void resync(displayPort_t *displayPort)
{
    displayPort->rows = 13 + displayPortProfileMsp()->rowAdjust; // XXX Will reflect NTSC/PAL in the future
    displayPort->cols = 30 + displayPortProfileMsp()->colAdjust;
    drawScreen(displayPort);
}

static uint32_t txBytesFree(const displayPort_t *displayPort)
{
    UNUSED(displayPort);
    return mspSerialTxBytesFree();
}

static const displayPortVTable_t mspDisplayPortVTable = {
    .grab = grab,
    .release = release,
    .clearScreen = clearScreen,
    .drawScreen = drawScreen,
    .screenSize = screenSize,
    .writeString = writeString,
    .writeChar = writeChar,
#ifdef USE_MAX7456_EXTENDED
    .isExtended = isExtended,
#endif
    .isTransferInProgress = isTransferInProgress,
    .heartbeat = heartbeat,
    .resync = resync,
    .isSynced = isSynced,
    .txBytesFree = txBytesFree
};

displayPort_t *displayPortMspInit(void)
{
    displayInit(&mspDisplayPort, &mspDisplayPortVTable);
    resync(&mspDisplayPort);
    return &mspDisplayPort;
}
#endif // USE_MSP_DISPLAYPORT
