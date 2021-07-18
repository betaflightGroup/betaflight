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

//
// CMS things for blackbox and flashfs.
//

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>

#include "platform.h"

#if defined(USE_CMS) && defined(USE_BLACKBOX)

#include "build/debug.h"
#include "build/version.h"

#include "blackbox/blackbox.h"
#include "blackbox/blackbox_io.h"

#include "cms/cms.h"
#include "cms/cms_types.h"
#include "cms/cms_menu_blackbox.h"

#include "common/printf.h"
#include "common/utils.h"

#include "config/feature.h"

#include "drivers/flash.h"
#include "drivers/time.h"
#include "drivers/sdcard.h"

#include "config/config.h"

#include "io/asyncfatfs/asyncfatfs.h"
#include "io/flashfs.h"
#include "io/beeper.h"

#include "pg/pg.h"

#include "flight/pid.h"

static const char * const cmsx_BlackboxDeviceNames[] = {
    "NONE",
    "FLASH",
    "SDCARD",
    "SERIAL"
};

static const char * const cmsx_BlackboxRateNames[] = {
    "1/1",
    "1/2",
    "1/4",
    "1/8",
    "1/16"
};

static uint8_t cmsx_BlackboxDevice;
static OSD_TAB_t cmsx_BlackboxDeviceTable = { &cmsx_BlackboxDevice, 3, cmsx_BlackboxDeviceNames };
static uint8_t cmsx_BlackboxRate;
static OSD_TAB_t cmsx_BlackboxRateTable = { &cmsx_BlackboxRate, 4, cmsx_BlackboxRateNames };
static debugType_e systemConfig_debug_mode;

#define CMS_BLACKBOX_STRING_LENGTH 8
static char cmsx_BlackboxStatus[CMS_BLACKBOX_STRING_LENGTH];
static char cmsx_BlackboxDeviceStorageUsed[CMS_BLACKBOX_STRING_LENGTH];
static char cmsx_BlackboxDeviceStorageFree[CMS_BLACKBOX_STRING_LENGTH];
static char cmsx_pidFreq[CMS_BLACKBOX_STRING_LENGTH];

static void cmsx_Blackbox_GetDeviceStatus(void)
{
    char * unit = "B";
#if defined(USE_SDCARD) || defined(USE_FLASHFS)
    bool storageDeviceIsWorking = false;
#endif
    uint32_t storageUsed = 0;
    uint32_t storageFree = 0;

    switch (blackboxConfig()->device)
    {
#ifdef USE_SDCARD
    case BLACKBOX_DEVICE_SDCARD:
        unit = "MB";

        if (!sdcard_isInserted()) {
            tfp_sprintf(cmsx_BlackboxStatus, "NO CARD");
        } else if (!sdcard_isFunctional()) {
            tfp_sprintf(cmsx_BlackboxStatus, "FAULT");
        } else {
            switch (afatfs_getFilesystemState()) {
            case AFATFS_FILESYSTEM_STATE_READY:
                tfp_sprintf(cmsx_BlackboxStatus, "READY");
                storageDeviceIsWorking = true;
                break;
            case AFATFS_FILESYSTEM_STATE_INITIALIZATION:
                tfp_sprintf(cmsx_BlackboxStatus, "INIT");
                break;
            case AFATFS_FILESYSTEM_STATE_FATAL:
            case AFATFS_FILESYSTEM_STATE_UNKNOWN:
            default:
                tfp_sprintf(cmsx_BlackboxStatus, "FAULT");
                break;
            }
        }

        if (storageDeviceIsWorking) {
            storageFree = afatfs_getContiguousFreeSpace() / 1024000;
            storageUsed = (sdcard_getMetadata()->numBlocks / 2000) - storageFree;
        }

        break;
#endif

#ifdef USE_FLASHFS
    case BLACKBOX_DEVICE_FLASH:
        unit = "KB";

        storageDeviceIsWorking = flashfsIsSupported();
        if (storageDeviceIsWorking) {
            tfp_sprintf(cmsx_BlackboxStatus, "READY");

            const flashPartition_t *flashPartition = flashPartitionFindByType(FLASH_PARTITION_TYPE_FLASHFS);
            const flashGeometry_t *flashGeometry = flashGetGeometry();

            storageUsed = flashfsGetOffset() / 1024;
            storageFree = ((FLASH_PARTITION_SECTOR_COUNT(flashPartition) * flashGeometry->sectorSize) / 1024) - storageUsed;
        } else {
            tfp_sprintf(cmsx_BlackboxStatus, "FAULT");
        }

        break;
#endif

    default:
        tfp_sprintf(cmsx_BlackboxStatus, "---");
    }

    /* Storage counters */
    tfp_sprintf(cmsx_BlackboxDeviceStorageUsed, "%ld%s", storageUsed, unit);
    tfp_sprintf(cmsx_BlackboxDeviceStorageFree, "%ld%s", storageFree, unit);
}

#ifdef USE_FLASHFS
static const void *cmsx_EraseFlash(displayPort_t *pDisplay, const void *ptr)
{
    UNUSED(ptr);

    if (!flashfsIsSupported()) {
        return NULL;
    }

    displayClearScreen(pDisplay, DISPLAY_CLEAR_WAIT);
    displayWrite(pDisplay, 5, 3, DISPLAYPORT_ATTR_INFO, "ERASING FLASH...");
    displayRedraw(pDisplay);

    flashfsEraseCompletely();
    while (!flashfsIsReady()) {
        //TODO: Make this non-blocking!
        delay(100);
    }

    beeper(BEEPER_BLACKBOX_ERASE);
    displayClearScreen(pDisplay, DISPLAY_CLEAR_WAIT);
    displayRedraw(pDisplay);

    // Update storage device status to show new used space amount
    cmsx_Blackbox_GetDeviceStatus();

    return MENU_CHAIN_BACK;
}
#endif // USE_FLASHFS

static const void *cmsx_Blackbox_onEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    cmsx_Blackbox_GetDeviceStatus();
    cmsx_BlackboxDevice = blackboxConfig()->device;
    cmsx_BlackboxRate = blackboxConfig()->sample_rate;
    systemConfig_debug_mode = systemConfig()->debug_mode;
    
    const uint16_t pidFreq = (uint16_t)pidGetPidFrequency();
    if (pidFreq > 1000) {
        tfp_sprintf(cmsx_pidFreq, "%1d.%02dKHZ", (pidFreq / 10) / 100, (pidFreq / 10) % 100);
    } else {
        tfp_sprintf(cmsx_pidFreq, "%3dHZ", pidFreq);
    }
    return NULL;
}

static const void *cmsx_Blackbox_onExit(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    if (blackboxMayEditConfig()) {
        blackboxConfigMutable()->device = cmsx_BlackboxDevice;
        blackboxValidateConfig();
    }
    blackboxConfigMutable()->sample_rate = cmsx_BlackboxRate;
    systemConfigMutable()->debug_mode = systemConfig_debug_mode;

    return NULL;
}

// Check before erase flash
#ifdef USE_FLASHFS
static const OSD_Entry menuEraseFlashCheckEntries[] = {
    { "CONFIRM ERASE", OME_Label, NULL, NULL, 0},
    { "YES",           OME_Funcall, cmsx_EraseFlash, NULL,                                                    0 },

    { "NO",            OME_Back, NULL, NULL, 0 },
    { NULL,            OME_END, NULL, NULL, 0 }
};

static CMS_Menu cmsx_menuEraseFlashCheck = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "MENUERASEFLASH",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = NULL,
    .onExit = NULL,
    .onDisplayUpdate = NULL,
    .entries = menuEraseFlashCheckEntries
};
#endif

static const OSD_Entry cmsx_menuBlackboxEntries[] =
{
    { "-- BLACKBOX --", OME_Label, NULL, NULL, 0},
    { "(PID FREQ)",  OME_String,  NULL,            &cmsx_pidFreq,                                             0 },
    { "SAMPLERATE",  OME_TAB,     NULL,            &cmsx_BlackboxRateTable,                                   REBOOT_REQUIRED },
    { "DEVICE",      OME_TAB,     NULL,            &cmsx_BlackboxDeviceTable,                                 REBOOT_REQUIRED },
    { "(STATUS)",    OME_String,  NULL,            &cmsx_BlackboxStatus,                                      0 },
    { "(USED)",      OME_String,  NULL,            &cmsx_BlackboxDeviceStorageUsed,                           0 },
    { "(FREE)",      OME_String,  NULL,            &cmsx_BlackboxDeviceStorageFree,                           0 },
    { "DEBUG MODE",  OME_TAB,     NULL,            &(OSD_TAB_t)   { &systemConfig_debug_mode, DEBUG_COUNT - 1, debugModeNames }, REBOOT_REQUIRED },

#ifdef USE_FLASHFS
    { "ERASE FLASH", OME_Submenu, cmsMenuChange,   &cmsx_menuEraseFlashCheck,                                 0 },
#endif // USE_FLASHFS

    { "BACK", OME_Back, NULL, NULL, 0 },
    { NULL, OME_END, NULL, NULL, 0 }
};

CMS_Menu cmsx_menuBlackbox = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "MENUBB",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_Blackbox_onEnter,
    .onExit = cmsx_Blackbox_onExit,
    .onDisplayUpdate = NULL,
    .entries = cmsx_menuBlackboxEntries
};

#endif
