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
 Original OSD code created by Marcin Baliniak
 OSD-CMS separation by jflyper
 CMS-displayPort separation by jflyper and martinbudden
 */

//#define CMS_PAGE_DEBUG // For multi-page/menu debugging
//#define CMS_MENU_DEBUG // For external menu content creators

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>

#include "platform.h"

#ifdef USE_CMS

#include "build/build_config.h"
#include "build/debug.h"
#include "build/version.h"

#include "cms/cms.h"
#include "cms/cms_menu_main.h"
#include "cms/cms_menu_saveexit.h"
#include "cms/cms_types.h"

#include "common/maths.h"
#include "common/typeconversion.h"

#include "config/config.h"
#include "config/feature.h"

#include "drivers/motor.h"
#include "drivers/osd_symbols.h"
#include "drivers/system.h"
#include "drivers/time.h"

#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/mixer.h"

#include "io/rcdevice_cam.h"
#include "io/usb_cdc_hid.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/rx.h"

#include "osd/osd.h"

#include "rx/rx.h"

// DisplayPort management

#ifndef CMS_MAX_DEVICE
#define CMS_MAX_DEVICE 4
#endif

#define CMS_MENU_STACK_LIMIT 10

displayPort_t *pCurrentDisplay;

static displayPort_t *cmsDisplayPorts[CMS_MAX_DEVICE];
static unsigned cmsDeviceCount;
static int cmsCurrentDevice = -1;
#ifdef USE_OSD
static unsigned int osdProfileCursor = 1;
#endif

int menuChainBack;

bool cmsDisplayPortRegister(displayPort_t *pDisplay)
{
    if (!pDisplay || cmsDeviceCount >= CMS_MAX_DEVICE) {
        return false;
    }

    cmsDisplayPorts[cmsDeviceCount++] = pDisplay;

    return true;
}

static displayPort_t *cmsDisplayPortSelectCurrent(void)
{
    if (cmsDeviceCount == 0) {
        return NULL;
    }

    if (cmsCurrentDevice < 0) {
        cmsCurrentDevice = 0;
    }

    return cmsDisplayPorts[cmsCurrentDevice];
}

static displayPort_t *cmsDisplayPortSelectNext(void)
{
    if (cmsDeviceCount == 0) {
        return NULL;
    }

    cmsCurrentDevice = (cmsCurrentDevice + 1) % cmsDeviceCount; // -1 Okay

    return cmsDisplayPorts[cmsCurrentDevice];
}

bool cmsDisplayPortSelect(displayPort_t *instance)
{
    for (unsigned i = 0; i < cmsDeviceCount; i++) {
        if (cmsDisplayPortSelectNext() == instance) {
            return true;
        }
    }
    return false;
}

#define CMS_POLL_INTERVAL_US   100000   // Interval of polling dynamic values (microsec)

// XXX LEFT_MENU_COLUMN and RIGHT_MENU_COLUMN must be adjusted
// dynamically depending on size of the active output device,
// or statically to accomodate sizes of all supported devices.
//
// Device characteristics
// OLED
//   21 cols x 8 rows
//     128x64 with 5x7 (6x8) : 21 cols x 8 rows
// MAX7456 (PAL)
//   30 cols x 16 rows
// MAX7456 (NTSC)
//   30 cols x 13 rows
// HoTT Telemetry Screen
//   21 cols x 8 rows
//
// Spektrum SRXL Telemtry Textgenerator
//   13 cols x 9 rows, top row printed as a Bold Heading
//   Needs the "smallScreen" adaptions

#define CMS_MAX_ROWS 16

#define NORMAL_SCREEN_MIN_COLS 18      // Less is a small screen
static bool    smallScreen;
static uint8_t leftMenuColumn;
static uint8_t rightMenuColumn;
static uint8_t maxMenuItems;
static uint8_t linesPerMenuItem;
static cms_key_e externKey = CMS_KEY_NONE;
static bool osdElementEditing = false;

bool cmsInMenu = false;

typedef struct cmsCtx_s {
    const CMS_Menu *menu;         // menu for this context
    uint8_t page;                 // page in the menu
    int8_t cursorRow;             // cursorRow in the page
} cmsCtx_t;

static cmsCtx_t menuStack[CMS_MENU_STACK_LIMIT];
static uint8_t menuStackIdx = 0;

static int8_t pageCount;         // Number of pages in the current menu
static const OSD_Entry *pageTop;       // First entry for the current page
static uint8_t pageMaxRow;       // Max row in the current page

static cmsCtx_t currentCtx;

static bool saveMenuInhibited = false;

#ifdef CMS_MENU_DEBUG // For external menu content creators

static char menuErrLabel[21 + 1] = "RANDOM DATA";

static OSD_Entry menuErrEntries[] = {
    { "BROKEN MENU", OME_Label, NULL, NULL, 0 },
    { menuErrLabel, OME_Label, NULL, NULL, 0 },
    { "BACK", OME_Back, NULL, NULL, 0 },
    { NULL, OME_END, NULL, NULL, 0 }
};

static CMS_Menu menuErr = {
    "MENUERR",
    OME_MENU,
    NULL,
    NULL,
    NULL,
    menuErrEntries,
};
#endif

#ifdef CMS_PAGE_DEBUG
#define cmsPageDebug() { \
    debug[0] = pageCount; \
    debug[1] = currentCtx.page; \
    debug[2] = pageMaxRow; \
    debug[3] = currentCtx.cursorRow; } struct _dummy
#endif

static void cmsUpdateMaxRow(displayPort_t *instance)
{
    UNUSED(instance);
    pageMaxRow = 0;

    for (const OSD_Entry *ptr = pageTop; ptr->type != OME_END; ptr++) {
        pageMaxRow++;
    }

    if (pageMaxRow > maxMenuItems) {
        pageMaxRow = maxMenuItems;
    }

    if (pageMaxRow > CMS_MAX_ROWS) {
        pageMaxRow = CMS_MAX_ROWS;
    }

    pageMaxRow--;
}

static uint8_t cmsCursorAbsolute(displayPort_t *instance)
{
    UNUSED(instance);
    return currentCtx.cursorRow + currentCtx.page * maxMenuItems;
}

uint8_t runtimeEntryFlags[CMS_MAX_ROWS] = { 0 };

#define LOOKUP_TABLE_TICKER_START_CYCLES 20   // Task loops for start/end of ticker (1 second delay)
#define LOOKUP_TABLE_TICKER_SCROLL_CYCLES 3   // Task loops for each scrolling step of the ticker (150ms delay)

typedef struct cmsTableTicker_s {
    uint8_t loopCounter;
    uint8_t state;
} cmsTableTicker_t;

cmsTableTicker_t runtimeTableTicker[CMS_MAX_ROWS];

static void cmsPageSelect(displayPort_t *instance, int8_t newpage)
{
    currentCtx.page = (newpage + pageCount) % pageCount;
    pageTop = &currentCtx.menu->entries[currentCtx.page * maxMenuItems];
    cmsUpdateMaxRow(instance);

    const OSD_Entry *p;
    int i;
    for (p = pageTop, i = 0; (p <= pageTop + pageMaxRow); p++, i++) {
        runtimeEntryFlags[i] = p->flags;
    }
    displayClearScreen(instance, DISPLAY_CLEAR_WAIT);
}

static void cmsPageNext(displayPort_t *instance)
{
    cmsPageSelect(instance, currentCtx.page + 1);
}

static void cmsPagePrev(displayPort_t *instance)
{
    cmsPageSelect(instance, currentCtx.page - 1);
}

static void cmsFormatFloat(int32_t value, char *floatString)
{
    uint8_t k;
    // np. 3450

    itoa(100000 + value, floatString, 10); // Create string from abs of integer value

    // 103450

    floatString[0] = floatString[1];
    floatString[1] = floatString[2];
    floatString[2] = '.';

    // 03.450
    // usuwam koncowe zera i kropke
    // Keep the first decimal place
    for (k = 5; k > 3; k--) {
        if (floatString[k] == '0' || floatString[k] == '.') {
            floatString[k] = 0;
        } else {
            break;
        }
    }

    // oraz zero wiodonce
    if (floatString[0] == '0') {
        floatString[0] = ' ';
    }
}

// CMS on OSD legacy was to use LEFT aligned values, not the RIGHT way ;-)
#define CMS_OSD_RIGHT_ALIGNED_VALUES

#ifndef CMS_OSD_RIGHT_ALIGNED_VALUES

// Pad buffer to the left, i.e. align left
static void cmsPadRightToSize(char *buf, int size)
{
    int i;

    for (i = 0 ; i < size ; i++) {
        if (buf[i] == 0) {
            break;
        }
    }

    for ( ; i < size ; i++) {
        buf[i] = ' ';
    }

    buf[size] = 0;
}
#endif

// Pad buffer to the left, i.e. align right
static void cmsPadLeftToSize(char *buf, int size)
{
    int i,j;
    int len = strlen(buf);

    for (i = size - 1, j = size - len ; i - j >= 0 ; i--) {
      buf[i] = buf[i - j];
    }

    for ( ; i >= 0 ; i--) {
      buf[i] = ' ';
    }

    buf[size] = 0;
}

static void cmsPadToSize(char *buf, int size)
{
    // Make absolutely sure the string terminated.
    buf[size] = 0x00,

#ifdef CMS_OSD_RIGHT_ALIGNED_VALUES
    cmsPadLeftToSize(buf, size);
#else
    smallScreen ? cmsPadLeftToSize(buf, size) : cmsPadRightToSize(buf, size);
#endif
}

static int cmsDisplayWrite(displayPort_t *instance, uint8_t x, uint8_t y, uint8_t attr, const char *s)
{
    char buffer[strlen(s) + 1];
    char* b = buffer;
    while (*s) {
        char c = toupper(*s++);
        *b++ = (c < 0x20 || c > 0x5F) ? ' ' : c; // limit to alphanumeric and punctuation
    }
    *b++ = '\0';

    return displayWrite(instance, x, y, attr, buffer);
}

static int cmsDrawMenuItemValue(displayPort_t *pDisplay, char *buff, uint8_t row, uint8_t maxSize)
{
    int colpos;
    int cnt;

    cmsPadToSize(buff, maxSize);
#ifdef CMS_OSD_RIGHT_ALIGNED_VALUES
    colpos = rightMenuColumn - maxSize;
#else
    colpos = smallScreen ? rightMenuColumn - maxSize : rightMenuColumn;
#endif
    cnt = cmsDisplayWrite(pDisplay, colpos, row, DISPLAYPORT_ATTR_NONE, buff);
    return cnt;
}

static int cmsDrawMenuEntry(displayPort_t *pDisplay, const OSD_Entry *p, uint8_t row, bool selectedRow, uint8_t *flags, cmsTableTicker_t *ticker)
{
    #define CMS_DRAW_BUFFER_LEN 12
    #define CMS_TABLE_VALUE_MAX_LEN 30
    #define CMS_NUM_FIELD_LEN 5
    #define CMS_CURSOR_BLINK_DELAY_MS 500

    char buff[CMS_DRAW_BUFFER_LEN +1]; // Make room for null terminator.
    char tableBuff[CMS_TABLE_VALUE_MAX_LEN +1];
    int cnt = 0;

#ifndef USE_OSD
    UNUSED(selectedRow);
#endif

    if (smallScreen) {
        row++;
    }

    switch (p->type) {
    case OME_String:
        if (IS_PRINTVALUE(*flags) && p->data) {
            strncpy(buff, p->data, CMS_DRAW_BUFFER_LEN);
            cnt = cmsDrawMenuItemValue(pDisplay, buff, row, CMS_DRAW_BUFFER_LEN);
            CLR_PRINTVALUE(*flags);
        }
        break;

    case OME_Submenu:
    case OME_Funcall:
        if (IS_PRINTVALUE(*flags)) {
            buff[0]= 0x0;

            if (p->type == OME_Submenu && p->func && *flags & OPTSTRING) {

                // Special case of sub menu entry with optional value display.

                const char *str = p->func(pDisplay, p->data);
                strncpy(buff, str, CMS_DRAW_BUFFER_LEN);
            } else if (p->type == OME_Funcall && p->data) {
                strncpy(buff, p->data, CMS_DRAW_BUFFER_LEN);
            }
            strncat(buff, ">", CMS_DRAW_BUFFER_LEN);

            row = smallScreen ? row - 1 : row;
            cnt = cmsDrawMenuItemValue(pDisplay, buff, row, strlen(buff));
            CLR_PRINTVALUE(*flags);
        }
        break;

    case OME_Bool:
        if (IS_PRINTVALUE(*flags) && p->data) {
            if (*((uint8_t *)(p->data))) {
              strcpy(buff, "YES");
            } else {
              strcpy(buff, "NO ");
            }

            cnt = cmsDrawMenuItemValue(pDisplay, buff, row, 3);
            CLR_PRINTVALUE(*flags);
        }
        break;

    case OME_TAB:
        if (IS_PRINTVALUE(*flags) || IS_SCROLLINGTICKER(*flags)) {
            bool drawText = false;
            OSD_TAB_t *ptr = p->data;
            const int labelLength = strlen(p->text) + 1; // account for the space between label and display data
            char *str = (char *)ptr->names[*ptr->val];   // lookup table display text
            const int displayLength = strlen(str);

            // Calculate the available space to display the lookup table entry based on the
            // screen size and the length of the label. Always display at least CMS_DRAW_BUFFER_LEN
            // characters to prevent really long labels from overriding the data display.
            const int availableSpace = MAX(CMS_DRAW_BUFFER_LEN, rightMenuColumn - labelLength - leftMenuColumn - 1);

            if (IS_PRINTVALUE(*flags)) {
                drawText = true;
                ticker->state = 0;
                ticker->loopCounter = 0;
                if (displayLength > availableSpace) {  // table entry text is longer than the available space so start the ticker
                    SET_SCROLLINGTICKER(*flags);
                } else {
                    CLR_SCROLLINGTICKER(*flags);
                }
            } else if (IS_SCROLLINGTICKER(*flags)) {
                ticker->loopCounter++;
                const uint8_t loopLimit = (ticker->state == 0 || ticker->state == (displayLength - availableSpace)) ? LOOKUP_TABLE_TICKER_START_CYCLES : LOOKUP_TABLE_TICKER_SCROLL_CYCLES;
                if (ticker->loopCounter >= loopLimit) {
                    ticker->loopCounter = 0;
                    drawText = true;
                    ticker->state++;
                    if (ticker->state > (displayLength - availableSpace)) {
                        ticker->state = 0;
                    }
                }
            }
            if (drawText) {
                strncpy(tableBuff, (char *)(str + ticker->state), CMS_TABLE_VALUE_MAX_LEN);
                cnt = cmsDrawMenuItemValue(pDisplay, tableBuff, row, availableSpace);
            }
            CLR_PRINTVALUE(*flags);
        }
        break;

#ifdef USE_OSD
    case OME_VISIBLE:
        if (IS_PRINTVALUE(*flags) && p->data) {
            uint16_t *val = (uint16_t *)p->data;
            bool cursorBlink = millis() % (2 * CMS_CURSOR_BLINK_DELAY_MS) < CMS_CURSOR_BLINK_DELAY_MS;
            for (unsigned x = 1; x < OSD_PROFILE_COUNT + 1; x++) {
                if (VISIBLE_IN_OSD_PROFILE(*val, x)) {
                    if (osdElementEditing && cursorBlink && selectedRow && (x == osdProfileCursor)) {
                        strcpy(buff + x - 1, " ");
                    } else {
                        strcpy(buff + x - 1, "X");
                    }
                } else {
                    if (osdElementEditing && cursorBlink && selectedRow && (x == osdProfileCursor)) {
                        strcpy(buff + x - 1, " ");
                    } else {
                        strcpy(buff + x - 1, "-");
                    }
                }
            }
            cnt = cmsDrawMenuItemValue(pDisplay, buff, row, 3);
            CLR_PRINTVALUE(*flags);
        }
        break;
#endif

    case OME_UINT8:
        if (IS_PRINTVALUE(*flags) && p->data) {
            OSD_UINT8_t *ptr = p->data;
            itoa(*ptr->val, buff, 10);
            cnt = cmsDrawMenuItemValue(pDisplay, buff, row, CMS_NUM_FIELD_LEN);
            CLR_PRINTVALUE(*flags);
        }
        break;

    case OME_INT8:
        if (IS_PRINTVALUE(*flags) && p->data) {
            OSD_INT8_t *ptr = p->data;
            itoa(*ptr->val, buff, 10);
            cnt = cmsDrawMenuItemValue(pDisplay, buff, row, CMS_NUM_FIELD_LEN);
            CLR_PRINTVALUE(*flags);
        }
        break;

    case OME_UINT16:
        if (IS_PRINTVALUE(*flags) && p->data) {
            OSD_UINT16_t *ptr = p->data;
            itoa(*ptr->val, buff, 10);
            cnt = cmsDrawMenuItemValue(pDisplay, buff, row, CMS_NUM_FIELD_LEN);
            CLR_PRINTVALUE(*flags);
        }
        break;

    case OME_INT16:
        if (IS_PRINTVALUE(*flags) && p->data) {
            OSD_INT16_t *ptr = p->data;
            itoa(*ptr->val, buff, 10);
            cnt = cmsDrawMenuItemValue(pDisplay, buff, row, CMS_NUM_FIELD_LEN);
            CLR_PRINTVALUE(*flags);
        }
        break;

    case OME_UINT32:
        if (IS_PRINTVALUE(*flags) && p->data) {
            OSD_UINT32_t *ptr = p->data;
            itoa(*ptr->val, buff, 10);
            cnt = cmsDrawMenuItemValue(pDisplay, buff, row, CMS_NUM_FIELD_LEN);
            CLR_PRINTVALUE(*flags);
        }
        break;

    case OME_INT32:
        if (IS_PRINTVALUE(*flags) && p->data) {
            OSD_INT32_t *ptr = p->data;
            itoa(*ptr->val, buff, 10);
            cnt = cmsDrawMenuItemValue(pDisplay, buff, row, CMS_NUM_FIELD_LEN);
            CLR_PRINTVALUE(*flags);
        }
        break;

    case OME_FLOAT:
        if (IS_PRINTVALUE(*flags) && p->data) {
            OSD_FLOAT_t *ptr = p->data;
            cmsFormatFloat(*ptr->val * ptr->multipler, buff);
            cnt = cmsDrawMenuItemValue(pDisplay, buff, row, CMS_NUM_FIELD_LEN);
            CLR_PRINTVALUE(*flags);
        }
        break;

    case OME_Label:
        if (IS_PRINTVALUE(*flags) && p->data) {
            // A label with optional string, immediately following text
            cnt = cmsDisplayWrite(pDisplay, leftMenuColumn + 1 + (uint8_t)strlen(p->text), row, DISPLAYPORT_ATTR_NONE, p->data);
            CLR_PRINTVALUE(*flags);
        }
        break;

    case OME_OSD_Exit:
    case OME_END:
    case OME_Back:
        break;

    case OME_MENU:
        // Fall through
    default:
#ifdef CMS_MENU_DEBUG
        // Shouldn't happen. Notify creator of this menu content
#ifdef CMS_OSD_RIGHT_ALIGNED_VALUES
        cnt = cmsDisplayWrite(pDisplay, rightMenuColumn - 6, row, DISPLAYPORT_ATTR_NONE, "BADENT");
#else
        cnt = cmsDisplayWrite(pDisplay, rightMenuColumn, row, DISPLAYPORT_ATTR_NONE, "BADENT");
#endif
#endif
        break;
    }

    return cnt;
}

static void cmsMenuCountPage(displayPort_t *pDisplay)
{
    UNUSED(pDisplay);
    const OSD_Entry *p;
    for (p = currentCtx.menu->entries; p->type != OME_END; p++);
    pageCount = (p - currentCtx.menu->entries - 1) / maxMenuItems + 1;
}

STATIC_UNIT_TESTED const void *cmsMenuBack(displayPort_t *pDisplay)
{
    // Let onExit function decide whether to allow exit or not.
    if (currentCtx.menu->onExit) {
        const void *result = currentCtx.menu->onExit(pDisplay, pageTop + currentCtx.cursorRow);
        if (result == MENU_CHAIN_BACK) {
            return result;
        }
    }

    saveMenuInhibited = false;

    if (!menuStackIdx) {
        return NULL;
    }

    currentCtx = menuStack[--menuStackIdx];

    cmsMenuCountPage(pDisplay);
    cmsPageSelect(pDisplay, currentCtx.page);

#if defined(CMS_PAGE_DEBUG)
    cmsPageDebug();
#endif

    return NULL;
}

// Skip read-only entries
static bool rowIsSkippable(const OSD_Entry *row)
{
    if (row->type == OME_Label) {
        return true;
    }

    if (row->type == OME_String) {
        return true;
    }
	
    if ((row->type == OME_UINT16 || row->type == OME_INT16) && row->flags == DYNAMIC) {
        return true;
    }
    return false;
}

static void cmsDrawMenu(displayPort_t *pDisplay, uint32_t currentTimeUs)
{
    if (!pageTop || !cmsInMenu) {
        return;
    }

    if (pDisplay->frameBufferCount > 1) {
        // With 2 or more frame buffers the display would flicker between a drawn-to framebuffer and a previous framebuffer.
        displayClearScreen(pDisplay, DISPLAY_CLEAR_WAIT);
    }

    const bool displayWasCleared = pDisplay->cleared;
    uint8_t i;
    const OSD_Entry *p;
    uint8_t top = smallScreen ? 1 : (pDisplay->rows - pageMaxRow)/2;

    pDisplay->cleared = false;

    // Polled (dynamic) value display denominator.

    bool drawPolled = false;
    static uint32_t lastPolledUs = 0;

    if (currentTimeUs > lastPolledUs + CMS_POLL_INTERVAL_US) {
        drawPolled = true;
        lastPolledUs = currentTimeUs;
    }

    uint32_t room = displayTxBytesFree(pDisplay);

    if (displayWasCleared) {
        for (p = pageTop, i= 0; (p <= pageTop + pageMaxRow); p++, i++) {
            SET_PRINTLABEL(runtimeEntryFlags[i]);
            SET_PRINTVALUE(runtimeEntryFlags[i]);
        }
    } else if (drawPolled) {
        for (p = pageTop, i = 0; (p <= pageTop + pageMaxRow); p++, i++) {
            if (IS_DYNAMIC(p))
                SET_PRINTVALUE(runtimeEntryFlags[i]);
        }
    }

    // Cursor manipulation

    while (rowIsSkippable(pageTop + currentCtx.cursorRow)) { // skip labels, strings and dynamic read-only entries
        currentCtx.cursorRow++;
    }

#if defined(CMS_PAGE_DEBUG)
    cmsPageDebug();
#endif

    if (pDisplay->cursorRow >= 0 && currentCtx.cursorRow != pDisplay->cursorRow) {
        room -= cmsDisplayWrite(pDisplay, leftMenuColumn, top + pDisplay->cursorRow * linesPerMenuItem, DISPLAYPORT_ATTR_NONE, " ");
    }

    if (room < 30) {
        return;
    }

    if (pDisplay->cursorRow != currentCtx.cursorRow) {
        room -= cmsDisplayWrite(pDisplay, leftMenuColumn, top + currentCtx.cursorRow * linesPerMenuItem, DISPLAYPORT_ATTR_NONE, ">");
        pDisplay->cursorRow = currentCtx.cursorRow;
    }

    if (room < 30) {
        return;
    }

    if (currentCtx.menu->onDisplayUpdate) {
        const void *result = currentCtx.menu->onDisplayUpdate(pDisplay, pageTop + currentCtx.cursorRow);
        if (result == MENU_CHAIN_BACK) {
            cmsMenuBack(pDisplay);

            return;
        }
    }

    // Print text labels
    for (i = 0, p = pageTop; (p <= pageTop + pageMaxRow); i++, p++) {
        if (IS_PRINTLABEL(runtimeEntryFlags[i])) {
            uint8_t coloff = leftMenuColumn;
            coloff += (p->type == OME_Label) ? 0 : 1;
            room -= cmsDisplayWrite(pDisplay, coloff, top + i * linesPerMenuItem, DISPLAYPORT_ATTR_NONE, p->text);
            CLR_PRINTLABEL(runtimeEntryFlags[i]);
            if (room < 30) {
                return;
            }
        }

    // Print values

    // XXX Polled values at latter positions in the list may not be
    // XXX printed if not enough room in the middle of the list.

        if (IS_PRINTVALUE(runtimeEntryFlags[i]) || IS_SCROLLINGTICKER(runtimeEntryFlags[i])) {
            bool selectedRow = i == currentCtx.cursorRow;
            room -= cmsDrawMenuEntry(pDisplay, p, top + i * linesPerMenuItem, selectedRow, &runtimeEntryFlags[i], &runtimeTableTicker[i]);
            if (room < 30) {
                return;
            }
        }
    }

    // Draw the up/down page indicators if the display has space.
    // Only draw the symbols when necessary after the screen has been cleared. Otherwise they're static.
    // If the device supports OSD symbols then use the up/down arrows. Otherwise assume it's a
    // simple text device and use the '^' (carat) and 'V' for arrow approximations.
    if (displayWasCleared && leftMenuColumn > 0) {      // make sure there's room to draw the symbol
        if (currentCtx.page > 0) {
            const uint8_t symbol = displaySupportsOsdSymbols(pDisplay) ? SYM_ARROW_NORTH : '^';
            displayWriteChar(pDisplay, leftMenuColumn - 1, top, DISPLAYPORT_ATTR_NONE, symbol);
        }
         if (currentCtx.page < pageCount - 1) {
            const uint8_t symbol = displaySupportsOsdSymbols(pDisplay) ? SYM_ARROW_SOUTH : 'V';
            displayWriteChar(pDisplay, leftMenuColumn - 1, top + pageMaxRow, DISPLAYPORT_ATTR_NONE, symbol);
        }
    }

}

const void *cmsMenuChange(displayPort_t *pDisplay, const void *ptr)
{
    const CMS_Menu *pMenu = (const CMS_Menu *)ptr;

    if (!pMenu) {
        return NULL;
    }

#ifdef CMS_MENU_DEBUG
    if (pMenu->GUARD_type != OME_MENU) {
        // ptr isn't pointing to a CMS_Menu.
        if (pMenu->GUARD_type <= OME_MAX) {
            strncpy(menuErrLabel, pMenu->GUARD_text, sizeof(menuErrLabel) - 1);
        } else {
            strncpy(menuErrLabel, "LABEL UNKNOWN", sizeof(menuErrLabel) - 1);
        }
        pMenu = &menuErr;
    }
#endif

    if (pMenu != currentCtx.menu) {
        saveMenuInhibited = false;

        if (currentCtx.menu) {
            // If we are opening the initial top-level menu, then currentCtx.menu will be NULL and nothing to do.
            // Otherwise stack the current menu before moving to the selected menu.
            if (menuStackIdx >= CMS_MENU_STACK_LIMIT - 1) {
                // menu stack limit reached - prevent array overflow
                return NULL;
            }
            menuStack[menuStackIdx++] = currentCtx;
        }

        currentCtx.menu = pMenu;
        currentCtx.cursorRow = 0;

        if (pMenu->onEnter) {
            const void *result = pMenu->onEnter(pDisplay);
            if (result == MENU_CHAIN_BACK) {
                return cmsMenuBack(pDisplay);
            }
        }

        cmsMenuCountPage(pDisplay);
        cmsPageSelect(pDisplay, 0);
    } else {
        // The (pMenu == curretMenu) case occurs when reopening for display cycling
        // currentCtx.cursorRow has been saved as absolute; convert it back to page + relative

        int8_t cursorAbs = currentCtx.cursorRow;
        currentCtx.cursorRow = cursorAbs % maxMenuItems;
        cmsMenuCountPage(pDisplay);
        cmsPageSelect(pDisplay, cursorAbs / maxMenuItems);
    }

#if defined(CMS_PAGE_DEBUG)
    cmsPageDebug();
#endif

    return NULL;
}

void cmsMenuOpen(void)
{
    const CMS_Menu *startMenu;
    if (!cmsInMenu) {
        // New open
        pCurrentDisplay = cmsDisplayPortSelectCurrent();
        if (!pCurrentDisplay) {
            return;
        }
        cmsInMenu = true;
        currentCtx = (cmsCtx_t){ NULL, 0, 0 };
        startMenu = &cmsx_menuMain;
        menuStackIdx = 0;
        setArmingDisabled(ARMING_DISABLED_CMS_MENU);
        displayLayerSelect(pCurrentDisplay, DISPLAYPORT_LAYER_FOREGROUND); // make sure the foreground layer is active
        if (osdConfig()->cms_background_type != DISPLAY_BACKGROUND_TRANSPARENT) {
            displaySetBackgroundType(pCurrentDisplay, (displayPortBackground_e)osdConfig()->cms_background_type); // set the background type if not transparent
        }
    } else {
        // Switch display
        displayPort_t *pNextDisplay = cmsDisplayPortSelectNext();
        startMenu = currentCtx.menu;
        if (pNextDisplay != pCurrentDisplay) {
            // DisplayPort has been changed.
            // Convert cursorRow to absolute value
            currentCtx.cursorRow = cmsCursorAbsolute(pCurrentDisplay);
            displaySetBackgroundType(pCurrentDisplay, DISPLAY_BACKGROUND_TRANSPARENT); // reset previous displayPort to transparent
            displayRelease(pCurrentDisplay);
            pCurrentDisplay = pNextDisplay;
            displaySetBackgroundType(pCurrentDisplay, (displayPortBackground_e)osdConfig()->cms_background_type); // set the background type if not transparent
        } else {
            return;
        }
    }
    displayGrab(pCurrentDisplay); // grab the display for use by the CMS
    // FIXME this should probably not have a dependency on the OSD or OSD slave code
#ifdef USE_OSD
    resumeRefreshAt = 0;
#endif

    if ( pCurrentDisplay->cols < NORMAL_SCREEN_MIN_COLS) {
      smallScreen       = true;
      linesPerMenuItem  = 2;
      leftMenuColumn    = 0;
      rightMenuColumn   = pCurrentDisplay->cols;
      maxMenuItems      = (pCurrentDisplay->rows) / linesPerMenuItem;
    } else {
      smallScreen       = false;
      linesPerMenuItem  = 1;
      leftMenuColumn    = 2;
#ifdef CMS_OSD_RIGHT_ALIGNED_VALUES
      rightMenuColumn   = pCurrentDisplay->cols - 2;
#else
      rightMenuColumn   = pCurrentDisplay->cols - CMS_DRAW_BUFFER_LEN;
#endif
      maxMenuItems      = pCurrentDisplay->rows - 2;
    }

    if (pCurrentDisplay->useFullscreen) {
    	leftMenuColumn = 0;
    	rightMenuColumn   = pCurrentDisplay->cols;
    	maxMenuItems      = pCurrentDisplay->rows;
    }

    cmsMenuChange(pCurrentDisplay, startMenu);
}

static void cmsTraverseGlobalExit(const CMS_Menu *pMenu)
{
    for (const OSD_Entry *p = pMenu->entries; p->type != OME_END ; p++) {
        if (p->type == OME_Submenu) {
            cmsTraverseGlobalExit(p->data);
        }
    }

}

const void *cmsMenuExit(displayPort_t *pDisplay, const void *ptr)
{
    int exitType = (int)ptr;
    switch (exitType) {
    case CMS_EXIT_SAVE:
    case CMS_EXIT_SAVEREBOOT:
    case CMS_POPUP_SAVE:
    case CMS_POPUP_SAVEREBOOT:

        cmsTraverseGlobalExit(&cmsx_menuMain);

        if (currentCtx.menu->onExit) {
            currentCtx.menu->onExit(pDisplay, (OSD_Entry *)NULL); // Forced exit
        }

        if ((exitType == CMS_POPUP_SAVE) || (exitType == CMS_POPUP_SAVEREBOOT)) {
            // traverse through the menu stack and call their onExit functions
            for (int i = menuStackIdx - 1; i >= 0; i--) {
                if (menuStack[i].menu->onExit) {
                    menuStack[i].menu->onExit(pDisplay, (OSD_Entry *)NULL);
                }
            }
        }

        saveConfigAndNotify();
        break;

    case CMS_EXIT:
        break;
    }

    cmsInMenu = false;

    displaySetBackgroundType(pCurrentDisplay, DISPLAY_BACKGROUND_TRANSPARENT); // reset the background to transparent

    displayRelease(pDisplay);
    currentCtx.menu = NULL;

    if ((exitType == CMS_EXIT_SAVEREBOOT) || (exitType == CMS_POPUP_SAVEREBOOT) || (exitType == CMS_POPUP_EXITREBOOT)) {
        displayClearScreen(pDisplay, DISPLAY_CLEAR_WAIT);
        cmsDisplayWrite(pDisplay, 5, 3, DISPLAYPORT_ATTR_NONE, "REBOOTING...");

        // Flush display
        displayRedraw(pDisplay);

        stopMotors();
        motorShutdown();
        delay(200);

        systemReset();
    }

    unsetArmingDisabled(ARMING_DISABLED_CMS_MENU);

    return NULL;
}

// Stick/key detection and key codes

#define IS_HI(X)  (rcData[X] > 1750)
#define IS_LO(X)  (rcData[X] < 1250)
#define IS_MID(X) (rcData[X] > 1250 && rcData[X] < 1750)

#define BUTTON_TIME   250 // msec
#define BUTTON_PAUSE  500 // msec

STATIC_UNIT_TESTED uint16_t cmsHandleKey(displayPort_t *pDisplay, cms_key_e key)
{
    uint16_t res = BUTTON_TIME;
    const OSD_Entry *p;

    if (!currentCtx.menu) {
        return res;
    }

    if (key == CMS_KEY_MENU) {
        cmsMenuOpen();
        return BUTTON_PAUSE;
    }

    if (key == CMS_KEY_ESC) {
        if (osdElementEditing) {
            osdElementEditing = false;
        } else {
            cmsMenuBack(pDisplay);
        }
        return BUTTON_PAUSE;
    }

    if (key == CMS_KEY_SAVEMENU && !saveMenuInhibited) {
        osdElementEditing = false;
        cmsMenuChange(pDisplay, getSaveExitMenu());

        return BUTTON_PAUSE;
    }

    if ((key == CMS_KEY_DOWN) && (!osdElementEditing)) {
        if (currentCtx.cursorRow < pageMaxRow) {
            currentCtx.cursorRow++;
        } else {
            cmsPageNext(pDisplay);
            currentCtx.cursorRow = 0;    // Goto top in any case
        }
    }

    if ((key == CMS_KEY_UP) && (!osdElementEditing)) {
        currentCtx.cursorRow--;

        // Skip non-title labels, strings and dynamic read-only entries
        while ((rowIsSkippable(pageTop + currentCtx.cursorRow)) && currentCtx.cursorRow > 0) {
            currentCtx.cursorRow--;
        }
        if (currentCtx.cursorRow == -1 || (pageTop + currentCtx.cursorRow)->type == OME_Label) {
            // Goto previous page
            cmsPagePrev(pDisplay);
            currentCtx.cursorRow = pageMaxRow;
        }
    }

    if ((key == CMS_KEY_DOWN || key == CMS_KEY_UP) && (!osdElementEditing)) {
        return res;
    }

    p = pageTop + currentCtx.cursorRow;

    switch (p->type) {
        case OME_Submenu:
            if (key == CMS_KEY_RIGHT) {
                cmsMenuChange(pDisplay, p->data);
                res = BUTTON_PAUSE;
            }
            break;

        case OME_Funcall:;
            const void *retval;
            if (p->func && key == CMS_KEY_RIGHT) {
                retval = p->func(pDisplay, p->data);
                if (retval == MENU_CHAIN_BACK) {
                    cmsMenuBack(pDisplay);
                }
                if ((p->flags & REBOOT_REQUIRED)) {
                    setRebootRequired();
                }
                res = BUTTON_PAUSE;
            }
            break;

        case OME_OSD_Exit:
            if (p->func && key == CMS_KEY_RIGHT) {
                p->func(pDisplay, p->data);
                res = BUTTON_PAUSE;
            }
            break;

        case OME_Back:
            cmsMenuBack(pDisplay);
            res = BUTTON_PAUSE;
            osdElementEditing = false;
            break;

        case OME_Bool:
            if (p->data) {
                uint8_t *val = p->data;
                const uint8_t previousValue = *val;
                *val = (key == CMS_KEY_RIGHT) ? 1 : 0;
                SET_PRINTVALUE(runtimeEntryFlags[currentCtx.cursorRow]);
                if ((p->flags & REBOOT_REQUIRED) && (*val != previousValue)) {
                    setRebootRequired();
                }
                if (p->func) {
                    p->func(pDisplay, p->data);
                }
            }
            break;

#ifdef USE_OSD
        case OME_VISIBLE:
            if (p->data) {
                uint16_t *val = (uint16_t *)p->data;
                const uint16_t previousValue = *val;
                if ((key == CMS_KEY_RIGHT) && (!osdElementEditing)) {
                    osdElementEditing = true;
                    osdProfileCursor = 1;
                } else if (osdElementEditing) {
#ifdef USE_OSD_PROFILES
                    if (key == CMS_KEY_RIGHT) {
                        if (osdProfileCursor < OSD_PROFILE_COUNT) {
                            osdProfileCursor++;
                        }
                    }
                    if (key == CMS_KEY_LEFT) {
                        if (osdProfileCursor > 1) {
                            osdProfileCursor--;
                        }
                    }
#endif
                    if (key == CMS_KEY_UP) {
                        *val |= OSD_PROFILE_FLAG(osdProfileCursor);
                    }
                    if (key == CMS_KEY_DOWN) {
                        *val &= ~OSD_PROFILE_FLAG(osdProfileCursor);
                    }
                }
                SET_PRINTVALUE(runtimeEntryFlags[currentCtx.cursorRow]);
                if ((p->flags & REBOOT_REQUIRED) && (*val != previousValue)) {
                    setRebootRequired();
                }
            }
            break;
#endif

        case OME_UINT8:
        case OME_FLOAT:
            if (p->data) {
                OSD_UINT8_t *ptr = p->data;
                const uint16_t previousValue = *ptr->val;
                if (key == CMS_KEY_RIGHT) {
                    if (*ptr->val < ptr->max) {
                        *ptr->val += ptr->step;
                    }
                } else {
                    if (*ptr->val > ptr->min) {
                        *ptr->val -= ptr->step;
                    }
                }
                SET_PRINTVALUE(runtimeEntryFlags[currentCtx.cursorRow]);
                if ((p->flags & REBOOT_REQUIRED) && (*ptr->val != previousValue)) {
                    setRebootRequired();
                }
                if (p->func) {
                    p->func(pDisplay, p);
                }
            }
            break;

        case OME_TAB:
            if (p->type == OME_TAB) {
                OSD_TAB_t *ptr = p->data;
                const uint8_t previousValue = *ptr->val;

                if (key == CMS_KEY_RIGHT) {
                    if (*ptr->val < ptr->max) {
                        *ptr->val += 1;
                    }
                } else {
                    if (*ptr->val > 0) {
                        *ptr->val -= 1;
                    }
                }
                if (p->func) {
                    p->func(pDisplay, p->data);
                }
                SET_PRINTVALUE(runtimeEntryFlags[currentCtx.cursorRow]);
                if ((p->flags & REBOOT_REQUIRED) && (*ptr->val != previousValue)) {
                    setRebootRequired();
                }
            }
            break;

        case OME_INT8:
            if (p->data) {
                OSD_INT8_t *ptr = p->data;
                const int8_t previousValue = *ptr->val;
                if (key == CMS_KEY_RIGHT) {
                    if (*ptr->val < ptr->max) {
                        *ptr->val += ptr->step;
                    }
                } else {
                    if (*ptr->val > ptr->min) {
                        *ptr->val -= ptr->step;
                    }
                }
                SET_PRINTVALUE(runtimeEntryFlags[currentCtx.cursorRow]);
                if ((p->flags & REBOOT_REQUIRED) && (*ptr->val != previousValue)) {
                    setRebootRequired();
                }
                if (p->func) {
                    p->func(pDisplay, p);
                }
            }
            break;

        case OME_UINT16:
            if (p->data) {
                OSD_UINT16_t *ptr = p->data;
                const uint16_t previousValue = *ptr->val;
                if (key == CMS_KEY_RIGHT) {
                    if (*ptr->val < ptr->max) {
                        *ptr->val += ptr->step;
                    }
                } else {
                    if (*ptr->val > ptr->min) {
                        *ptr->val -= ptr->step;
                    }
                }
                SET_PRINTVALUE(runtimeEntryFlags[currentCtx.cursorRow]);
                if ((p->flags & REBOOT_REQUIRED) && (*ptr->val != previousValue)) {
                    setRebootRequired();
                }
                if (p->func) {
                    p->func(pDisplay, p);
                }
            }
            break;

        case OME_INT16:
            if (p->data) {
                OSD_INT16_t *ptr = p->data;
                const int16_t previousValue = *ptr->val;
                if (key == CMS_KEY_RIGHT) {
                    if (*ptr->val < ptr->max) {
                        *ptr->val += ptr->step;
                    }
                } else {
                    if (*ptr->val > ptr->min) {
                        *ptr->val -= ptr->step;
                    }
                }
                SET_PRINTVALUE(runtimeEntryFlags[currentCtx.cursorRow]);
                if ((p->flags & REBOOT_REQUIRED) && (*ptr->val != previousValue)) {
                    setRebootRequired();
                }
                if (p->func) {
                    p->func(pDisplay, p);
                }
            }
            break;

        case OME_UINT32:
            if (p->data) {
                OSD_UINT32_t *ptr = p->data;
                const uint32_t previousValue = *ptr->val;
                if (key == CMS_KEY_RIGHT) {
                    if (*ptr->val < ptr->max) {
                        *ptr->val += ptr->step;
                    }
                } else {
                    if (*ptr->val > ptr->min) {
                        *ptr->val -= ptr->step;
                    }
                }
                SET_PRINTVALUE(runtimeEntryFlags[currentCtx.cursorRow]);
                if ((p->flags & REBOOT_REQUIRED) && (*ptr->val != previousValue)) {
                    setRebootRequired();
                }
                if (p->func) {
                    p->func(pDisplay, p);
                }
            }
            break;

        case OME_INT32:
            if (p->data) {
                OSD_INT32_t *ptr = p->data;
                const int32_t previousValue = *ptr->val;
                if (key == CMS_KEY_RIGHT) {
                    if (*ptr->val < ptr->max) {
                        *ptr->val += ptr->step;
                    }
                } else {
                    if (*ptr->val > ptr->min) {
                        *ptr->val -= ptr->step;
                    }
                }
                SET_PRINTVALUE(runtimeEntryFlags[currentCtx.cursorRow]);
                if ((p->flags & REBOOT_REQUIRED) && (*ptr->val != previousValue)) {
                    setRebootRequired();
                }
                if (p->func) {
                    p->func(pDisplay, p);
                }
            }
            break;

        case OME_String:
            break;

        case OME_Label:
        case OME_END:
            break;

        case OME_MENU:
            // Shouldn't happen
            break;
    }
    return res;
}

void cmsSetExternKey(cms_key_e extKey)
{
    if (externKey == CMS_KEY_NONE)
        externKey = extKey;
}

uint16_t cmsHandleKeyWithRepeat(displayPort_t *pDisplay, cms_key_e key, int repeatCount)
{
    uint16_t ret = 0;

    for (int i = 0 ; i < repeatCount ; i++) {
        ret = cmsHandleKey(pDisplay, key);
    }

    return ret;
}

static uint16_t cmsScanKeys(timeMs_t currentTimeMs, timeMs_t lastCalledMs, int16_t rcDelayMs)
{
    static int holdCount = 1;
    static int repeatCount = 1;
    static int repeatBase = 0;

    //
    // Scan 'key' first
    //

    cms_key_e key = CMS_KEY_NONE;

    if (externKey != CMS_KEY_NONE) {
        rcDelayMs = cmsHandleKey(pCurrentDisplay, externKey);
        externKey = CMS_KEY_NONE;
    } else {
        if (IS_MID(THROTTLE) && IS_LO(YAW) && IS_HI(PITCH) && !ARMING_FLAG(ARMED)) {
            key = CMS_KEY_MENU;
        } else if (IS_HI(PITCH)) {
            key = CMS_KEY_UP;
        } else if (IS_LO(PITCH)) {
            key = CMS_KEY_DOWN;
        } else if (IS_LO(ROLL)) {
            key = CMS_KEY_LEFT;
        } else if (IS_HI(ROLL)) {
            key = CMS_KEY_RIGHT;
        } else if (IS_LO(YAW)) {
            key = CMS_KEY_ESC;
        } else if (IS_HI(YAW)) {
            key = CMS_KEY_SAVEMENU;
        }

        if (key == CMS_KEY_NONE) {
            // No 'key' pressed, reset repeat control
            holdCount = 1;
            repeatCount = 1;
            repeatBase = 0;
        } else {
            // The 'key' is being pressed; keep counting
            ++holdCount;
        }

        if (rcDelayMs > 0) {
            rcDelayMs -= (currentTimeMs - lastCalledMs);
        } else if (key) {
            rcDelayMs = cmsHandleKeyWithRepeat(pCurrentDisplay, key, repeatCount);

            // Key repeat effect is implemented in two phases.
            // First phldase is to decrease rcDelayMs reciprocal to hold time.
            // When rcDelayMs reached a certain limit (scheduling interval),
            // repeat rate will not raise anymore, so we call key handler
            // multiple times (repeatCount).
            //
            // XXX Caveat: Most constants are adjusted pragmatically.
            // XXX Rewrite this someday, so it uses actual hold time instead
            // of holdCount, which depends on the scheduling interval.

            if (((key == CMS_KEY_LEFT) || (key == CMS_KEY_RIGHT)) && (holdCount > 20)) {

                // Decrease rcDelayMs reciprocally

                rcDelayMs /= (holdCount - 20);

                // When we reach the scheduling limit,

                if (rcDelayMs <= 50) {

                    // start calling handler multiple times.

                    if (repeatBase == 0) {
                        repeatBase = holdCount;
                    }

                    repeatCount = repeatCount + (holdCount - repeatBase) / 5;

                    if (repeatCount > 5) {
                        repeatCount= 5;
                    }
                }
            }
        }
    }

    return rcDelayMs;
}

static void cmsUpdate(uint32_t currentTimeUs)
{
    if (IS_RC_MODE_ACTIVE(BOXPARALYZE)
#ifdef USE_RCDEVICE
        || rcdeviceInMenu
#endif
#ifdef USE_USB_CDC_HID
        || cdcDeviceIsMayBeActive() // If this target is used as a joystick, we should leave here.
#endif
       ) {
        return;
    }

    static int16_t rcDelayMs = BUTTON_TIME;

    static uint32_t lastCalledMs = 0;
    static uint32_t lastCmsHeartBeatMs = 0;

    const uint32_t currentTimeMs = currentTimeUs / 1000;

    if (!cmsInMenu) {
        // Detect menu invocation
        if (IS_MID(THROTTLE) && IS_LO(YAW) && IS_HI(PITCH) && !ARMING_FLAG(ARMED) && !IS_RC_MODE_ACTIVE(BOXSTICKCOMMANDDISABLE)) {
            cmsMenuOpen();
            rcDelayMs = BUTTON_PAUSE;    // Tends to overshoot if BUTTON_TIME
        }
    } else {
        displayBeginTransaction(pCurrentDisplay, DISPLAY_TRANSACTION_OPT_RESET_DRAWING);

        rcDelayMs = cmsScanKeys(currentTimeMs, lastCalledMs, rcDelayMs);

        cmsDrawMenu(pCurrentDisplay, currentTimeUs);

        if (currentTimeMs > lastCmsHeartBeatMs + 500) {
            // Heart beat for external CMS display device @ 500msec
            // (Timeout @ 1000msec)
            displayHeartbeat(pCurrentDisplay);
            lastCmsHeartBeatMs = currentTimeMs;
        }

        displayCommitTransaction(pCurrentDisplay);
    }

    // Some key (command), notably flash erase, takes too long to use the
    // currentTimeMs to be used as lastCalledMs (freezes CMS for a minute or so
    // if used).
    lastCalledMs = millis();
}

void cmsHandler(timeUs_t currentTimeUs)
{
    if (cmsDeviceCount > 0) {
        cmsUpdate(currentTimeUs);
    }
}

void cmsInit(void)
{
    cmsDeviceCount = 0;
    cmsCurrentDevice = -1;
}

void inhibitSaveMenu(void)
{
    saveMenuInhibited = true;
}

void cmsAddMenuEntry(OSD_Entry *menuEntry, char *text, OSD_MenuElement type, CMSEntryFuncPtr func, void *data, uint8_t flags)
{
        menuEntry->text = text;
        menuEntry->type = type;
        menuEntry->func = func;
        menuEntry->data = data;
        menuEntry->flags = flags;
}

#endif // CMS
