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
// Menu element types
// XXX Upon separation, all OME would be renamed to CME_ or similar.
//

#pragma once

//type of elements

typedef enum
{
    OME_Label,
    OME_Back,
    OME_OSD_Exit,
    OME_Submenu,
    OME_Funcall,
    OME_Bool,
    OME_INT8,
    OME_UINT8,
    OME_UINT8_SCALED,   // 0-255 scaled by base10 exponent that ranges from -3 to 2. So exponent=1 means 0-2550; exponent=-1 means 0-25.5
    OME_UINT16,
    OME_INT16,
    OME_UINT32,
    OME_INT32,
    OME_String,
    OME_FLOAT, //only up to 255 value and cant be 2.55 or 25.5, just for PID's
    //wlasciwosci elementow
#ifdef USE_OSD
    OME_VISIBLE,
#endif
    OME_TAB,
    OME_END,

    // Debug aid
    OME_MENU,

    OME_MAX = OME_MENU
} OSD_MenuElement;

typedef const void *(*CMSEntryFuncPtr)(displayPort_t *displayPort, const void *ptr);

typedef struct
{
    const char * text;
    OSD_MenuElement type;
    CMSEntryFuncPtr func;
    void *data;
    uint8_t flags;
} __attribute__((packed)) OSD_Entry;

// Bits in flags
#define PRINT_VALUE    0x01  // Value has been changed, need to redraw
#define PRINT_LABEL    0x02  // Text label should be printed
#define DYNAMIC        0x04  // Value should be updated dynamically
#define OPTSTRING      0x08  // (Temporary) Flag for OME_Submenu, indicating func should be called to get a string to display.
#define REBOOT_REQUIRED 0x10 // Reboot is required if the value is changed
#define SCROLLING_TICKER 0x20// Long values are displayed as horizontally scrolling tickers (OME_TAB only)

#define IS_PRINTVALUE(x) ((x) & PRINT_VALUE)
#define SET_PRINTVALUE(x) do { (x) |= PRINT_VALUE; } while (0)
#define CLR_PRINTVALUE(x) do { (x) &= ~PRINT_VALUE; } while (0)

#define IS_PRINTLABEL(x) ((x) & PRINT_LABEL)
#define SET_PRINTLABEL(x) do { (x) |= PRINT_LABEL; } while (0)
#define CLR_PRINTLABEL(x) do { (x) &= ~PRINT_LABEL; } while (0)

#define IS_DYNAMIC(p) ((p)->flags & DYNAMIC)

#define IS_SCROLLINGTICKER(x) ((x) & SCROLLING_TICKER)
#define SET_SCROLLINGTICKER(x) do { (x) |= SCROLLING_TICKER; } while (0)
#define CLR_SCROLLINGTICKER(x) do { (x) &= ~SCROLLING_TICKER; } while (0)

typedef const void *(*CMSMenuFuncPtr)(displayPort_t *pDisp);

// Special return value(s) for function chaining by CMSMenuFuncPtr
extern int menuChainBack;
#define MENU_CHAIN_BACK  (&menuChainBack) // Causes automatic cmsMenuBack

/*
onExit function is called with self:
(1) Pointer to an OSD_Entry when cmsMenuBack() was called.
    Point to an OSD_Entry with type == OME_Back if BACK was selected.
(2) NULL if called from menu exit (forced exit at top level).
*/

typedef const void *(*CMSMenuOnExitPtr)(displayPort_t *pDisp, const OSD_Entry *self);

typedef const void *(*CMSMenuOnDisplayUpdatePtr)(displayPort_t *pDisp, const OSD_Entry *selected);

typedef struct
{
#ifdef CMS_MENU_DEBUG
    // These two are debug aids for menu content creators.
    const char *GUARD_text;
    const OSD_MenuElement GUARD_type;
#endif
    const CMSMenuFuncPtr onEnter;
    const CMSMenuOnExitPtr onExit;
    const CMSMenuOnDisplayUpdatePtr onDisplayUpdate;
    const OSD_Entry *entries;
} CMS_Menu;

typedef struct
{
    uint8_t *val;
    uint8_t min;
    uint8_t max;
    uint8_t step;
} OSD_UINT8_t;

typedef struct
{
    uint8_t *val;
    uint8_t min;
    uint8_t max;
    uint8_t step;
    int8_t exponent;
} OSD_UINT8_SCALED_t;

typedef struct
{
    int8_t *val;
    int8_t min;
    int8_t max;
    int8_t step;
} OSD_INT8_t;

typedef struct
{
    int16_t *val;
    int16_t min;
    int16_t max;
    int16_t step;
} OSD_INT16_t;

typedef struct
{
    uint16_t *val;
    uint16_t min;
    uint16_t max;
    uint16_t step;
} OSD_UINT16_t;

typedef struct
{
    int32_t *val;
    int32_t min;
    int32_t max;
    int32_t step;
} OSD_INT32_t;

typedef struct
{
    uint32_t *val;
    uint32_t min;
    uint32_t max;
    uint32_t step;
} OSD_UINT32_t;

typedef struct
{
    uint8_t *val;
    uint8_t min;
    uint8_t max;
    uint8_t step;
    uint16_t multipler;
} OSD_FLOAT_t;

typedef struct
{
    uint8_t *val;
    uint8_t max;
    const char * const *names;
} OSD_TAB_t;

typedef struct
{
    char *val;
} OSD_String_t;
