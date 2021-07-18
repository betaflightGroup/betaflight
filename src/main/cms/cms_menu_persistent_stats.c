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

#ifdef USE_CMS
#ifdef USE_PERSISTENT_STATS

#include "cms/cms.h"
#include "cms/cms_types.h"
#include "cms/cms_menu_persistent_stats.h"

#include "config/config.h"
#include "pg/stats.h"

uint32_t stats_total_flights;
uint32_t stats_total_time_s;
uint32_t stats_total_dist_m;
int8_t stats_min_armed_time_s;

static const void *cmsx_PersistentStats_onEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    stats_total_flights = statsConfig()->stats_total_flights;
    stats_total_time_s = statsConfig()->stats_total_time_s;
    stats_total_dist_m = statsConfig()->stats_total_dist_m;
    stats_min_armed_time_s = statsConfig()->stats_min_armed_time_s;

    return NULL;
}

static const void *cmsx_PersistentStats_onExit(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    statsConfigMutable()->stats_total_flights = stats_total_flights;
    statsConfigMutable()->stats_total_time_s = stats_total_time_s;
    statsConfigMutable()->stats_total_dist_m = stats_total_dist_m;
    statsConfigMutable()->stats_min_armed_time_s = stats_min_armed_time_s;

    return NULL;
}

static const void *cmsx_ResetStats(displayPort_t *pDisplay, const void *ptr)
{
    UNUSED(ptr);

    stats_total_flights = 0;
    stats_total_time_s = 0;
    stats_total_dist_m = 0;

    displayClearScreen(pDisplay, DISPLAY_CLEAR_WAIT);
    displayRedraw(pDisplay);

    return NULL;
}

static const OSD_Entry cmsx_menuPersistentStatsEntries[] =
{
    {"-- PERSISTENT STATS --", OME_Label, NULL, NULL, 0},
    {"FLIGHTS", OME_UINT32, NULL, &(OSD_UINT32_t){ &stats_total_flights, 0, UINT32_MAX, 1}, 0},
    {"TIME(sec)", OME_UINT32, NULL, &(OSD_UINT32_t){ &stats_total_time_s, 0, UINT32_MAX, 1}, 0},
    {"DIST(m)", OME_UINT32, NULL, &(OSD_UINT32_t){ &stats_total_dist_m, 0, UINT32_MAX, 1}, 0},
    {"RESET STATS", OME_Funcall, cmsx_ResetStats, NULL, 0},
    {"--- SETTINGS ---", OME_Label, NULL, NULL, 0},
    {"MIN ARMED TIME(sec)", OME_INT8, NULL, &(OSD_INT8_t){ &stats_min_armed_time_s, -1, INT8_MAX, 1}, 0},

    {"BACK", OME_Back, NULL, NULL, 0},
    { NULL, OME_END, NULL, NULL, 0}
};

CMS_Menu cmsx_menuPersistentStats = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "PRESSTATS",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_PersistentStats_onEnter,
    .onExit = cmsx_PersistentStats_onExit,
    .onDisplayUpdate = NULL,
    .entries = cmsx_menuPersistentStatsEntries
};

#endif
#endif
