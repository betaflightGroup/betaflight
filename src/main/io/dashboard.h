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

#pragma once

#include "common/time.h"
#include "pg/pg.h"
#include "drivers/bus_i2c.h"

#define ENABLE_DEBUG_DASHBOARD_PAGE

#if !defined(DASHBOARD_I2C_INSTANCE)
#if defined(I2C_DEVICE)
#define DASHBOARD_I2C_INSTANCE I2C_DEVICE
#else
#define DASHBOARD_I2C_INSTANCE I2C_NONE
#endif
#endif

#define DASHBOARD_I2C_ADDRESS   0x3C     // OLED at address 0x3C in 7bit

typedef enum {
    PAGE_WELCOME,
    PAGE_ARMED,
    PAGE_BATTERY,
    PAGE_SENSORS,
    PAGE_RX,
    PAGE_PROFILE,
    PAGE_RPROF,
    PAGE_TASKS,
#ifdef USE_GPS
    PAGE_GPS,
#endif
#ifdef ENABLE_DEBUG_DASHBOARD_PAGE
    PAGE_DEBUG,
#endif
#ifdef USE_BLACKBOX
    PAGE_BB,
#endif
    PAGE_COUNT
} pageId_e;

void dashboardInit(void);
void dashboardUpdate(timeUs_t currentTimeUs);

void dashboardShowFixedPage(pageId_e pageId);

void dashboardEnablePageCycling(void);
void dashboardDisablePageCycling(void);
void dashboardResetPageCycling(void);
void dashboardSetNextPageChangeAt(timeUs_t futureMicros);
