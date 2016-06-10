/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */
 
#pragma once

#define SERIAL_RX
#define USE_CLI

#if (FLASH_SIZE > 64)
#define BLACKBOX
#define GPS
#define TELEMETRY
#define USE_SERVOS
#endif

#if (FLASH_SIZE > 128)
#define DISPLAY
#else
#define SKIP_CLI_COMMAND_HELP
#define SKIP_RX_MSP
#endif
