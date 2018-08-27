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

#include <stdbool.h>
#include <stdint.h>

// Streams data out to the EEPROM, padding to the write size as
// needed, and updating the checksum as it goes.

typedef struct config_streamer_s {
    uintptr_t address;
    int size;
    union {
        uint8_t b[4];
        uint32_t w;
    } buffer;
    int at;
    int err;
    bool unlocked;
} config_streamer_t;

void config_streamer_init(config_streamer_t *c);

void config_streamer_start(config_streamer_t *c, uintptr_t base, int size);
int config_streamer_write(config_streamer_t *c, const uint8_t *p, uint32_t size);
int config_streamer_flush(config_streamer_t *c);

int config_streamer_finish(config_streamer_t *c);
int config_streamer_status(config_streamer_t *c);
