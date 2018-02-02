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

#define MAX_PINIO 4

#define PINIO_CONFIG_OUT_INVERTED 0x80
#define PINIO_CONFIG_MASK         0x7F
#define PINIO_CONFIG_OUT_PP       0x01

struct pinioConfig_s;

void pinioInit(const struct pinioConfig_s *pinioConfig);
void pinioSet(int index, bool on);
