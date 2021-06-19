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

#include "rx/ghst_protocol.h"

#define GHST_MAX_NUM_CHANNELS           16

struct rxConfig_s;
struct rxRuntimeState_s;

void ghstRxWriteTelemetryData(const void *data, int len);
void ghstRxSendTelemetryData(const struct rxRuntimeState_s *rxRuntimeState);


bool ghstRxInit(const struct rxConfig_s *initialRxConfig, struct rxRuntimeState_s *rxRuntimeState);
