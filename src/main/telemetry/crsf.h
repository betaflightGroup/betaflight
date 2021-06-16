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

#include "common/time.h"

#include "rx/crsf_protocol.h"

#define CRSF_MSP_RX_BUF_SIZE 128
#define CRSF_MSP_TX_BUF_SIZE 128

void initCrsfTelemetry(void);
uint32_t getCrsfDesiredSpeed(void);
void setCrsfDefaultSpeed(void);
bool checkCrsfTelemetryState(void);
void handleCrsfTelemetry(timeUs_t currentTimeUs);
void crsfScheduleDeviceInfoResponse(void);
void crsfScheduleMspResponse(void);
int getCrsfFrame(uint8_t *frame, crsfFrameType_e frameType);
void crsfProcessCommand(uint8_t *frameStart);
#if defined(USE_CRSF_CMS_TELEMETRY)
void crsfProcessDisplayPortCmd(uint8_t *frameStart);
#endif
#if defined(USE_MSP_OVER_TELEMETRY)
void initCrsfMspBuffer(void);
bool bufferCrsfMspFrame(uint8_t *frameStart, int frameLength);
#endif
#if defined(USE_CRSF_V3)
void speedNegotiationProcess(uint32_t currentTime);
#endif
