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

#include "drivers/io_types.h"

#include "pg/pg.h"

#define GET_FRAME_ERR_LPF_FREQUENCY(period) (1 / (period / 10.0f))
#define FRAME_ERR_RESAMPLE_US 100000

typedef struct rxConfig_s {
    uint8_t rcmap[RX_MAPPABLE_CHANNEL_COUNT];  // mapping of radio channels to internal RPYTA+ order
    uint8_t serialrx_provider;                 // type of UART-based receiver (0 = spek 10, 1 = spek 11, 2 = sbus). Must be enabled by FEATURE_RX_SERIAL first.
    uint8_t serialrx_inverted;                 // invert the serial RX protocol compared to it's default setting
    uint8_t halfDuplex;                        // allow rx to operate in half duplex mode on F4, ignored for F1 and F3.
    ioTag_t spektrum_bind_pin_override_ioTag;
    ioTag_t spektrum_bind_plug_ioTag;
    uint8_t spektrum_sat_bind;                 // number of bind pulses for Spektrum satellite receivers
    uint8_t spektrum_sat_bind_autoreset;       // whenever we will reset (exit) binding mode after hard reboot
    uint8_t rssi_channel;
    uint8_t rssi_scale;
    uint8_t rssi_invert;
    uint16_t midrc;                            // Some radios have not a neutral point centered on 1500. can be changed here
    uint16_t mincheck;                         // minimum rc end
    uint16_t maxcheck;                         // maximum rc end
    uint8_t fpvCamAngleDegrees;                // Camera angle to be scaled into rc commands
    uint8_t airModeActivateThreshold;          // Throttle setpoint percent where airmode gets activated
    uint16_t rx_min_usec;
    uint16_t rx_max_usec;
    uint8_t max_aux_channel;
    uint8_t rssi_src_frame_errors;             // true to use frame drop flags in the rx protocol
    int8_t rssi_offset;                        // offset applied to the RSSI value before it is returned
    uint8_t rc_smoothing_mode;                 // Whether filter based rc smoothing is on or off
    uint8_t rc_smoothing_setpoint_cutoff;      // Filter cutoff frequency for the setpoint filter (0 = auto)
    uint8_t rc_smoothing_feedforward_cutoff;   // Filter cutoff frequency for the feedforward filter (0 = auto)
    uint8_t rc_smoothing_throttle_cutoff;      // Filter cutoff frequency for the setpoint filter (0 = auto)
    uint8_t rc_smoothing_debug_axis;           // Axis to log as debug values when debug_mode = RC_SMOOTHING
    uint8_t rc_smoothing_auto_factor_rpy;      // Used to adjust the "smoothness" determined by the auto cutoff calculations
    uint8_t rc_smoothing_auto_factor_throttle; // Used to adjust the "smoothness" determined by the auto cutoff calculations
    uint8_t rssi_src_frame_lpf_period;         // Period of the cutoff frequency for the source frame RSSI filter (in 0.1 s)
    uint8_t srxl2_unit_id;                     // Spektrum SRXL2 RX unit id
    uint8_t srxl2_baud_fast;                   // Select Spektrum SRXL2 fast baud rate
    uint8_t sbus_baud_fast;                    // Select SBus fast baud rate
    uint8_t crsf_use_rx_snr;                   // Use RX SNR (in dB) instead of RSSI dBm for CRSF
    uint32_t msp_override_channels_mask;       // Channels to override when the MSP override mode is enabled
} rxConfig_t;

PG_DECLARE(rxConfig_t, rxConfig);
