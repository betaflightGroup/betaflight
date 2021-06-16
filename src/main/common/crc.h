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

struct sbuf_s;

uint16_t crc16_ccitt(uint16_t crc, unsigned char a);
uint16_t crc16_ccitt_update(uint16_t crc, const void *data, uint32_t length);
struct sbuf_s;
void crc16_ccitt_sbuf_append(struct sbuf_s *dst, uint8_t *start);

uint8_t crc8_calc(uint8_t crc, unsigned char a, uint8_t poly);
uint8_t crc8_update(uint8_t crc, const void *data, uint32_t length, uint8_t poly);
void crc8_sbuf_append(struct sbuf_s *dst, uint8_t *start, uint8_t poly);
#define crc8_dvb_s2(crc, a)                         crc8_calc(crc, a, 0xD5)
#define crc8_dvb_s2_update(crc, data, length)       crc8_update(crc, data, length, 0xD5)
#define crc8_dvb_s2_sbuf_append(dst, start)         crc8_sbuf_append(dst, start, 0xD5)
#define crc8_poly_0xba(crc, a)                      crc8_calc(crc, a, 0xBA)
#define crc8_poly_0xba_sbuf_append(dst, start)      crc8_sbuf_append(dst, start, 0xBA)

uint8_t crc8_xor_update(uint8_t crc, const void *data, uint32_t length);
void crc8_xor_sbuf_append(struct sbuf_s *dst, uint8_t *start);
