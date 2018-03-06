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

#ifdef BEEPER
#define BEEP_TOGGLE              systemBeepToggle()
#define BEEP_OFF                 systemBeep(false)
#ifdef USE_DSHOT
#define BEEP_ON                  systemBeepDshot()
#else  // USE_DSHOT
#define BEEP_ON                  systemBeep(true)
#endif // USE_DSHOT
#else  // BEEPER
#define BEEP_TOGGLE do {} while (0)
#define BEEP_OFF    do {} while (0)
#define BEEP_ON     do {} while (0)
#endif // BEEPER

void systemBeepDshot(void);
void systemBeep(bool on);
void systemBeepToggle(void);
struct beeperDevConfig_s;
void beeperInit(const struct beeperDevConfig_s *beeperDevConfig);
