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

#include <stdint.h>
#include <stdbool.h>
#include "platform.h"

#include "config_helper.h"
#include "io/serial.h"
#include "pg/piniobox.h"
#include "pg/sdcard.h"
#include "pg/motor.h"
#include "target.h"
#include "config/config.h"
#include "drivers/pwm_output.h"

#define  USE_TARGET_CONFIG

void targetConfiguration(void)
{
    pinioBoxConfigMutable()->permanentId[0] = 40;
    pinioBoxConfigMutable()->permanentId[1] = 41;

    sdcardConfigMutable()->mode = SDCARD_MODE_SDIO;
}

