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

#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <string.h>

#include "platform.h"

#include "common/utils.h"
#include "common/maths.h"
#include "common/axis.h"
#include "common/sensor_alignment.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "drivers/sensor.h"

#include "boardalignment.h"

static bool standardBoardAlignment = true;     // board orientation correction
static fp_rotationMatrix_t boardRotation;

// no template required since defaults are zero
PG_REGISTER(boardAlignment_t, boardAlignment, PG_BOARD_ALIGNMENT, 0);

static bool isBoardAlignmentStandard(const boardAlignment_t *boardAlignment)
{
    return !boardAlignment->rollDegrees && !boardAlignment->pitchDegrees && !boardAlignment->yawDegrees;
}

void initBoardAlignment(const boardAlignment_t *boardAlignment)
{
    if (isBoardAlignmentStandard(boardAlignment)) {
        return;
    }

    standardBoardAlignment = false;

    fp_angles_t rotationAngles;
    rotationAngles.angles.roll  = degreesToRadians(boardAlignment->rollDegrees );
    rotationAngles.angles.pitch = degreesToRadians(boardAlignment->pitchDegrees);
    rotationAngles.angles.yaw   = degreesToRadians(boardAlignment->yawDegrees  );

    buildRotationMatrix(&rotationAngles, &boardRotation);
}

static void alignBoard(float *vec)
{
    applyMatrixRotation(vec, &boardRotation);
}

FAST_CODE_NOINLINE void alignSensorViaMatrix(float *dest, fp_rotationMatrix_t* sensorRotationMatrix)
{
    applyMatrixRotation(dest, sensorRotationMatrix);

    if (!standardBoardAlignment) {
        alignBoard(dest);
    }
}

FAST_CODE void alignSensorViaRotation(float *dest, uint8_t rotation)
{
    const float x = dest[X];
    const float y = dest[Y];
    const float z = dest[Z];

    switch (rotation) {
    default:
    case CW0_DEG:
        dest[X] = x;
        dest[Y] = y;
        dest[Z] = z;
        break;
    case CW90_DEG:
        dest[X] = y;
        dest[Y] = -x;
        dest[Z] = z;
        break;
    case CW180_DEG:
        dest[X] = -x;
        dest[Y] = -y;
        dest[Z] = z;
        break;
    case CW270_DEG:
        dest[X] = -y;
        dest[Y] = x;
        dest[Z] = z;
        break;
    case CW0_DEG_FLIP:
        dest[X] = -x;
        dest[Y] = y;
        dest[Z] = -z;
        break;
    case CW90_DEG_FLIP:
        dest[X] = y;
        dest[Y] = x;
        dest[Z] = -z;
        break;
    case CW180_DEG_FLIP:
        dest[X] = x;
        dest[Y] = -y;
        dest[Z] = -z;
        break;
    case CW270_DEG_FLIP:
        dest[X] = -y;
        dest[Y] = -x;
        dest[Z] = -z;
        break;
    }

    if (!standardBoardAlignment) {
        alignBoard(dest);
    }
}
