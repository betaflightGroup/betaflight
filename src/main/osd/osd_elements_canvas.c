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

/*
 * Author: Dominic Clifton - Sync generation, Sync Detection, Video Overlay and first-cut of working Pixel OSD system.
 */
#include "stdbool.h"
#include "math.h"

#include "platform.h"

#ifdef USE_CANVAS

#include "common/maths.h"

#include "drivers/display_canvas.h"

#include "osd/osd_elements_canvas.h"


#define PITCH_STEP       10
void simple_artificial_horizon(displayCanvas_t *canvas, int16_t roll, int16_t pitch, int16_t x, int16_t y,
        int16_t width, int16_t height, int8_t max_pitch, uint8_t n_pitch_steps)
{
    width /= 2;
    height /= 2;

    float sin_roll = sinf(DECIDEGREES_TO_RADIANS(roll));
    float cos_roll = cosf(DECIDEGREES_TO_RADIANS(roll));

    int pitch_step_offset = pitch / (PITCH_STEP * 10);

    /* how many degrees the "lines" are offset from their ideal pos
         * since we need both, don't do fmodf.. */
    float modulo_pitch =DECIDEGREES_TO_DEGREES(pitch) - pitch_step_offset * 10.0f;

    // roll to pitch transformation
    int16_t pp_x = x + width * ((sin_roll * modulo_pitch) / (float)max_pitch);
    int16_t pp_y = y + height * ((cos_roll * modulo_pitch) / (float)max_pitch);

    int16_t d_x, d_x2; // delta x
    int16_t d_y, d_y2; // delta y

    d_x = cos_roll * width / 2;
    d_y = sin_roll * height / 2;

    d_x = 3 * d_x / 4;
    d_y = 3 * d_y / 4;
    d_x2 = 3 * d_x / 4;
    d_y2 = 3 * d_y / 4;

    int16_t d_x_10 = width * sin_roll * PITCH_STEP / (float)max_pitch;
    int16_t d_y_10 = height * cos_roll * PITCH_STEP / (float)max_pitch;

    int16_t d_x_2 = d_x_10 / 6;
    int16_t d_y_2 = d_y_10 / 6;

    for (int i = (-max_pitch / 10)-1; i<(max_pitch/10)+1; i++) {
        int angle = (pitch_step_offset + i);

        if (angle < -n_pitch_steps) continue;
        if (angle > n_pitch_steps) continue;

        angle *= PITCH_STEP;

        /* Wraparound */
        if (angle > 90) {
            angle = 180 - angle;
        } else if (angle < -90) {
            angle = -180 - angle;
        }

        int16_t pp_x2 = pp_x - i * d_x_10;
        int16_t pp_y2 = pp_y - i * d_y_10;

        displayCanvasSetStrokeColor(canvas, DISPLAY_CANVAS_COLOR_WHITE);

        if (angle < 0) {
            displayCanvasMoveToPoint(canvas, pp_x2 - d_x2, pp_y2 + d_y2);
            displayCanvasStrokeLineToPoint(canvas, pp_x2 + d_x2, pp_y2 - d_y2);

            displayCanvasMoveToPoint(canvas, pp_x2 - d_x2, pp_y2 + d_y2);
            displayCanvasStrokeLineToPoint(canvas, pp_x2 - d_x2 - d_x_2, pp_y2 + d_y2 - d_y_2);

            displayCanvasMoveToPoint(canvas, pp_x2 + d_x2, pp_y2 - d_y2);
            displayCanvasStrokeLineToPoint(canvas, pp_x2 + d_x2 - d_x_2, pp_y2 - d_y2 - d_y_2);

            //write_string(tmp_str, pp_x2 - d_x - 4, pp_y2 + d_y, 0, 0, TEXT_VA_MIDDLE, TEXT_HA_CENTER, FONT_OUTLINED8X8);
            //write_string(tmp_str, pp_x2 + d_x + 4, pp_y2 - d_y, 0, 0, TEXT_VA_MIDDLE, TEXT_HA_CENTER, FONT_OUTLINED8X8);
        } else if (angle > 0) {
            displayCanvasMoveToPoint(canvas, pp_x2 - d_x2, pp_y2 + d_y2);
            displayCanvasStrokeLineToPoint(canvas, pp_x2 + d_x2, pp_y2 - d_y2);

            displayCanvasMoveToPoint(canvas, pp_x2 - d_x2, pp_y2 + d_y2);
            displayCanvasStrokeLineToPoint(canvas, pp_x2 - d_x2 + d_x_2, pp_y2 + d_y2 + d_y_2);

            displayCanvasMoveToPoint(canvas, pp_x2 + d_x2, pp_y2 - d_y2);
            displayCanvasStrokeLineToPoint(canvas, pp_x2 + d_x2 + d_x_2, pp_y2 - d_y2 + d_y_2);

            //write_string(tmp_str, pp_x2 - d_x - 4, pp_y2 + d_y, 0, 0, TEXT_VA_MIDDLE, TEXT_HA_CENTER, FONT_OUTLINED8X8);
            //write_string(tmp_str, pp_x2 + d_x + 4, pp_y2 - d_y, 0, 0, TEXT_VA_MIDDLE, TEXT_HA_CENTER, FONT_OUTLINED8X8);
        } else {
            displayCanvasMoveToPoint(canvas, pp_x2 - d_x, pp_y2 + d_y);
            displayCanvasStrokeLineToPoint(canvas, pp_x2 - d_x / 3, pp_y2 + d_y / 3);

            displayCanvasMoveToPoint(canvas, pp_x2 + d_x / 3, pp_y2 - d_y / 3);
            displayCanvasStrokeLineToPoint(canvas, pp_x2 + d_x, pp_y2 - d_y);
        }
    }
}

#endif
