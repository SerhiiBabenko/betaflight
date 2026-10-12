/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that they will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * trick_replay: feeds recorded samples through the trick detector core (host build).
 *
 * Usage: trick_replay [hover_throttle loop_share loop_min_s recover_s] < samples.txt
 * Input, one sample per line (whitespace separated):
 *   dt gx gy gz ax ay az sx sy sz throttle has_throttle armed acro vetoed
 * Output, one line per finished trick:
 *   t_end trick count landed roll_right share duration roll pitch yaw text
 */

#include <stdio.h>
#include <stdlib.h>

#include "flight/trick_detector.h"

static const char *const trickNames[] = { "none", "unmatched", "front_flip", "back_flip", "roll", "power_loop" };

int main(int argc, char **argv)
{
    trickDetectorParams_t params = { 0.0f, 0.25f, 1.9f, 0.5f };
    if (argc == 5) {
        params.hoverThrottle = strtof(argv[1], NULL);
        params.loopShare = strtof(argv[2], NULL);
        params.loopMinS = strtof(argv[3], NULL);
        params.recoverS = strtof(argv[4], NULL);
    }
    static trickDetector_t det;
    trickDetectorInit(&det, &params);

    trickSample_t s;
    int hasThr, armed, acro, vetoed;
    char line[512];
    while (fgets(line, sizeof(line), stdin)) {
        if (sscanf(line, "%f %f %f %f %f %f %f %f %f %f %f %d %d %d %d",
                   &s.dtS, &s.gyro[0], &s.gyro[1], &s.gyro[2], &s.acc[0], &s.acc[1], &s.acc[2],
                   &s.setpoint[0], &s.setpoint[1], &s.setpoint[2], &s.throttle,
                   &hasThr, &armed, &acro, &vetoed) != 15) {
            continue;
        }
        s.hasThrottle = hasThr;
        s.armed = armed;
        s.acro = acro;
        s.vetoed = vetoed;
        trickEvent_t ev;
        if (trickDetectorStep(&det, &s, &ev)) {
            char text[32];
            trickFormatAnnouncement(&ev, text, sizeof(text));
            printf("%.4f %s %d %d %d %.3f %.3f %.1f %.1f %.1f %s\n", ev.tEndS, trickNames[ev.trick], ev.count,
                   ev.landed, ev.rollRight, ev.share, ev.durationS, ev.angles[0], ev.angles[1], ev.angles[2],
                   text[0] ? text : "-");
        }
    }
    return 0;
}
