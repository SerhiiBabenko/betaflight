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
 * Trick detector glue: reads Betaflight signals, runs the pure core
 * (trick_detector.c) and holds the current OSD announcement.
 */

#include <math.h>

#include "platform.h"

#ifdef USE_TRICK_DETECTOR

#include "build/debug.h"

#include "common/maths.h"

#include "fc/core.h"
#include "fc/rc.h"
#include "fc/runtime_config.h"

#include "flight/failsafe.h"
#include "flight/mixer.h"
#include "flight/trick_detector.h"

#include "pg/trick_detector.h"

#include "sensors/acceleration.h"
#include "sensors/gyro.h"
#include "sensors/sensors.h"

#include "trick_detector_task.h"

#define NOT_ACRO_MODES (ANGLE_MODE | HORIZON_MODE | ALT_HOLD_MODE | POS_HOLD_MODE | FAILSAFE_MODE | \
                        GPS_RESCUE_MODE | AUTOPILOT_MODE | PASSTHRU_MODE | CHIRP_MODE)

static trickDetector_t detector;
static bool initialised;
static timeUs_t lastUpdateUs;

static char announcement[32];
static timeUs_t announcementUntilUs;

static void initDetector(void)
{
    const trickDetectorConfig_t *cfg = trickDetectorConfig();
    const trickDetectorParams_t params = {
        .hoverThrottle = cfg->hover_throttle / 100.0f,
        .loopShare = cfg->loop_share / 100.0f,
        .loopMinS = cfg->loop_min_ms / 1000.0f,
        .recoverS = cfg->recover_ms / 1000.0f,
        .level = cfg->level,
    };
    trickDetectorInit(&detector, &params);
    initialised = true;
}

void trickDetectorUpdate(timeUs_t currentTimeUs)
{
    if (!initialised) {
        initDetector();
        lastUpdateUs = currentTimeUs;
        return;
    }
    const timeDelta_t dtUs = cmpTimeUs(currentTimeUs, lastUpdateUs);
    lastUpdateUs = currentTimeUs;

    trickSample_t s;
    for (int axis = 0; axis < 3; axis++) {
        s.gyro[axis] = gyroGetFilteredDownsampled(axis);
        s.setpoint[axis] = getSetpointRate(axis);
        s.acc[axis] = sensors(SENSOR_ACC) ? acc.accADC.v[axis] * acc.dev.acc_1G_rec : 0.0f;
    }
    s.throttle = mixerGetThrottle();
    s.hasThrottle = true;
    s.dtS = dtUs * 1e-6f;
    s.armed = ARMING_FLAG(ARMED);
    s.acro = !FLIGHT_MODE(NOT_ACRO_MODES);
    s.vetoed = isCrashFlipModeActive() || failsafeIsActive() || gyroOverflowDetected() || gyroYawSpinDetected();

    trickEvent_t ev;
    if (trickDetectorStep(&detector, &s, &ev)) {
        if (trickFormatAnnouncement(&ev, announcement, sizeof(announcement)) > 0) {
            announcementUntilUs = currentTimeUs + trickDetectorConfig()->announce_ms * 1000;
        }
        DEBUG_SET(DEBUG_TRICK, 4, lrintf(ev.share * 100.0f));
        DEBUG_SET(DEBUG_TRICK, 6, ev.trick);
        DEBUG_SET(DEBUG_TRICK, 7, ev.landed);
    }

    const int main = fabsf(detector.ang[1]) >= fabsf(detector.ang[0]) ? 1 : 0;
    DEBUG_SET(DEBUG_TRICK, 0, detector.out + 2 * detector.sawInverted + 4 * detector.pending);
    DEBUG_SET(DEBUG_TRICK, 1, lrintf(acosf(detector.up[2]) * (180.0f / M_PIf)));
    DEBUG_SET(DEBUG_TRICK, 2, lrintf(detector.ang[main]));
    DEBUG_SET(DEBUG_TRICK, 3, lrintf(detector.ang[1 - main]));
    DEBUG_SET(DEBUG_TRICK, 5, lrintf(detector.hover * 1000.0f));
}

const char *trickDetectorAnnouncement(timeUs_t currentTimeUs)
{
    if (!announcement[0] || cmpTimeUs(announcementUntilUs, currentTimeUs) <= 0) {
        return NULL;
    }
    return announcement;
}

#endif // USE_TRICK_DETECTOR
