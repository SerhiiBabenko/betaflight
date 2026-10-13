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
 * Trick detector: pure core.
 *
 * Recognises freestyle tricks (Front/Back Flip, Roll, Power Loop) from a stream of
 * samples and reports each one once it is over. No Betaflight globals: the glue in
 * trick_detector_task.c feeds samples, so the core runs unchanged in unit tests and
 * in the offline replay tool.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum {
    TRICK_NONE = 0,
    TRICK_UNMATCHED,        // a rotation that is not a v1 trick: never announced
    TRICK_FRONT_FLIP,
    TRICK_BACK_FLIP,
    TRICK_ROLL,
    TRICK_POWER_LOOP,
} trickId_e;

typedef struct trickDetectorParams_s {
    float hoverThrottle;    // 0..1; 0 = learn in flight
    float loopShare;        // back pitch Thrust share above this is a Power Loop (0..1)
    float loopMinS;         // back pitch longer than this is a Power Loop; 0 = off
    float recoverS;         // Recovery window
} trickDetectorParams_t;

typedef struct trickSample_s {
    float gyro[3];          // deg/s, body roll/pitch/yaw; +pitch = nose down
    float acc[3];           // g, body frame (specific force)
    float setpoint[3];      // deg/s commanded
    float throttle;         // 0..1 mixer throttle
    bool hasThrottle;       // false: use body-z accel as the thrust stand-in
    float dtS;
    bool armed;
    bool acro;
    bool vetoed;            // turtle, failsafe, gyro overflow or yaw-spin recovery
} trickSample_t;

typedef struct trickEvent_s {
    trickId_e trick;
    uint8_t count;          // rotations (Front/Back Flip, Roll); 1 otherwise
    bool rollRight;
    bool landed;
    float tEndS;
    float durationS;
    float share;            // Thrust share (back pitch), for DEBUG
    float angles[3];        // integrated roll/pitch/yaw, deg
} trickEvent_t;

#define TRICK_LOOKBACK_SAMPLES 1000     // 4 s at 250 Hz
#define TRICK_WALK_SLACK       64       // extra slots: samples that arrive while the walk-back runs
#define TRICK_HIST_SLOTS       (TRICK_LOOKBACK_SAMPLES + TRICK_WALK_SLACK)
#define TRICK_SHARE_BIN_DEG    5.0f
#define TRICK_SHARE_BINS       288      // up to 1440 deg
#define TRICK_WALK_BUDGET      128      // history entries one step may walk (keeps the FC task short)

typedef struct trickHistory_s {
    int16_t rate[3];        // 0.1 deg/s
    uint16_t dtUs;
    uint8_t thrust;
} trickHistory_t;

typedef struct trickDetector_s {
    trickDetectorParams_t params;
    uint32_t tUs;           // integer clock: float accumulation drifts over a flight
    float up[3];
    float hover;
    uint32_t hoverN;

    // lookback ring buffer (only needed up to the moment the quad leaves Upright)
    trickHistory_t hist[TRICK_HIST_SLOTS];
    uint16_t histHead;
    uint16_t histCount;

    // segment since leaving Upright
    bool out;
    bool sawInverted;
    uint32_t tLeaveUs;
    float ang[3];
    uint32_t stopSinceUs;   // 0 = rotation not stopped

    // walk-back results per (axis roll/pitch) x (sign -, +), taken when leaving Upright
    float preAng[2][2][3];
    uint32_t preT0Us[2][2];
    // Thrust share histogram for back pitch (pitch, negative sign)
    float binOn[TRICK_SHARE_BINS];
    float binTot[TRICK_SHARE_BINS];
    float progress;         // |pitch| degrees turned since the back-pitch walk-back start

    // the walk-back runs over several steps, at most TRICK_WALK_BUDGET entries each
    bool walking;
    uint8_t walkCombo;      // axis * 2 + sign being walked; 4 = replaying back-pitch share
    uint16_t walkLeaveIdx;  // slot of the sample that left Upright
    uint16_t walkCount;     // usable history when it left
    uint16_t walkK;         // next entry back from walkLeaveIdx
    uint32_t walkTEntryUs;
    uint16_t shareK;        // back-pitch walk length: entries 1..shareK-1 feed the share
    // live back-pitch share samples held until the walked-back ones are in (order matters)
    float liveDeg[TRICK_WALK_SLACK];
    uint8_t liveThrust[TRICK_WALK_SLACK];
    uint8_t liveN;

    // finished trick waiting for its Recovery window
    bool pending;
    trickEvent_t pendingEvent;
    uint32_t okSinceUs;     // 0 = not in Controlled flight
    uint32_t pendingEndUs;

    uint16_t stepWork;      // history entries the last step walked (<= TRICK_WALK_BUDGET)
} trickDetector_t;

void trickDetectorInit(trickDetector_t *d, const trickDetectorParams_t *params);
// Feed one sample. Returns true and fills *out when a trick is over (landed or not).
bool trickDetectorStep(trickDetector_t *d, const trickSample_t *s, trickEvent_t *out);
// "DOUBLE BACK FLIP!" etc. Writes "" and returns 0 when there is nothing to announce.
int trickFormatAnnouncement(const trickEvent_t *ev, char *buf, size_t len);
