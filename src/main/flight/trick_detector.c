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
 * Trick detector pure core. See SPEC-v1 (betaflight-addons) section 2.
 *
 * Segments run Upright -> Inverted -> Upright. Rotation is integrated per axis from the
 * moment the quad leaves Upright, plus a walk-back into the history to where the
 * rotation really began (a slow Power Loop climbs for seconds before leaving Upright).
 * Thrust share is measured per degree of rotation, not per sample.
 */

#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "flight/trick_detector.h"

// compiled-in constants: they describe the tricks, not the quad
#define UPRIGHT_DEG         75.0f
#define INVERTED_DEG        105.0f
#define STILL_RATE          30.0f       // accel corrects attitude only below this...
#define ACC_GAIN            0.02f       // ...and with |acc| ~ 1 g
#define STOP_RATE           60.0f       // rotation stopped below this on roll and pitch...
#define STOP_HOLD_S         0.10f       // ...for this long
#define BEGIN_RATE          15.0f       // walk-back continues while main-axis rate exceeds this
#define LOOKBACK_S          4.0f
#define TOL_MAIN_DEG        90.0f
#define TOL_OFF_DEG         150.0f
#define THRUST_G            1.5f        // accel stand-in for thrust
#define THRUST_MARGIN       0.10f       // throttle above hover + this = thrust
#define SHARE_UPTO          0.75f       // back pitch: first 75% of the rotation
#define TRACK_ERR           150.0f
#define CONFIRM_HOLD_S      0.15f
#define HOVER_INIT          0.30f
#define HOVER_N_MAX         2000

#define US(seconds)         ((uint32_t)lrintf((seconds) * 1e6f))

#define DEG2RAD             0.017453292519943295f
#define RAD2DEG             57.29577951308232f

static float maxAbs3(const float v[3])
{
    float m = fabsf(v[0]);
    if (fabsf(v[1]) > m) {
        m = fabsf(v[1]);
    }
    if (fabsf(v[2]) > m) {
        m = fabsf(v[2]);
    }
    return m;
}

void trickDetectorInit(trickDetector_t *d, const trickDetectorParams_t *params)
{
    memset(d, 0, sizeof(*d));
    d->params = *params;
    d->up[2] = 1.0f;
    d->hover = HOVER_INIT;
}

// gyro-propagated "up" vector in the body frame; returns tilt from vertical (deg)
static float updateAttitude(trickDetector_t *d, const float g[3], const float a[3], float dt)
{
    const float w[3] = { g[0] * DEG2RAD, g[1] * DEG2RAD, g[2] * DEG2RAD };
    float *u = d->up;
    const float cx = w[1] * u[2] - w[2] * u[1];
    const float cy = w[2] * u[0] - w[0] * u[2];
    const float cz = w[0] * u[1] - w[1] * u[0];
    u[0] -= cx * dt;
    u[1] -= cy * dt;
    u[2] -= cz * dt;
    const float na = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    if (na > 0.9f && na < 1.1f && maxAbs3(g) < STILL_RATE) {
        for (int i = 0; i < 3; i++) {
            u[i] += ACC_GAIN * (a[i] / na - u[i]);
        }
    }
    const float n = sqrtf(u[0] * u[0] + u[1] * u[1] + u[2] * u[2]);
    for (int i = 0; i < 3; i++) {
        u[i] /= n;
    }
    float c = u[2];
    c = c > 1.0f ? 1.0f : (c < -1.0f ? -1.0f : c);
    return acosf(c) * RAD2DEG;
}

// back = 0 is the entry in slot idx; larger values are older
static const trickHistory_t *histFrom(const trickDetector_t *d, int idx, int back)
{
    int i = idx - back;
    while (i < 0) {
        i += TRICK_HIST_SLOTS;
    }
    return &d->hist[i];
}

static void addShare(trickDetector_t *d, float deg, bool thrust)
{
    d->progress += deg;
    int b = (int)(d->progress / TRICK_SHARE_BIN_DEG);
    if (b >= TRICK_SHARE_BINS) {
        b = TRICK_SHARE_BINS - 1;
    }
    d->binTot[b] += deg;
    if (thrust) {
        d->binOn[b] += deg;
    }
}

static void startWalkCombo(trickDetector_t *d)
{
    const int axis = d->walkCombo >> 1, sg = d->walkCombo & 1;
    float *pre = d->preAng[axis][sg];
    pre[0] = pre[1] = pre[2] = 0.0f;
    d->preT0Us[axis][sg] = d->tLeaveUs;
    d->walkTEntryUs = d->tLeaveUs - histFrom(d, d->walkLeaveIdx, 0)->dtUs;    // time of entry 1
    d->walkK = 1;
}

// Called on the sample where the quad leaves Upright (already pushed to history as entry 0).
static void beginSegment(trickDetector_t *d)
{
    d->out = true;
    d->sawInverted = false;
    d->tLeaveUs = d->tUs;
    d->stopSinceUs = 0;
    memset(d->ang, 0, sizeof(d->ang));
    memset(d->binOn, 0, sizeof(d->binOn));
    memset(d->binTot, 0, sizeof(d->binTot));
    d->progress = 0.0f;

    d->walking = true;
    d->walkLeaveIdx = (d->histHead + TRICK_HIST_SLOTS - 1) % TRICK_HIST_SLOTS;
    d->walkCount = d->histCount;
    d->walkCombo = 0;
    d->liveN = 0;
    startWalkCombo(d);
}

// Walk back from "left Upright" while the axis rate keeps its sign above BEGIN_RATE, for each
// (axis, sign); then replay the back-pitch walk into the share bins, oldest first, followed by
// the live samples held meanwhile. Visits at most `budget` entries; resumes on the next call.
static void continueWalk(trickDetector_t *d, int budget)
{
    int work = 0;
    while (d->walking && work < budget) {
        if (d->walkCombo < 4) {
            const int axis = d->walkCombo >> 1, sg = d->walkCombo & 1;
            const float sign = sg ? 1.0f : -1.0f;
            bool end = d->walkK >= d->walkCount;
            if (!end) {
                const trickHistory_t *h = histFrom(d, d->walkLeaveIdx, d->walkK);
                work++;
                if (h->rate[axis] * 0.1f * sign <= BEGIN_RATE || d->tLeaveUs - d->walkTEntryUs >= US(LOOKBACK_S)) {
                    end = true;
                } else {
                    const float dt = h->dtUs * 1e-6f;
                    float *pre = d->preAng[axis][sg];
                    for (int i = 0; i < 3; i++) {
                        pre[i] += h->rate[i] * 0.1f * dt;
                    }
                    d->preT0Us[axis][sg] = d->walkTEntryUs;
                    d->walkTEntryUs -= h->dtUs;
                    d->walkK++;
                }
            }
            if (end) {
                if (axis == 1 && sg == 0) {
                    d->shareK = d->walkK;
                }
                if (++d->walkCombo < 4) {
                    startWalkCombo(d);
                }
            }
        } else if (d->shareK > 1) {
            // back pitch Thrust share: walked-back samples oldest first
            d->shareK--;
            const trickHistory_t *h = histFrom(d, d->walkLeaveIdx, d->shareK);
            work++;
            addShare(d, fabsf(h->rate[1] * 0.1f) * h->dtUs * 1e-6f, h->thrust);
        } else {
            for (int i = 0; i < d->liveN; i++) {
                addShare(d, d->liveDeg[i], d->liveThrust[i]);
            }
            d->liveN = 0;
            d->walking = false;
        }
    }
    d->stepWork += work;
}

static void classify(const trickDetector_t *d, trickEvent_t *ev)
{
    memset(ev, 0, sizeof(*ev));
    const int main = fabsf(d->ang[1]) >= fabsf(d->ang[0]) ? 1 : 0;
    const int sg = d->ang[main] >= 0.0f ? 1 : 0;
    const float *pre = d->preAng[main][sg];
    for (int i = 0; i < 3; i++) {
        ev->angles[i] = pre[i] + d->ang[i];
    }
    const float r = ev->angles[0], pt = ev->angles[1];
    const float m = main ? fabsf(pt) : fabsf(r);
    const float off = main ? fabsf(r) : fabsf(pt);
    int n = (int)lrintf(m / 360.0f);
    if (n < 1) {
        n = 1;
    }
    ev->tEndS = d->tUs * 1e-6f;
    ev->durationS = (d->tUs - d->preT0Us[main][sg]) * 1e-6f;

    if (main == 1 && pt < 0.0f) {
        const float upto = SHARE_UPTO * m;
        float on = 0.0f, tot = 0.0f;
        for (int b = 0; b < TRICK_SHARE_BINS; b++) {
            if ((b + 0.5f) * TRICK_SHARE_BIN_DEG > upto) {
                break;
            }
            on += d->binOn[b];
            tot += d->binTot[b];
        }
        ev->share = tot > 0.0f ? on / tot : 0.0f;
    }

    ev->trick = TRICK_UNMATCHED;
    ev->count = 1;
    if (m >= 360.0f - TOL_MAIN_DEG && fabsf(m - 360.0f * n) <= TOL_MAIN_DEG && off <= TOL_OFF_DEG) {
        if (main == 1) {
            if (pt > 0.0f) {
                ev->trick = TRICK_FRONT_FLIP;   // v1: no Matty rule
            } else {
                const bool longRot = d->params.loopMinS > 0.0f && ev->durationS > d->params.loopMinS;
                ev->trick = (ev->share > d->params.loopShare || longRot) ? TRICK_POWER_LOOP : TRICK_BACK_FLIP;
            }
        } else {
            ev->trick = TRICK_ROLL;
            ev->rollRight = r > 0.0f;
        }
        if (ev->trick != TRICK_POWER_LOOP) {
            ev->count = (uint8_t)n;
        }
    }
}

static int16_t toDeci(float rate)
{
    float v = rate * 10.0f;
    v = v > 32767.0f ? 32767.0f : (v < -32767.0f ? -32767.0f : v);
    return (int16_t)lrintf(v);
}

bool trickDetectorStep(trickDetector_t *d, const trickSample_t *s, trickEvent_t *out)
{
    bool emitted = false;
    d->stepWork = 0;
    d->tUs += US(s->dtS);

    const float tilt = updateAttitude(d, s->gyro, s->acc, s->dtS);
    const bool upright = tilt < UPRIGHT_DEG;
    const bool inverted = tilt > INVERTED_DEG;
    const float rp = fmaxf(fabsf(s->gyro[0]), fabsf(s->gyro[1]));

    if (!s->armed || !s->acro || s->vetoed) {
        d->out = false;
        d->pending = false;
        d->walking = false;
        d->histCount = 0;
        return false;
    }

    // hover learning (real throttle only): calm, Upright, ~1 g
    if (s->hasThrottle && upright && maxAbs3(s->gyro) < 40.0f && fabsf(s->acc[2] - 1.0f) < 0.1f && s->throttle > 0.05f) {
        if (d->hoverN < HOVER_N_MAX) {
            d->hoverN++;
        }
        d->hover += (s->throttle - d->hover) / (float)d->hoverN;
    }
    const float hover = d->params.hoverThrottle > 0.0f ? d->params.hoverThrottle : d->hover;

    // Recovery window for a finished trick
    if (d->pending) {
        float err[3];
        for (int i = 0; i < 3; i++) {
            err[i] = s->gyro[i] - s->setpoint[i];
        }
        const bool ok = upright && maxAbs3(err) < TRACK_ERR;
        if (ok && !d->okSinceUs) {
            d->okSinceUs = d->tUs;
        }
        if (!ok) {
            d->okSinceUs = 0;
        }
        if (d->okSinceUs && d->tUs - d->okSinceUs >= US(CONFIRM_HOLD_S)) {
            d->pendingEvent.landed = true;
            d->pendingEvent.tEndS = d->tUs * 1e-6f;
            *out = d->pendingEvent;
            emitted = true;
            d->pending = false;
        } else if (d->tUs - d->pendingEndUs > US(d->params.recoverS) && !d->okSinceUs) {
            *out = d->pendingEvent;
            emitted = true;
            d->pending = false;
        }
    }

    const bool thrust = s->hasThrottle ? (s->throttle > hover + THRUST_MARGIN) : (s->acc[2] > THRUST_G);
    trickHistory_t *h = &d->hist[d->histHead];
    for (int i = 0; i < 3; i++) {
        h->rate[i] = toDeci(s->gyro[i]);
    }
    h->dtUs = (uint16_t)lrintf(s->dtS * 1e6f);
    h->thrust = thrust;
    d->histHead = (d->histHead + 1) % TRICK_HIST_SLOTS;
    if (d->histCount < TRICK_LOOKBACK_SAMPLES) {
        d->histCount++;
    }

    if (!d->out) {
        if (upright) {
            return emitted;
        }
        beginSegment(d);
    }
    if (d->walking) {
        // finish at once rather than let new samples overwrite entries still to be walked
        continueWalk(d, d->liveN < TRICK_WALK_SLACK - 1 ? TRICK_WALK_BUDGET : INT_MAX);
    }
    for (int i = 0; i < 3; i++) {
        d->ang[i] += s->gyro[i] * s->dtS;
    }
    if (d->walking) {
        d->liveDeg[d->liveN] = fabsf(s->gyro[1]) * s->dtS;
        d->liveThrust[d->liveN] = thrust;
        d->liveN++;
    } else {
        addShare(d, fabsf(s->gyro[1]) * s->dtS, thrust);
    }
    d->sawInverted |= inverted;

    if (!d->sawInverted) {
        if (upright) {
            d->out = false;     // came back without going Inverted: not a trick
            d->walking = false;
        }
        return emitted;
    }

    if (upright) {
        const int main = fabsf(d->ang[1]) >= fabsf(d->ang[0]) ? 1 : 0;
        const int cur = fabsf(s->gyro[1]) >= fabsf(s->gyro[0]) ? 1 : 0;
        const bool switched = rp > 2.0f * STOP_RATE && cur != main && fabsf(d->ang[main]) >= 270.0f;
        if (rp < STOP_RATE) {
            if (!d->stopSinceUs) {
                d->stopSinceUs = d->tUs;
            }
        } else {
            d->stopSinceUs = 0;
        }
        if (switched || (d->stopSinceUs && d->tUs - d->stopSinceUs >= US(STOP_HOLD_S))) {
            continueWalk(d, INT_MAX);  // a trick can't end this soon after leaving; just in case
            trickEvent_t ev;
            classify(d, &ev);
            if (switched) {
                // chaining: still tracking through the Upright crossing is enough
                ev.landed = true;
                if (!emitted) {
                    *out = ev;
                    emitted = true;
                }
            } else {
                d->pending = true;
                d->pendingEvent = ev;
                d->pendingEndUs = d->tUs;
                d->okSinceUs = 0;
            }
            d->out = false;
            d->stopSinceUs = 0;
        }
    }
    return emitted;
}

int trickFormatAnnouncement(const trickEvent_t *ev, char *buf, size_t len)
{
    const char *name = NULL;
    switch (ev->trick) {
    case TRICK_FRONT_FLIP:
        name = "FRONT FLIP";
        break;
    case TRICK_BACK_FLIP:
        name = "BACK FLIP";
        break;
    case TRICK_ROLL:
        name = "ROLL";
        break;
    case TRICK_POWER_LOOP:
        name = "POWER LOOP";
        break;
    default:
        break;
    }
    if (!len) {
        return 0;
    }
    if (!name || !ev->landed) {
        buf[0] = '\0';
        return 0;
    }
    int n;
    if (ev->count == 2) {
        n = snprintf(buf, len, "DOUBLE %s!", name);
    } else if (ev->count == 3) {
        n = snprintf(buf, len, "TRIPLE %s!", name);
    } else if (ev->count > 3) {
        n = snprintf(buf, len, "%dX %s!", ev->count, name);
    } else {
        n = snprintf(buf, len, "%s!", name);
    }
    return n < 0 ? 0 : n;
}
