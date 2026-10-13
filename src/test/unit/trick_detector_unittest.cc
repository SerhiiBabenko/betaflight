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

#include <stdint.h>
#include <stdbool.h>

#include <algorithm>
#include <string>
#include <vector>

extern "C" {
    #include "flight/trick_detector.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

// Scripted flight: segments of constant body rate and thrust, sampled at 250 Hz.
// gyro = commanded rate (perfect tracking), acc = thrust along body z (what an
// accelerometer reads in flight), throttle = thrust x hover throttle.
struct Segment {
    float seconds;
    float roll, pitch, yaw;     // deg/s; +pitch = nose down
    float thrustG;              // 1 = hover, 0 = throttle cut
};

static const float HOVER_THROTTLE = 0.3f;
static const float DT = 1.0f / 250.0f;

class TrickDetectorTest : public ::testing::Test {
protected:
    trickDetector_t det;
    std::vector<trickEvent_t> events;

    void SetUp() override
    {
        trickDetectorParams_t p = { 0.0f, 0.25f, 1.9f, 0.5f, TRICK_LEVEL_EASY };
        trickDetectorInit(&det, &p);
    }

    void fly(const std::vector<Segment> &script)
    {
        for (const Segment &seg : script) {
            const int n = (int)(seg.seconds / DT + 0.5f);
            for (int i = 0; i < n; i++) {
                trickSample_t s = {};
                s.gyro[0] = seg.roll;
                s.gyro[1] = seg.pitch;
                s.gyro[2] = seg.yaw;
                s.setpoint[0] = seg.roll;
                s.setpoint[1] = seg.pitch;
                s.setpoint[2] = seg.yaw;
                s.acc[2] = seg.thrustG;
                s.throttle = seg.thrustG * HOVER_THROTTLE;
                s.hasThrottle = true;
                s.dtS = DT;
                s.armed = true;
                s.acro = true;
                trickEvent_t ev;
                if (trickDetectorStep(&det, &s, &ev)) {
                    events.push_back(ev);
                }
            }
        }
    }
};

static Segment hover(float seconds) { return { seconds, 0, 0, 0, 1.0f }; }

TEST_F(TrickDetectorTest, BackPitch360WithThrottleCutIsALandedBackFlip)
{
    fly({ hover(2.0f), { 0.7f, 0, -360.0f / 0.7f, 0, 0.0f }, hover(2.0f) });

    ASSERT_EQ(1u, events.size());
    EXPECT_EQ(TRICK_BACK_FLIP, events[0].trick);
    EXPECT_EQ(1, events[0].count);
    EXPECT_TRUE(events[0].landed);
}

TEST_F(TrickDetectorTest, ForwardPitch360IsALandedFrontFlip)
{
    fly({ hover(2.0f), { 0.7f, 0, 360.0f / 0.7f, 0, 0.0f }, hover(2.0f) });

    ASSERT_EQ(1u, events.size());
    EXPECT_EQ(TRICK_FRONT_FLIP, events[0].trick);
    EXPECT_TRUE(events[0].landed);
}

TEST_F(TrickDetectorTest, Roll360IsARollWithItsDirection)
{
    fly({ hover(2.0f), { 0.5f, 720.0f, 0, 0, 0.0f }, hover(2.0f),
                       { 0.5f, -720.0f, 0, 0, 0.0f }, hover(2.0f) });

    ASSERT_EQ(2u, events.size());
    EXPECT_EQ(TRICK_ROLL, events[0].trick);
    EXPECT_TRUE(events[0].rollRight);
    EXPECT_EQ(TRICK_ROLL, events[1].trick);
    EXPECT_FALSE(events[1].rollRight);
}

TEST_F(TrickDetectorTest, BackPitchWithThrustThroughTheClimbIsAPowerLoop)
{
    // climb under power while pitching back slowly, float over the top, pull out
    fly({ hover(2.0f), { 1.0f, 0, -120.0f, 0, 2.0f }, { 0.6f, 0, -150.0f, 0, 0.0f },
          { 0.5f, 0, -300.0f, 0, 1.8f }, hover(2.0f) });

    ASSERT_EQ(1u, events.size());
    EXPECT_EQ(TRICK_POWER_LOOP, events[0].trick);
    EXPECT_EQ(1, events[0].count);
    EXPECT_TRUE(events[0].landed);
}

TEST_F(TrickDetectorTest, SlowBackPitchWithoutThrustIsAPowerLoopByDuration)
{
    // tie-breaker: a back pitch longer than loopMinS (1.9 s) counts as a loop
    fly({ hover(2.0f), { 2.4f, 0, -150.0f, 0, 0.0f }, hover(2.0f) });

    ASSERT_EQ(1u, events.size());
    EXPECT_EQ(TRICK_POWER_LOOP, events[0].trick);
}

TEST_F(TrickDetectorTest, TwoBackFlipsInOneRotationAreCountedAsADouble)
{
    fly({ hover(2.0f), { 1.4f, 0, -720.0f / 1.4f, 0, 0.0f }, hover(2.0f) });

    ASSERT_EQ(1u, events.size());
    EXPECT_EQ(TRICK_BACK_FLIP, events[0].trick);
    EXPECT_EQ(2, events[0].count);
}

TEST_F(TrickDetectorTest, FlipFlowingStraightIntoARollAnnouncesBoth)
{
    fly({ hover(2.0f), { 0.7f, 0, -360.0f / 0.7f, 0, 0.0f }, { 0.5f, 720.0f, 0, 0, 0.0f }, hover(2.0f) });

    ASSERT_EQ(2u, events.size());
    EXPECT_EQ(TRICK_BACK_FLIP, events[0].trick);
    EXPECT_TRUE(events[0].landed);
    EXPECT_EQ(TRICK_ROLL, events[1].trick);
    EXPECT_TRUE(events[1].landed);
}

TEST_F(TrickDetectorTest, FlipFollowedByATumbleIsNotLanded)
{
    // rotation completes, then the gyro stops following the sticks (prop strike tumble)
    fly({ hover(2.0f), { 0.7f, 0, -360.0f / 0.7f, 0, 0.0f } });
    const size_t before = events.size();
    for (int i = 0; i < 250; i++) {
        trickSample_t s = {};
        s.gyro[0] = (i / 25) % 2 ? 40.0f : -40.0f;  // upright but jerking against zero sticks...
        s.gyro[2] = 400.0f;                          // ...while spinning on yaw
        s.acc[2] = 1.0f;
        s.throttle = HOVER_THROTTLE;
        s.hasThrottle = true;
        s.dtS = DT;
        s.armed = true;
        s.acro = true;
        trickEvent_t ev;
        if (trickDetectorStep(&det, &s, &ev)) {
            events.push_back(ev);
        }
    }

    ASSERT_EQ(before + 1, events.size());
    EXPECT_EQ(TRICK_BACK_FLIP, events.back().trick);
    EXPECT_FALSE(events.back().landed);
}

TEST_F(TrickDetectorTest, HalfRollThenHalfPitchLookAlikeIsNotATrick)
{
    // Split-S-like: roll to inverted, then pitch back half a loop
    fly({ hover(2.0f), { 0.5f, 360.0f, 0, 0, 0.0f }, { 0.7f, 0, -180.0f / 0.7f, 0, 0.5f }, hover(2.0f) });

    for (const trickEvent_t &ev : events) {
        EXPECT_EQ(TRICK_UNMATCHED, ev.trick);
    }
}

TEST_F(TrickDetectorTest, LongWalkBackIsSpreadOverSteps)
{
    // 4 s of slow back pitch while still Upright fills the whole lookback, then the loop:
    // no single step may walk more than the budget (the FC task must stay short)
    int maxWork = 0;
    for (const Segment &seg : std::vector<Segment>{ hover(2.0f), { 4.0f, 0, -18.0f, 0, 1.0f },
                                                    { 0.8f, 0, -450.0f, 0, 1.0f }, hover(2.0f) }) {
        const int n = (int)(seg.seconds / DT + 0.5f);
        for (int i = 0; i < n; i++) {
            trickSample_t s = {};
            s.gyro[0] = s.setpoint[0] = seg.roll;
            s.gyro[1] = s.setpoint[1] = seg.pitch;
            s.gyro[2] = s.setpoint[2] = seg.yaw;
            s.acc[2] = seg.thrustG;
            s.throttle = seg.thrustG * HOVER_THROTTLE;
            s.hasThrottle = true;
            s.dtS = DT;
            s.armed = true;
            s.acro = true;
            trickEvent_t ev;
            if (trickDetectorStep(&det, &s, &ev)) {
                events.push_back(ev);
            }
            maxWork = std::max(maxWork, (int)det.stepWork);
        }
    }

    EXPECT_LE(maxWork, TRICK_WALK_BUDGET);
    ASSERT_EQ(1u, events.size());
    EXPECT_EQ(TRICK_POWER_LOOP, events[0].trick);      // 4.8 s back pitch: a loop by duration
    // walked back the full 4 s lookback: the loop (360) plus the slow climb inside the window,
    // which ends ~71 deg into the loop where the quad leaves Upright
    EXPECT_NEAR(-(360.0f + 18.0f * (4.0f - 71.0f / 450.0f)), events[0].angles[1], 1.0f);
}

// one trick of `deg` degrees on pitch (+ forward, - back) or roll, flown in 0.7 s without thrust
static Segment pitchBy(float deg) { return { 0.7f, 0, deg / 0.7f, 0, 0.0f }; }
static Segment rollBy(float deg) { return { 0.5f, deg / 0.5f, 0, 0, 0.0f }; }

class TrickGradeTest : public TrickDetectorTest {
protected:
    void level(trickLevel_e lv)
    {
        trickDetectorParams_t p = { 0.0f, 0.25f, 1.9f, 0.5f, (uint8_t)lv };
        trickDetectorInit(&det, &p);
    }
    // the single event of one trick flown between hovers
    trickEvent_t one(const Segment &trick)
    {
        events.clear();
        fly({ hover(2.0f), trick, hover(2.0f) });
        EXPECT_EQ(1u, events.size());
        return events.empty() ? trickEvent_t{} : events[0];
    }
};

TEST_F(TrickGradeTest, FrontFlipIsCleanPlainOrUglyByHowFarItMissesAFullTurn)
{
    EXPECT_EQ(TRICK_GRADE_CLEAN, one(pitchBy(360.0f)).grade);
    EXPECT_EQ(TRICK_GRADE_CLEAN, one(pitchBy(350.0f)).grade);
    EXPECT_EQ(TRICK_GRADE_PLAIN, one(pitchBy(330.0f)).grade);
    EXPECT_EQ(TRICK_GRADE_PLAIN, one(pitchBy(400.0f)).grade);
    const trickEvent_t ugly = one(pitchBy(295.0f));
    EXPECT_EQ(TRICK_FRONT_FLIP, ugly.trick);
    EXPECT_EQ(TRICK_GRADE_UGLY, ugly.grade);
}

TEST_F(TrickGradeTest, BackFlipExitingNoseDownIsStillClean)
{
    // a clean Back Flip ends ~30 deg nose-down, into forward flight; a Front Flip that short is not clean
    EXPECT_EQ(TRICK_GRADE_CLEAN, one(pitchBy(-330.0f)).grade);
    EXPECT_EQ(TRICK_GRADE_PLAIN, one(pitchBy(330.0f)).grade);
    EXPECT_EQ(TRICK_GRADE_PLAIN, one(pitchBy(-395.0f)).grade);
    EXPECT_EQ(TRICK_GRADE_UGLY, one(pitchBy(-310.0f)).grade);   // stalled ~50 short
}

TEST_F(TrickGradeTest, RollIsGradedLikeAFrontFlip)
{
    EXPECT_EQ(TRICK_GRADE_CLEAN, one(rollBy(370.0f)).grade);
    EXPECT_EQ(TRICK_GRADE_UGLY, one(rollBy(-430.0f)).grade);
}

TEST_F(TrickGradeTest, PowerLoopIsNeverGraded)
{
    const trickEvent_t ev = one({ 2.4f, 0, -150.0f, 0, 0.0f });
    EXPECT_EQ(TRICK_POWER_LOOP, ev.trick);
    EXPECT_EQ(TRICK_GRADE_PLAIN, ev.grade);
}

TEST_F(TrickGradeTest, StricterLevelsHideWhatEasyCallsUgly)
{
    EXPECT_EQ(TRICK_GRADE_UGLY, one(pitchBy(-310.0f)).grade);
    level(TRICK_LEVEL_NORMAL);
    EXPECT_EQ(TRICK_GRADE_UGLY, one(pitchBy(-310.0f)).grade);
    level(TRICK_LEVEL_STRICT);
    const trickEvent_t ev = one(pitchBy(-310.0f));
    EXPECT_EQ(TRICK_UNMATCHED, ev.trick);
    char buf[32];
    EXPECT_EQ(0, trickFormatAnnouncement(&ev, buf, sizeof(buf)));
}

TEST_F(TrickGradeTest, StrictNarrowsClean)
{
    level(TRICK_LEVEL_STRICT);
    EXPECT_EQ(TRICK_GRADE_CLEAN, one(pitchBy(355.0f)).grade);
    EXPECT_EQ(TRICK_GRADE_PLAIN, one(pitchBy(350.0f)).grade);
    EXPECT_EQ(TRICK_GRADE_UGLY, one(pitchBy(340.0f)).grade);
}

TEST_F(TrickGradeTest, DoublesAreGradedAgainstTwoTurnsOnEveryLevel)
{
    level(TRICK_LEVEL_STRICT);
    const trickEvent_t ev = one({ 1.4f, 0, 715.0f / 1.4f, 0, 0.0f });
    EXPECT_EQ(TRICK_FRONT_FLIP, ev.trick);
    EXPECT_EQ(2, ev.count);
    EXPECT_EQ(TRICK_GRADE_CLEAN, ev.grade);
}

TEST_F(TrickDetectorTest, NothingIsReportedOutsideAcro)
{
    for (int i = 0; i < (int)(3.0f / DT); i++) {
        trickSample_t s = {};
        s.gyro[1] = (i > 500 && i < 675) ? -514.0f : 0.0f;
        s.acc[2] = 1.0f;
        s.throttle = HOVER_THROTTLE;
        s.hasThrottle = true;
        s.dtS = DT;
        s.armed = true;
        s.acro = false;     // Angle mode
        trickEvent_t ev;
        if (trickDetectorStep(&det, &s, &ev)) {
            events.push_back(ev);
        }
    }

    EXPECT_EQ(0u, events.size());
}

static std::string announce(trickId_e trick, uint8_t count)
{
    trickEvent_t ev = {};
    ev.trick = trick;
    ev.count = count;
    ev.landed = true;
    char buf[32];
    trickFormatAnnouncement(&ev, buf, sizeof(buf));
    return buf;
}

TEST(TrickAnnouncementTest, NamesTheV1Tricks)
{
    EXPECT_EQ("FRONT FLIP!", announce(TRICK_FRONT_FLIP, 1));
    EXPECT_EQ("BACK FLIP!", announce(TRICK_BACK_FLIP, 1));
    EXPECT_EQ("ROLL!", announce(TRICK_ROLL, 1));
    EXPECT_EQ("POWER LOOP!", announce(TRICK_POWER_LOOP, 1));
}

TEST(TrickAnnouncementTest, CountsAsDoubleTripleThenNumber)
{
    EXPECT_EQ("DOUBLE BACK FLIP!", announce(TRICK_BACK_FLIP, 2));
    EXPECT_EQ("TRIPLE ROLL!", announce(TRICK_ROLL, 3));
    EXPECT_EQ("4X FRONT FLIP!", announce(TRICK_FRONT_FLIP, 4));
}

static std::string announce(trickId_e trick, uint8_t count, trickGrade_e grade)
{
    trickEvent_t ev = {};
    ev.trick = trick;
    ev.count = count;
    ev.grade = grade;
    ev.landed = true;
    char buf[32];
    trickFormatAnnouncement(&ev, buf, sizeof(buf));
    return buf;
}

TEST(TrickAnnouncementTest, GradeComesFirst)
{
    EXPECT_EQ("CLEAN ROLL!", announce(TRICK_ROLL, 1, TRICK_GRADE_CLEAN));
    EXPECT_EQ("UGLY DOUBLE BACK FLIP!", announce(TRICK_BACK_FLIP, 2, TRICK_GRADE_UGLY));
    EXPECT_EQ("CLEAN TRIPLE FRONT FLIP!", announce(TRICK_FRONT_FLIP, 3, TRICK_GRADE_CLEAN));
    EXPECT_EQ("POWER LOOP!", announce(TRICK_POWER_LOOP, 1, TRICK_GRADE_PLAIN));
}

TEST(TrickAnnouncementTest, EveryAnnouncementFitsTheOsdWidth)
{
    for (int t = TRICK_FRONT_FLIP; t <= TRICK_POWER_LOOP; t++) {
        for (int n = 1; n <= 255; n++) {
            for (int g = TRICK_GRADE_PLAIN; g <= TRICK_GRADE_UGLY; g++) {
                const std::string text = announce((trickId_e)t, (uint8_t)n, (trickGrade_e)g);
                EXPECT_LE(text.size(), (size_t)TRICK_TEXT_MAX) << text;
                EXPECT_FALSE(text.empty());
            }
        }
    }
}

TEST(TrickAnnouncementTest, UnmatchedOrNotLandedAnnouncesNothing)
{
    EXPECT_EQ("", announce(TRICK_UNMATCHED, 1));
    trickEvent_t ev = {};
    ev.trick = TRICK_BACK_FLIP;
    ev.count = 1;
    ev.landed = false;
    char buf[32];
    EXPECT_EQ(0, trickFormatAnnouncement(&ev, buf, sizeof(buf)));
    EXPECT_STREQ("", buf);
}
