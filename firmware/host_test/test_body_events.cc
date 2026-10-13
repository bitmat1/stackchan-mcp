#include "body_events.h"

#include <gtest/gtest.h>

#include <vector>

using body_events::MotionDetector;
using body_events::MotionEvent;
using body_events::ProximityDetector;
using body_events::ProximityEvent;

namespace {

constexpr float kUpright[3] = {0, 0, 1};
constexpr float kOnSide[3] = {1, 0, 0};
constexpr float kNoSpin[3] = {0, 0, 0};
constexpr float kSpin[3] = {0, 60, 0};
constexpr uint32_t kStep = 20;  // firmware samples at 50 Hz

struct Feed {
    MotionDetector det;
    uint32_t t = 1000;
    std::vector<MotionEvent> events;
    uint32_t held_ms = 0;

    void Run(const float a[3], const float g[3], uint32_t ms, bool head_moving = false) {
        for (uint32_t end = t + ms; t < end; t += kStep) {
            MotionEvent e = det.Update(a, g, head_moving, t, &held_ms);
            if (e != MotionEvent::kNone) events.push_back(e);
        }
    }
};

}  // namespace

TEST(MotionDetectorTest, LiftAndSetDownUpright) {
    Feed f;
    f.Run(kUpright, kNoSpin, 3000);
    EXPECT_TRUE(f.events.empty());
    f.Run(kUpright, kSpin, 1000);
    f.Run(kUpright, kNoSpin, 3000);
    ASSERT_EQ(f.events.size(), 2u);
    EXPECT_EQ(f.events[0], MotionEvent::kPickedUp);
    EXPECT_EQ(f.events[1], MotionEvent::kPutDown);
    EXPECT_GT(f.held_ms, 1000u);
}

TEST(MotionDetectorTest, SetDownOnItsSideIsTippedOver) {
    Feed f;
    f.Run(kUpright, kNoSpin, 3000);
    f.Run(kUpright, kSpin, 1000);
    f.Run(kOnSide, kNoSpin, 3000);
    ASSERT_EQ(f.events.size(), 2u);
    EXPECT_EQ(f.events[1], MotionEvent::kTippedOver);
}

TEST(MotionDetectorTest, StandingItBackUpIsPutDown) {
    Feed f;
    f.Run(kUpright, kNoSpin, 3000);
    f.Run(kUpright, kSpin, 1000);
    f.Run(kOnSide, kNoSpin, 3000);
    f.Run(kOnSide, kSpin, 1000);
    f.Run(kUpright, kNoSpin, 3000);
    ASSERT_EQ(f.events.size(), 4u);
    EXPECT_EQ(f.events[1], MotionEvent::kTippedOver);
    EXPECT_EQ(f.events[3], MotionEvent::kPutDown);
}

TEST(MotionDetectorTest, HeadMotionIsIgnoredAndNewPoseRelearned) {
    Feed f;
    f.Run(kUpright, kNoSpin, 3000);
    // The servo tilts the head (and the IMU) while it moves.
    f.Run(kUpright, kSpin, 1000, /*head_moving=*/true);
    const float tilted[3] = {0, 0.5f, 0.866f};
    f.Run(tilted, kNoSpin, 3000);
    EXPECT_TRUE(f.events.empty());
    // Lifting from the new pose and setting it back is not "tipped over".
    f.Run(tilted, kSpin, 1000);
    f.Run(tilted, kNoSpin, 3000);
    ASSERT_EQ(f.events.size(), 2u);
    EXPECT_EQ(f.events[1], MotionEvent::kPutDown);
}

TEST(MotionDetectorTest, ShakeNeedsSeveralJolts) {
    Feed f;
    f.Run(kUpright, kNoSpin, 3000);
    const float jolt[3] = {0, 0, 2.2f};
    for (int i = 0; i < 4; i++) {
        f.Run(jolt, kNoSpin, kStep);
        f.Run(kUpright, kNoSpin, 120);
    }
    bool shook = false;
    for (auto e : f.events) shook |= e == MotionEvent::kShake;
    EXPECT_TRUE(shook);
}

TEST(MotionDetectorTest, SingleBumpIsNotAShake) {
    Feed f;
    f.Run(kUpright, kNoSpin, 3000);
    const float jolt[3] = {0, 0, 2.2f};
    f.Run(jolt, kNoSpin, kStep);
    f.Run(kUpright, kNoSpin, 3000);
    for (auto e : f.events) EXPECT_NE(e, MotionEvent::kShake);
}

// One knock: a single sample knocked sideways, then back to rest.
constexpr float kKnocked[3] = {0.35f, 0, 0.95f};

void Knock(Feed& f) {
    f.Run(kKnocked, kNoSpin, kStep);
}

TEST(MotionDetectorTest, DoubleKnockAtRestIsAKnock) {
    Feed f;
    f.Run(kUpright, kNoSpin, 3000);
    Knock(f);
    f.Run(kUpright, kNoSpin, 280);
    Knock(f);
    f.Run(kUpright, kNoSpin, 2000);
    ASSERT_EQ(f.events.size(), 1u);
    EXPECT_EQ(f.events[0], MotionEvent::kKnock);
}

TEST(MotionDetectorTest, SingleKnockOrSlowKnocksAreNothing) {
    Feed f;
    f.Run(kUpright, kNoSpin, 3000);
    Knock(f);
    f.Run(kUpright, kNoSpin, 1500);
    Knock(f);
    f.Run(kUpright, kNoSpin, 2000);
    EXPECT_TRUE(f.events.empty());
}

TEST(MotionDetectorTest, KnockThresholdIsAdjustable) {
    Feed f;
    f.det.set_knock_threshold(0.5f);
    f.Run(kUpright, kNoSpin, 3000);
    Knock(f);
    f.Run(kUpright, kNoSpin, 280);
    Knock(f);
    f.Run(kUpright, kNoSpin, 2000);
    EXPECT_TRUE(f.events.empty());
    EXPECT_GT(f.det.TakePeakJerk(), 0.3f);
    EXPECT_EQ(f.det.TakePeakJerk(), 0.0f);
}

TEST(MotionDetectorTest, KnocksWhileHeadMovesAreIgnored) {
    Feed f;
    f.Run(kUpright, kNoSpin, 3000);
    f.Run(kKnocked, kNoSpin, kStep, true);
    f.Run(kUpright, kNoSpin, 280, true);
    f.Run(kKnocked, kNoSpin, kStep, true);
    f.Run(kUpright, kNoSpin, 2000);
    EXPECT_TRUE(f.events.empty());
}

TEST(ProximityDetectorTest, NearThenFarWithDuration) {
    ProximityDetector det;
    uint32_t t = 0, dur = 0;
    std::vector<ProximityEvent> ev;
    auto feed = [&](int raw, int n) {
        for (int i = 0; i < n; i++, t += 200) {
            auto e = det.Update(raw, t, &dur);
            if (e != ProximityEvent::kNone) ev.push_back(e);
        }
    };
    feed(80, 10);    // crosstalk baseline
    feed(900, 10);   // hand in front for 2 s
    feed(85, 10);
    ASSERT_EQ(ev.size(), 2u);
    EXPECT_EQ(ev[0], ProximityEvent::kNear);
    EXPECT_EQ(ev[1], ProximityEvent::kFar);
    EXPECT_GE(dur, 1600u);
}

TEST(ProximityDetectorTest, HandAtFiveCmOnTheRealSensor) {
    // Measured on the robot: ~25 clear, ~135 with a hand 5 cm away.
    ProximityDetector det;
    uint32_t t = 0;
    for (int i = 0; i < 20; i++, t += 200) det.Update(25 + i % 5, t);
    bool near = false;
    for (int i = 0; i < 25; i++, t += 200) near |= det.Update(135, t) == ProximityEvent::kNear;
    EXPECT_TRUE(near);
    EXPECT_TRUE(det.near());  // a steady hand is not absorbed into the baseline
    EXPECT_LT(det.baseline(), 40);
}

TEST(ProximityDetectorTest, RepeatVisitsInsideCooldownStaySilent) {
    ProximityDetector det;
    uint32_t t = 0;
    int nears = 0;
    for (int visit = 0; visit < 3; visit++) {
        for (int i = 0; i < 5; i++, t += 200) nears += det.Update(80, t) == ProximityEvent::kNear;
        for (int i = 0; i < 5; i++, t += 200) nears += det.Update(900, t) == ProximityEvent::kNear;
    }
    EXPECT_EQ(nears, 1);
}

TEST(ProximityDetectorTest, SingleSpikeIgnored) {
    ProximityDetector det;
    uint32_t t = 0;
    for (int i = 0; i < 5; i++, t += 200) det.Update(80, t);
    EXPECT_EQ(det.Update(900, t), ProximityEvent::kNone);
    EXPECT_EQ(det.Update(80, t + 200), ProximityEvent::kNone);
    EXPECT_FALSE(det.near());
}
