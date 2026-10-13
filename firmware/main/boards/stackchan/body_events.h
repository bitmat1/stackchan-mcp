// Turns raw IMU / proximity samples into the few events the robot reports.
// Pure logic with no ESP-IDF dependencies so it can be tested on the host
// (firmware/host_test/test_body_events.cc).
#pragma once

#include <cmath>
#include <cstdint>

namespace body_events {

enum class MotionEvent : uint8_t { kNone, kPickedUp, kPutDown, kTippedOver, kShake, kKnock };
enum class ProximityEvent : uint8_t { kNone, kNear, kFar };

inline const char* Name(MotionEvent e) {
    switch (e) {
        case MotionEvent::kPickedUp: return "picked_up";
        case MotionEvent::kPutDown: return "put_down";
        case MotionEvent::kTippedOver: return "tipped_over";
        case MotionEvent::kShake: return "shake";
        case MotionEvent::kKnock: return "knock";
        default: return "";
    }
}

inline const char* Name(ProximityEvent e) {
    switch (e) {
        case ProximityEvent::kNear: return "near";
        case ProximityEvent::kFar: return "far";
        default: return "";
    }
}

// The CoreS3 rides on the moving head, so servo motion is invisible to this
// detector: callers pass head_moving and samples are ignored until the head
// has been still for kSettleMs, after which the resting gravity is re-learned.
class MotionDetector {
public:
    static constexpr uint32_t kSettleMs = 600;       // after the head stops
    static constexpr uint32_t kRestMs = 1500;        // still this long = resting
    static constexpr uint32_t kLiftMs = 250;         // disturbed this long = picked up
    static constexpr float kStillAccelG = 0.06f;     // | |a| - 1 g | while still
    static constexpr float kStillGyroDps = 6.0f;
    static constexpr float kLiftAccelG = 0.12f;
    static constexpr float kLiftGyroDps = 25.0f;
    static constexpr float kShakePeakG = 0.9f;       // | |a| - 1 g | of one jolt
    static constexpr int kShakePeaks = 3;
    static constexpr uint32_t kShakeWindowMs = 1500;
    static constexpr uint32_t kShakeCooldownMs = 3000;
    static constexpr float kTippedDeg = 40.0f;       // put down this far from upright
    // Knock = two sharp jolts (sample-to-sample change in accel) 120..700 ms
    // apart while the robot is resting: a double tap on the head or body.
    // Requiring two keeps a bump of the table or setting it down from
    // counting. The threshold is adjustable at run time (set_knock_threshold).
    static constexpr float kKnockJerkG = 0.25f;
    static constexpr uint32_t kKnockRingMs = 100;    // jolts this close = one knock
    static constexpr uint32_t kKnockMinGapMs = 120;
    static constexpr uint32_t kKnockMaxGapMs = 700;
    static constexpr uint32_t kKnockRestMs = 300;    // still this long before the first
    static constexpr uint32_t kKnockCooldownMs = 1500;

    // accel in g, gyro in deg/s. Returns at most one event per sample;
    // held_ms reports how long the robot was held for kPutDown/kTippedOver.
    MotionEvent Update(const float a[3], const float g[3], bool head_moving, uint32_t now_ms,
                       uint32_t* held_ms = nullptr) {
        if (head_moving) {
            have_prev_ = false;
            head_still_since_ = now_ms;
            disturbed_since_ = 0;
            still_since_ = 0;
            if (!held_) have_rest_ = false;  // gravity moves with the head
            return MotionEvent::kNone;
        }
        if (now_ms - head_still_since_ < kSettleMs) return MotionEvent::kNone;

        const float mag = std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
        const float dev = std::fabs(mag - 1.0f);
        const float rot = std::sqrt(g[0] * g[0] + g[1] * g[1] + g[2] * g[2]);
        const uint32_t still_for = still_since_ != 0 ? now_ms - still_since_ : 0;

        float jerk = 0;
        if (have_prev_) {
            for (int i = 0; i < 3; i++) jerk += (a[i] - prev_[i]) * (a[i] - prev_[i]);
            jerk = std::sqrt(jerk);
        }
        for (int i = 0; i < 3; i++) prev_[i] = a[i];
        have_prev_ = true;
        if (jerk > peak_jerk_) peak_jerk_ = jerk;
        if (!held_ && jerk > knock_jerk_g_ && dev < kShakePeakG &&
            now_ms - last_jolt_ms_ > kKnockRingMs) {
            const uint32_t gap = now_ms - first_knock_ms_;
            bool knocked = false;
            if (knock_pending_ && gap >= kKnockMinGapMs && gap <= kKnockMaxGapMs) {
                knock_pending_ = false;
                knocked = now_ms - last_knock_ms_ > kKnockCooldownMs;
            } else if (still_for >= kKnockRestMs) {
                knock_pending_ = true;
                first_knock_ms_ = now_ms;
            } else {
                knock_pending_ = false;
            }
            last_jolt_ms_ = now_ms;
            if (knocked) {
                last_knock_ms_ = now_ms;
                return MotionEvent::kKnock;
            }
        } else if (jerk > knock_jerk_g_) {
            last_jolt_ms_ = now_ms;  // still ringing from the same knock
        }

        if (dev > kShakePeakG && now_ms - last_peak_ms_ > 80) {
            if (now_ms - first_peak_ms_ > kShakeWindowMs) {
                first_peak_ms_ = now_ms;
                peaks_ = 0;
            }
            last_peak_ms_ = now_ms;
            if (++peaks_ >= kShakePeaks && now_ms - last_shake_ms_ > kShakeCooldownMs) {
                last_shake_ms_ = now_ms;
                peaks_ = 0;
                return MotionEvent::kShake;
            }
        }

        const bool still = dev < kStillAccelG && rot < kStillGyroDps;
        const bool disturbed = dev > kLiftAccelG || rot > kLiftGyroDps;
        if (still) {
            if (still_since_ == 0) still_since_ = now_ms;
        } else {
            still_since_ = 0;
        }

        if (!held_) {
            if (still && now_ms - still_since_ >= kRestMs) {
                if (!have_rest_) {
                    for (int i = 0; i < 3; i++) rest_[i] = a[i] / mag;
                    have_rest_ = true;
                }
            }
            if (!have_rest_) return MotionEvent::kNone;  // never lift-detect from an unknown pose
            if (disturbed) {
                if (disturbed_since_ == 0) disturbed_since_ = now_ms;
                if (now_ms - disturbed_since_ >= kLiftMs) {
                    held_ = true;
                    held_since_ = now_ms;
                    return MotionEvent::kPickedUp;
                }
            } else {
                disturbed_since_ = 0;
            }
            return MotionEvent::kNone;
        }

        // Held: wait until it rests again, then compare with the old upright.
        if (still && now_ms - still_since_ >= kRestMs) {
            held_ = false;
            disturbed_since_ = 0;
            if (held_ms != nullptr) held_ms[0] = now_ms - held_since_;
            float cosang = 0;
            for (int i = 0; i < 3; i++) cosang += rest_[i] * a[i] / mag;
            const bool tipped = cosang < std::cos(kTippedDeg * 3.14159265f / 180.0f);
            // Keep the upright reference while it lies on its side, so
            // standing it back up reads as put_down, not another tip.
            if (!tipped) {
                for (int i = 0; i < 3; i++) rest_[i] = a[i] / mag;
            }
            return tipped ? MotionEvent::kTippedOver : MotionEvent::kPutDown;
        }
        return MotionEvent::kNone;
    }

    bool held() const { return held_; }
    float knock_threshold() const { return knock_jerk_g_; }
    void set_knock_threshold(float g) { knock_jerk_g_ = g; }
    // Biggest sample-to-sample jolt since the last call, for tuning.
    float TakePeakJerk() {
        const float peak = peak_jerk_;
        peak_jerk_ = 0;
        return peak;
    }

private:
    float rest_[3] = {0, 0, 1};
    bool have_rest_ = false;
    bool held_ = false;
    uint32_t head_still_since_ = 0;
    uint32_t still_since_ = 0;
    uint32_t disturbed_since_ = 0;
    uint32_t held_since_ = 0;
    uint32_t first_peak_ms_ = 0;
    uint32_t last_peak_ms_ = 0;
    uint32_t last_shake_ms_ = 0;
    int peaks_ = 0;
    float prev_[3] = {0, 0, 0};
    bool have_prev_ = false;
    float knock_jerk_g_ = kKnockJerkG;
    float peak_jerk_ = 0;
    bool knock_pending_ = false;
    uint32_t first_knock_ms_ = 0;
    uint32_t last_jolt_ms_ = 0;
    uint32_t last_knock_ms_ = 0;
};

// Raw LTR-553 counts include crosstalk through the cover glass, so "near"
// is measured against a baseline: it drops quickly to the clear reading but
// rises only one count per sample, so a hand cannot become the baseline.
// On the CoreS3 the clear reading is ~25 and a hand at 5 cm adds ~110.
class ProximityDetector {
public:
    static constexpr int kNearDelta = 60;    // counts above baseline
    static constexpr int kFarDelta = 30;     // hysteresis
    static constexpr int kNearSamples = 2;
    static constexpr int kFarSamples = 3;
    static constexpr uint32_t kNearCooldownMs = 20000;  // one greeting per visit

    // duration_ms reports how long the object stayed near on kFar.
    ProximityEvent Update(int raw, uint32_t now_ms, uint32_t* duration_ms = nullptr) {
        if (raw < 0) return ProximityEvent::kNone;
        if (baseline_ < 0) baseline_ = raw;
        if (!near_) {
            if (raw > baseline_ + kNearDelta) {
                if (++count_ >= kNearSamples) {
                    near_ = true;
                    count_ = 0;
                    near_since_ = now_ms;
                    if (now_ms - last_near_event_ms_ >= kNearCooldownMs || !reported_once_) {
                        reported_once_ = true;
                        last_near_event_ms_ = now_ms;
                        reported_near_ = true;
                        return ProximityEvent::kNear;
                    }
                    reported_near_ = false;
                }
            } else {
                count_ = 0;
                if (raw < baseline_) baseline_ -= (baseline_ - raw + 3) / 4;
                else if (raw > baseline_) baseline_ += 1;
            }
            return ProximityEvent::kNone;
        }
        if (raw < baseline_ + kFarDelta) {
            if (++count_ >= kFarSamples) {
                near_ = false;
                count_ = 0;
                if (reported_near_) {
                    if (duration_ms != nullptr) duration_ms[0] = now_ms - near_since_;
                    return ProximityEvent::kFar;
                }
            }
        } else {
            count_ = 0;
        }
        return ProximityEvent::kNone;
    }

    bool near() const { return near_; }
    int baseline() const { return baseline_; }

private:
    int baseline_ = -1;
    int count_ = 0;
    bool near_ = false;
    bool reported_once_ = false;
    bool reported_near_ = false;
    uint32_t near_since_ = 0;
    uint32_t last_near_event_ms_ = 0;
};

}  // namespace body_events
