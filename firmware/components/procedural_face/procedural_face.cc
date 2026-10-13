/*
 * SPDX-License-Identifier: MIT
 *
 * Procedurally drawn Stack-chan face. See procedural_face.h.
 */
#include "procedural_face.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace procedural_face {
namespace {

constexpr float kPi = 3.14159265f;

// ---- Geometry helpers ------------------------------------------------------

struct V2 {
    float x, y;
};

inline V2 operator+(V2 a, V2 b) { return {a.x + b.x, a.y + b.y}; }
inline V2 operator-(V2 a, V2 b) { return {a.x - b.x, a.y - b.y}; }
inline V2 operator*(V2 a, float s) { return {a.x * s, a.y * s}; }
inline float Dot(V2 a, V2 b) { return a.x * b.x + a.y * b.y; }
inline float Len(V2 a) { return std::sqrt(Dot(a, a)); }
inline float Clamp01(float v) { return v < 0 ? 0 : (v > 1 ? 1 : v); }

float SdSegment(V2 p, V2 a, V2 b) {
    V2 pa = p - a, ba = b - a;
    float denom = Dot(ba, ba);
    float h = denom > 0 ? Clamp01(Dot(pa, ba) / denom) : 0;
    return Len(pa - ba * h);
}

// Approximate ellipse distance (Inigo Quilez), good enough for 1px AA.
float SdEllipse(V2 p, V2 r) {
    float k0 = Len({p.x / r.x, p.y / r.y});
    float k1 = Len({p.x / (r.x * r.x), p.y / (r.y * r.y)});
    if (k1 == 0) return -std::min(r.x, r.y);
    return k0 * (k0 - 1.0f) / k1;
}

float SdRoundBox(V2 p, V2 half, float radius) {
    V2 q = {std::fabs(p.x) - half.x + radius, std::fabs(p.y) - half.y + radius};
    V2 qm = {std::max(q.x, 0.0f), std::max(q.y, 0.0f)};
    return Len(qm) + std::min(std::max(q.x, q.y), 0.0f) - radius;
}

// Heart of unit size: tip at (0,0), lobes reaching y = 1 (y up).
float SdHeart(V2 p) {
    p.x = std::fabs(p.x);
    if (p.y + p.x > 1.0f) {
        return Len(p - V2{0.25f, 0.75f}) - std::sqrt(2.0f) / 4.0f;
    }
    float a = Dot(p - V2{0.0f, 1.0f}, p - V2{0.0f, 1.0f});
    float m = 0.5f * std::max(p.x + p.y, 0.0f);
    V2 q = p - V2{m, m};
    float b = Dot(q, q);
    return std::sqrt(std::min(a, b)) * (p.x - p.y > 0 ? 1.0f : -1.0f);
}

// Five-pointed star (Inigo Quilez), radius r, inner ratio rf.
float SdStar5(V2 p, float r, float rf) {
    const V2 k1 = {0.809016994375f, -0.587785252292f};
    const V2 k2 = {-k1.x, k1.y};
    p.x = std::fabs(p.x);
    p = p - k1 * (2.0f * std::max(Dot(k1, p), 0.0f));
    p = p - k2 * (2.0f * std::max(Dot(k2, p), 0.0f));
    p.x = std::fabs(p.x);
    p.y -= r;
    V2 ba = V2{-k1.y, k1.x} * rf - V2{0, 1};
    float h = Clamp01(Dot(p, ba) / Dot(ba, ba));
    return Len(p - ba * h) * ((p.y * ba.x - p.x * ba.y) > 0 ? 1.0f : -1.0f);
}

// ---- Canvas ---------------------------------------------------------------

struct Rgb {
    uint8_t r, g, b;
};

inline uint16_t ToRgb565(Rgb c) {
    return static_cast<uint16_t>(((c.r & 0xF8) << 8) | ((c.g & 0xFC) << 3) | (c.b >> 3));
}

inline Rgb FromRgb565(uint16_t v) {
    uint8_t r = (v >> 11) & 0x1F, g = (v >> 5) & 0x3F, b = v & 0x1F;
    return {static_cast<uint8_t>((r << 3) | (r >> 2)), static_cast<uint8_t>((g << 2) | (g >> 4)),
            static_cast<uint8_t>((b << 3) | (b >> 2))};
}

constexpr Rgb kPink = {255, 120, 160};
constexpr Rgb kBlue = {100, 180, 255};
constexpr Rgb kRed = {255, 70, 70};
constexpr Rgb kHeartRed = {255, 80, 120};
constexpr Rgb kYellow = {255, 215, 80};
constexpr Rgb kGloomColor = {120, 120, 220};
constexpr Rgb kLens = {40, 40, 50};

class Canvas {
public:
    Canvas(uint16_t* px, int w, int h, Rgb fg)
        : px_(px), w_(w), h_(h), fg_(fg),
          scale_(std::min(w / 320.0f, h / 240.0f)),
          ox_((w - 320.0f * scale_) * 0.5f), oy_((h - 240.0f * scale_) * 0.5f) {}

    Rgb fg() const { return fg_; }

    // Fills every pixel whose design-space distance `sdf(p)` is negative,
    // with ~1px anti-aliasing, inside the design-space box [lo, hi].
    template <typename F>
    void Fill(V2 lo, V2 hi, Rgb color, float alpha, F sdf) {
        int x0 = std::max(0, static_cast<int>(std::floor(lo.x * scale_ + ox_)) - 1);
        int y0 = std::max(0, static_cast<int>(std::floor(lo.y * scale_ + oy_)) - 1);
        int x1 = std::min(w_ - 1, static_cast<int>(std::ceil(hi.x * scale_ + ox_)) + 1);
        int y1 = std::min(h_ - 1, static_cast<int>(std::ceil(hi.y * scale_ + oy_)) + 1);
        const float inv = 1.0f / scale_;
        for (int y = y0; y <= y1; y++) {
            for (int x = x0; x <= x1; x++) {
                V2 p = {(x + 0.5f - ox_) * inv, (y + 0.5f - oy_) * inv};
                float d = sdf(p) * scale_;  // in device pixels
                float a = Clamp01(0.5f - d) * alpha;
                if (a <= 0.0f) continue;
                uint16_t& dst = px_[y * w_ + x];
                if (a >= 1.0f) {
                    dst = ToRgb565(color);
                } else {
                    Rgb under = FromRgb565(dst);
                    Rgb mix = {static_cast<uint8_t>(under.r + (color.r - under.r) * a),
                               static_cast<uint8_t>(under.g + (color.g - under.g) * a),
                               static_cast<uint8_t>(under.b + (color.b - under.b) * a)};
                    dst = ToRgb565(mix);
                }
            }
        }
    }

    void Circle(V2 c, float r, Rgb color, float alpha = 1) {
        Fill(c - V2{r, r}, c + V2{r, r}, color, alpha, [&](V2 p) { return Len(p - c) - r; });
    }

    void Ellipse(V2 c, V2 r, Rgb color, float alpha = 1) {
        Fill(c - r, c + r, color, alpha, [&](V2 p) { return SdEllipse(p - c, r); });
    }

    void Ring(V2 c, float r, float thickness, Rgb color) {
        float o = r + thickness;
        Fill(c - V2{o, o}, c + V2{o, o}, color, 1,
             [&](V2 p) { return std::fabs(Len(p - c) - r) - thickness * 0.5f; });
    }

    void Line(V2 a, V2 b, float thickness, Rgb color, float alpha = 1) {
        float r = thickness * 0.5f;
        V2 lo = {std::min(a.x, b.x) - r, std::min(a.y, b.y) - r};
        V2 hi = {std::max(a.x, b.x) + r, std::max(a.y, b.y) + r};
        Fill(lo, hi, color, alpha, [&](V2 p) { return SdSegment(p, a, b) - r; });
    }

    // Stroked polyline; overlapping joints do not double-blend.
    void Polyline(const V2* pts, int n, float thickness, Rgb color, float alpha = 1) {
        if (n < 2) return;
        float r = thickness * 0.5f;
        V2 lo = pts[0], hi = pts[0];
        for (int i = 1; i < n; i++) {
            lo = {std::min(lo.x, pts[i].x), std::min(lo.y, pts[i].y)};
            hi = {std::max(hi.x, pts[i].x), std::max(hi.y, pts[i].y)};
        }
        Fill(lo - V2{r, r}, hi + V2{r, r}, color, alpha, [&](V2 p) {
            float d = 1e9f;
            for (int i = 0; i + 1 < n; i++) d = std::min(d, SdSegment(p, pts[i], pts[i + 1]));
            return d - r;
        });
    }

    // Stroke along the curve y = c.y + f(x - c.x) for |x - c.x| <= half_w,
    // with round caps; df is f's derivative. One curve evaluation per pixel,
    // so a long curve costs about as much as a circle (a Polyline tests
    // every segment at every pixel, which made Bender's mouth ~20x dearer).
    template <typename Y, typename DY>
    void Band(V2 c, float half_w, float thickness, Rgb color, V2 lo, V2 hi, Y f, DY df) {
        const float r = thickness * 0.5f;
        Fill(lo - V2{r, r}, hi + V2{r, r}, color, 1, [&](V2 p) {
            const float x = p.x - c.x;
            if (std::fabs(x) > half_w) {
                const float u = x > 0 ? half_w : -half_w;
                return Len(p - (c + V2{u, f(u)})) - r;
            }
            const float s = df(x);
            return std::fabs(p.y - c.y - f(x)) / std::sqrt(1 + s * s) - r;
        });
    }

    // Arc around c with radius r from angle a0 to a1 (degrees, y up).
    void Arc(V2 c, float r, float a0, float a1, float thickness, Rgb color) {
        constexpr int kSteps = 18;
        V2 pts[kSteps + 1];
        for (int i = 0; i <= kSteps; i++) {
            float a = (a0 + (a1 - a0) * i / kSteps) * kPi / 180.0f;
            pts[i] = {c.x + r * std::cos(a), c.y - r * std::sin(a)};
        }
        Polyline(pts, kSteps + 1, thickness, color);
    }

    // Quadratic Bezier stroke from a to b bending through control point k.
    void Curve(V2 a, V2 k, V2 b, float thickness, Rgb color) {
        constexpr int kSteps = 16;
        V2 pts[kSteps + 1];
        for (int i = 0; i <= kSteps; i++) {
            float t = static_cast<float>(i) / kSteps, u = 1 - t;
            pts[i] = a * (u * u) + k * (2 * u * t) + b * (t * t);
        }
        Polyline(pts, kSteps + 1, thickness, color);
    }

    void Heart(V2 c, float size, Rgb color, float alpha = 1) {
        // Unit heart spans x -0.6..0.6, y 0..1.05 (y up); centre it on c.
        Fill(c - V2{size * 0.65f, size * 0.6f}, c + V2{size * 0.65f, size * 0.6f}, color, alpha,
             [&](V2 p) {
                 V2 q = {(p.x - c.x) / size, (c.y - p.y) / size + 0.5f};
                 return SdHeart(q) * size;
             });
    }

    void Star(V2 c, float r, Rgb color, float rotation_deg = 0) {
        float s = std::sin(rotation_deg * kPi / 180), co = std::cos(rotation_deg * kPi / 180);
        Fill(c - V2{r, r}, c + V2{r, r}, color, 1, [&](V2 p) {
            V2 d = p - c;
            V2 q = {d.x * co + d.y * s, -(-d.x * s + d.y * co)};
            return SdStar5(q, r, 0.45f);
        });
    }

    // Four-pointed twinkle.
    void Sparkle(V2 c, float r, Rgb color) {
        Fill(c - V2{r, r}, c + V2{r, r}, color, 1, [&](V2 p) {
            V2 d = p - c;
            float ax = std::fabs(d.x), ay = std::fabs(d.y);
            // Concave diamond: |x|^0.5 + |y|^0.5 <= r^0.5
            float v = std::sqrt(ax) + std::sqrt(ay) - std::sqrt(r);
            return v * 2.0f;
        });
    }

    void Drop(V2 c, float r, Rgb color) {
        // Round bottom with a point on top.
        Fill(c - V2{r, r * 2.2f}, c + V2{r, r}, color, 1, [&](V2 p) {
            V2 d = p - c;
            float circle = Len(d) - r;
            float cone = d.y < 0 ? std::fabs(d.x) - r * (1.0f + d.y / (r * 2.0f)) : 1e9f;
            float top = -(d.y + r * 2.0f);
            return std::min(circle, std::max(cone, top));
        });
    }

    void RoundBox(V2 c, V2 half, float radius, Rgb color) {
        Fill(c - half, c + half, color, 1, [&](V2 p) { return SdRoundBox(p - c, half, radius); });
    }

private:
    uint16_t* px_;
    int w_, h_;
    Rgb fg_;
    float scale_, ox_, oy_;
};

// ---- Expression table ------------------------------------------------------

enum class Eye : uint8_t {
    kRound,
    kHappy,    // ^ closed smile
    kClosedDown,  // u closed sad / crying
    kLine,     // - closed
    kHeart,
    kCross,
    kSpiral,
    kStar,
    kWide,     // ring with small pupil
    kNarrow,   // half-lidded
    kShades,   // sunglasses (drawn once across both eyes)
    kSqueeze,  // > <
    kDot,      // tiny pupil only
};

enum class Mouth : uint8_t {
    kLine,
    kSmile,
    kBigSmile,
    kFrown,
    kO,
    kBigO,
    kWavy,
    kCat,
    kSmirk,
    kKiss,
    kSmallLine,
};

enum Overlay : uint16_t {
    kNone = 0,
    kBlush = 1 << 0,
    kTear = 1 << 1,
    kTears = 1 << 2,
    kSweat = 1 << 3,
    kAnger = 1 << 4,
    kHearts = 1 << 5,
    kQuestion = 1 << 6,
    kExclaim = 1 << 7,
    kZzz = 1 << 8,
    kSparkles = 1 << 9,
    kGloom = 1 << 10,
    kDots = 1 << 11,
    kOrbitStars = 1 << 12,
};

struct Brow {
    bool show = false;
    float angle = 0;  // degrees; positive lowers the inner end (cross)
    float dy = 0;     // vertical offset, negative = raised
};

struct Spec {
    const char* name;
    Eye left, right;
    float eye_open;  // vertical squash for round eyes, 1 = full circle
    Brow brow_left, brow_right;
    Mouth mouth;
    float mouth_width;
    V2 mouth_offset;
    uint16_t overlays;
    V2 gaze;
    bool animated;
};

constexpr Brow kNoBrow{};
constexpr Brow B(float angle, float dy) { return Brow{true, angle, dy}; }

const Spec kSpecs[kNumExpressions] = {
    // name           left            right           open  brow L         brow R         mouth             w    mouth off   overlays                      gaze         anim
    {"idle",        Eye::kRound,    Eye::kRound,    1.0f, kNoBrow,       kNoBrow,       Mouth::kLine,     56, {0, 0},    kNone,                        {0, 0},      false},
    {"happy",       Eye::kHappy,    Eye::kHappy,    1.0f, kNoBrow,       kNoBrow,       Mouth::kSmile,    64, {0, 0},    kBlush,                       {0, 0},      false},
    {"thinking",    Eye::kRound,    Eye::kRound,    0.9f, B(-8, -6),     B(10, 2),      Mouth::kSmirk,    40, {14, 2},   kDots,                        {12, -10},   true},
    {"sad",         Eye::kRound,    Eye::kRound,    0.85f, B(-22, 0),    B(-22, 0),     Mouth::kFrown,    54, {0, 4},    kTear,                        {0, 6},      true},
    {"surprised",   Eye::kWide,     Eye::kWide,     1.0f, B(0, -12),     B(0, -12),     Mouth::kO,        26, {0, 4},    kExclaim,                     {0, 0},      false},
    {"embarrassed", Eye::kSqueeze,  Eye::kSqueeze,  1.0f, kNoBrow,       kNoBrow,       Mouth::kWavy,     44, {0, 0},    kBlush | kSweat,              {0, 0},      true},
    {"laughing",    Eye::kHappy,    Eye::kHappy,    1.0f, B(-10, -6),    B(-10, -6),    Mouth::kBigSmile, 70, {0, -4},   kBlush | kSparkles,           {0, 0},      true},
    {"excited",     Eye::kStar,     Eye::kStar,     1.0f, kNoBrow,       kNoBrow,       Mouth::kBigSmile, 64, {0, -4},   kSparkles,                    {0, 0},      true},
    {"love",        Eye::kHeart,    Eye::kHeart,    1.0f, kNoBrow,       kNoBrow,       Mouth::kSmile,    52, {0, 0},    kBlush | kHearts,             {0, 0},      true},
    {"wink",        Eye::kRound,    Eye::kHappy,    1.0f, kNoBrow,       B(-6, -4),     Mouth::kSmile,    58, {0, 0},    kSparkles,                    {0, 0},      true},
    {"smug",        Eye::kNarrow,   Eye::kNarrow,   1.0f, B(10, 2),      B(-12, -8),    Mouth::kSmirk,    50, {6, 0},    kNone,                        {6, 0},      false},
    {"cool",        Eye::kShades,   Eye::kShades,   1.0f, kNoBrow,       kNoBrow,       Mouth::kSmirk,    46, {6, 0},    kSparkles,                    {0, 0},      true},
    {"crying",      Eye::kClosedDown, Eye::kClosedDown, 1.0f, B(-22, 0), B(-22, 0),     Mouth::kBigO,     40, {0, 6},    kTears,                       {0, 0},      true},
    {"angry",       Eye::kRound,    Eye::kRound,    0.8f, B(26, 4),      B(26, 4),      Mouth::kFrown,    50, {0, 6},    kAnger,                       {0, 2},      true},
    {"annoyed",     Eye::kNarrow,   Eye::kNarrow,   1.0f, B(14, 2),      B(14, 2),      Mouth::kWavy,     40, {8, 4},    kAnger,                       {-8, 4},     true},
    {"shocked",     Eye::kDot,      Eye::kDot,      1.0f, B(-6, -14),    B(-6, -14),    Mouth::kBigO,     46, {0, 6},    kGloom | kSweat,              {0, 0},      true},
    {"confused",    Eye::kRound,    Eye::kNarrow,   1.0f, B(-10, -10),   B(14, 2),      Mouth::kWavy,     44, {0, 2},    kQuestion,                    {0, 0},      false},
    {"doubtful",    Eye::kNarrow,   Eye::kNarrow,   1.0f, B(8, 0),       B(-4, -8),     Mouth::kSmallLine, 36, {10, 2},  kNone,                        {-10, 0},    false},
    {"sleepy",      Eye::kLine,     Eye::kLine,     1.0f, kNoBrow,       kNoBrow,       Mouth::kO,        18, {0, 6},    kZzz,                         {0, 4},      true},
    {"bored",       Eye::kNarrow,   Eye::kNarrow,   1.0f, B(0, 4),       B(0, 4),       Mouth::kSmallLine, 40, {0, 4},   kNone,                        {-14, 6},    false},
    {"nervous",     Eye::kRound,    Eye::kRound,    0.8f, B(-18, -4),    B(-18, -4),    Mouth::kWavy,     50, {0, 4},    kSweat,                       {8, 0},      true},
    {"scared",      Eye::kWide,     Eye::kWide,     1.0f, B(-24, -6),    B(-24, -6),    Mouth::kWavy,     56, {0, 8},    kGloom | kSweat,              {0, 0},      true},
    {"curious",     Eye::kRound,    Eye::kRound,    1.15f, B(0, -4),     B(-8, -12),    Mouth::kO,        16, {0, 4},    kNone,                        {10, -4},    false},
    {"dizzy",       Eye::kSpiral,   Eye::kSpiral,   1.0f, kNoBrow,       kNoBrow,       Mouth::kWavy,     50, {0, 4},    kOrbitStars,                  {0, 0},      true},
    {"relieved",    Eye::kHappy,    Eye::kHappy,    1.0f, B(-14, 0),     B(-14, 0),     Mouth::kSmile,    52, {0, 0},    kSweat,                       {0, 0},      true},
    {"listening",   Eye::kRound,    Eye::kRound,    1.1f, B(-4, -8),     B(-4, -8),     Mouth::kCat,      30, {0, 0},    kNone,                        {0, -2},     false},
    {"kiss",        Eye::kHappy,    Eye::kHappy,    1.0f, kNoBrow,       kNoBrow,       Mouth::kKiss,     20, {6, 0},    kBlush | kHearts,             {0, 0},      true},
};

static_assert(sizeof(kSpecs) / sizeof(kSpecs[0]) == kNumExpressions, "spec per expression");

// ---- Layout (320x240 design space) -----------------------------------------

constexpr float kEyeY = 100;
constexpr float kEyeDx = 70;
constexpr float kEyeR = 20;
constexpr float kMouthY = 162;
constexpr float kStroke = 7;

float Wave(uint32_t tick, uint32_t period_ms, float phase = 0) {
    return std::sin((static_cast<float>(tick % period_ms) / period_ms + phase) * 2 * kPi);
}

float Ramp(uint32_t tick, uint32_t period_ms, float phase = 0) {
    float t = static_cast<float>(tick % period_ms) / period_ms + phase;
    return t - std::floor(t);
}

void DrawEye(Canvas& c, Eye kind, V2 center, bool is_left, float open, int blink, const State& s) {
    const Rgb fg = c.fg();
    float blink_squash = 1.0f;
    // Blinking closes the eyes that have an open shape.
    if (blink > 0 && (kind == Eye::kRound || kind == Eye::kWide || kind == Eye::kNarrow ||
                      kind == Eye::kHeart || kind == Eye::kDot)) {
        if (blink >= 2) {
            c.Line(center + V2{-kEyeR, 2}, center + V2{kEyeR, 2}, kStroke, fg);
            return;
        }
        blink_squash = 0.35f;
    }
    switch (kind) {
        case Eye::kRound:
            c.Ellipse(center, {kEyeR * std::min(open, 1.2f), kEyeR * open * blink_squash}, fg);
            break;
        case Eye::kHappy:
            c.Arc(center + V2{0, 8}, kEyeR, 20, 160, kStroke, fg);
            break;
        case Eye::kClosedDown:
            c.Arc(center + V2{0, -8}, kEyeR, 200, 340, kStroke, fg);
            break;
        case Eye::kLine:
            c.Line(center + V2{-kEyeR, 2}, center + V2{kEyeR, 2}, kStroke, fg);
            break;
        case Eye::kHeart: {
            float beat = 1.0f + 0.08f * Wave(s.tick_ms, 900);
            c.Heart(center, 46 * beat * std::max(blink_squash, 0.5f), kHeartRed);
            break;
        }
        case Eye::kCross:
            c.Line(center + V2{-13, -13}, center + V2{13, 13}, kStroke, fg);
            c.Line(center + V2{-13, 13}, center + V2{13, -13}, kStroke, fg);
            break;
        case Eye::kSpiral: {
            constexpr int kN = 40;
            V2 pts[kN];
            float spin = (is_left ? 1 : -1) * Ramp(s.tick_ms, 1500) * 2 * kPi;
            for (int i = 0; i < kN; i++) {
                float t = static_cast<float>(i) / (kN - 1);
                float a = t * 3.2f * 2 * kPi + spin;
                float r = 2 + t * (kEyeR + 2);
                pts[i] = center + V2{r * std::cos(a), r * std::sin(a)};
            }
            c.Polyline(pts, kN, 3.5f, fg);
            break;
        }
        case Eye::kStar:
            c.Star(center, 24 + 2 * Wave(s.tick_ms, 700), kYellow, 8 * Wave(s.tick_ms, 1400));
            break;
        case Eye::kWide:
            c.Ring(center, kEyeR + 4, 4, fg);
            c.Circle(center + V2{s.gaze_x * 0.2f, s.gaze_y * 0.2f}, 8 * blink_squash, fg);
            break;
        case Eye::kNarrow:
            // Half-lidded: lower half of the eye under a flat lid.
            c.Fill(center - V2{kEyeR, kEyeR}, center + V2{kEyeR, kEyeR}, fg, 1, [&](V2 p) {
                V2 d = p - center;
                return std::max(Len(d) - kEyeR, -(d.y - 2 + 8 * (1 - open * blink_squash)));
            });
            break;
        case Eye::kShades:
            // Drawn once for both eyes from the left eye's call.
            if (is_left) {
                V2 mid = {160 + s.gaze_x, center.y};
                c.RoundBox(mid + V2{-kEyeDx, 0}, {38, 24}, 10, fg);
                c.RoundBox(mid + V2{kEyeDx, 0}, {38, 24}, 10, fg);
                c.RoundBox(mid + V2{-kEyeDx, 0}, {33, 19}, 7, kLens);
                c.RoundBox(mid + V2{kEyeDx, 0}, {33, 19}, 7, kLens);
                c.Line(mid + V2{-32, -6}, mid + V2{32, -6}, 6, fg);
                c.Line(mid + V2{-kEyeDx - 18, -2}, mid + V2{-kEyeDx - 6, -14}, 3, fg, 0.8f);
                c.Line(mid + V2{kEyeDx - 18, -2}, mid + V2{kEyeDx - 6, -14}, 3, fg, 0.8f);
            }
            break;
        case Eye::kSqueeze: {
            float dir = is_left ? 1.0f : -1.0f;
            V2 pts[3] = {center + V2{-12 * dir, -12}, center + V2{10 * dir, 0},
                         center + V2{-12 * dir, 12}};
            c.Polyline(pts, 3, kStroke, fg);
            break;
        }
        case Eye::kDot:
            c.Circle(center, 7 * std::max(blink_squash, 0.5f), fg);
            break;
    }
}

void DrawBrow(Canvas& c, const Brow& b, V2 eye, bool is_left) {
    if (!b.show) return;
    const float half = 16;
    float rise = half * std::tan(b.angle * kPi / 180);
    float y = eye.y - 34 + b.dy;
    // Inner end is toward the face centre; positive angle lowers it.
    float inner_x = is_left ? eye.x + half : eye.x - half;
    float outer_x = is_left ? eye.x - half : eye.x + half;
    c.Line({outer_x, y - rise}, {inner_x, y + rise}, 5, c.fg());
}

void DrawMouth(Canvas& c, Mouth kind, float w, V2 center, int lip, const State& s) {
    const Rgb fg = c.fg();
    if (lip >= 0) {
        // Lip-sync shapes override the expression mouth while speaking.
        switch (lip) {
            case 0: break;  // closed: fall through to the expression mouth
            case 1: c.Ellipse(center, {14, 8}, fg); return;
            case 2: c.Ellipse(center + V2{0, 2}, {17, 15}, fg); return;
            case 3: c.Ellipse(center, {24, 7}, fg); return;
            case 4: c.Ellipse(center, {9, 10}, fg); return;
            default: break;
        }
    }
    float hw = w * 0.5f;
    switch (kind) {
        case Mouth::kLine:
            c.Line(center + V2{-hw, 0}, center + V2{hw, 0}, kStroke, fg);
            break;
        case Mouth::kSmallLine:
            c.Line(center + V2{-hw, -2}, center + V2{hw, 3}, kStroke, fg);
            break;
        case Mouth::kSmile:
            c.Curve(center + V2{-hw, -6}, center + V2{0, 18}, center + V2{hw, -6}, kStroke, fg);
            break;
        case Mouth::kFrown:
            c.Curve(center + V2{-hw, 8}, center + V2{0, -14}, center + V2{hw, 8}, kStroke, fg);
            break;
        case Mouth::kBigSmile: {
            float top = center.y - 8;
            c.Fill(center - V2{hw, 10}, center + V2{hw, 30}, fg, 1, [&](V2 p) {
                float e = SdEllipse(p - V2{center.x, top}, {hw, 30});
                return std::max(e, top - p.y);
            });
            // Tongue.
            c.Fill(center - V2{hw, 0}, center + V2{hw, 30}, kHeartRed, 1, [&](V2 p) {
                float tongue = SdEllipse(p - V2{center.x, top + 24}, {hw * 0.45f, 10});
                float inside = SdEllipse(p - V2{center.x, top}, {hw - 5, 25});
                return std::max(tongue, std::max(inside, top - p.y));
            });
            break;
        }
        case Mouth::kO:
            c.Ellipse(center, {w * 0.5f, w * 0.55f}, fg);
            break;
        case Mouth::kBigO: {
            float wob = 1.0f + 0.06f * Wave(s.tick_ms, 600);
            c.Ellipse(center + V2{0, 4}, {hw * 0.75f, hw * wob}, fg);
            break;
        }
        case Mouth::kWavy: {
            constexpr int kN = 24;
            V2 pts[kN];
            for (int i = 0; i < kN; i++) {
                float t = static_cast<float>(i) / (kN - 1);
                pts[i] = center + V2{-hw + t * w, 4 * std::sin(t * 4 * kPi)};
            }
            c.Polyline(pts, kN, 5, fg);
            break;
        }
        case Mouth::kCat:
            c.Arc(center + V2{-hw * 0.5f, -4}, hw * 0.5f, 200, 340, 5, fg);
            c.Arc(center + V2{hw * 0.5f, -4}, hw * 0.5f, 200, 340, 5, fg);
            break;
        case Mouth::kSmirk:
            c.Curve(center + V2{-hw, 2}, center + V2{hw * 0.3f, 10}, center + V2{hw, -8}, kStroke, fg);
            break;
        case Mouth::kKiss:
            c.Arc(center + V2{0, -6}, 7, -100, 110, 5, fg);
            c.Arc(center + V2{0, 7}, 7, -110, 100, 5, fg);
            break;
    }
}

void DrawQuestion(Canvas& c, V2 at, float size, Rgb color) {
    float k = size / 30;
    c.Arc(at + V2{0, -8 * k}, 9 * k, 160, -50, 5 * k, color);
    c.Line(at + V2{5.5f * k, 0}, at + V2{0, 6 * k}, 5 * k, color);
    c.Line(at + V2{0, 6 * k}, at + V2{0, 11 * k}, 5 * k, color);
    c.Circle(at + V2{0, 19 * k}, 3.2f * k, color);
}

void DrawZ(Canvas& c, V2 at, float size, Rgb color, float alpha) {
    V2 pts[4] = {at + V2{-size, -size}, at + V2{size, -size}, at + V2{-size, size},
                 at + V2{size, size}};
    c.Polyline(pts, 4, std::max(2.0f, size * 0.35f), color, alpha);
}

void DrawOverlays(Canvas& c, uint16_t ov, const State& s) {
    const uint32_t t = s.tick_ms;
    const V2 left = {160 - kEyeDx, kEyeY}, right = {160 + kEyeDx, kEyeY};
    if (ov & kBlush) {
        c.Ellipse(left + V2{-12, 32}, {20, 9}, kPink, 0.75f);
        c.Ellipse(right + V2{12, 32}, {20, 9}, kPink, 0.75f);
    }
    if (ov & kGloom) {
        for (int i = 0; i < 7; i++) {
            float x = 110 + i * 17;
            c.Line({x, 18}, {x, 44.0f + (i % 2) * 8}, 3, kGloomColor, 0.7f);
        }
    }
    if (ov & kTear) {
        float drop = Ramp(t, 2200) * 40;
        c.Drop(left + V2{6, 26 + drop}, 6, kBlue);
    }
    if (ov & kTears) {
        for (V2 eye : {left, right}) {
            c.Line(eye + V2{-10, 16}, eye + V2{-12, 120}, 9, kBlue, 0.85f);
            float drop = Ramp(t, 900, eye.x > 160 ? 0.5f : 0) * 90;
            c.Drop(eye + V2{-11, 20 + drop}, 6, kBlue);
        }
    }
    if (ov & kSweat) {
        float slide = Ramp(t, 1800) * 14;
        c.Drop({266, 56 + slide}, 8, kBlue);
    }
    if (ov & kAnger) {
        float pulse = 1.0f + 0.12f * Wave(t, 500);
        V2 a = {262, 52};
        float r = 10 * pulse, g = 4 * pulse;
        // Four bracket strokes around a centre point.
        for (int q = 0; q < 4; q++) {
            float sx = (q & 1) ? 1.0f : -1.0f, sy = (q & 2) ? 1.0f : -1.0f;
            V2 p0 = a + V2{sx * g, sy * (g + r)};
            V2 p1 = a + V2{sx * g, sy * g};
            V2 p2 = a + V2{sx * (g + r), sy * g};
            V2 pts[3] = {p0, p1, p2};
            c.Polyline(pts, 3, 5, kRed);
        }
    }
    if (ov & kHearts) {
        for (int i = 0; i < 3; i++) {
            float phase = Ramp(t, 2600, i / 3.0f);
            float x = (i == 1 ? 46 : 272) + 8 * std::sin(phase * 2 * kPi);
            float y = 90 - phase * 80;
            c.Heart({x, y}, 18 + i * 3, kHeartRed, 1.0f - phase * 0.7f);
        }
    }
    if (ov & kQuestion) DrawQuestion(c, {270, 44}, 34, kYellow);
    if (ov & kExclaim) {
        c.Line({270, 22}, {270, 50}, 7, kYellow);
        c.Circle({270, 62}, 4.5f, kYellow);
    }
    if (ov & kZzz) {
        for (int i = 0; i < 3; i++) {
            float phase = Ramp(t, 2400, i / 3.0f);
            DrawZ(c, {236 + phase * 50, 70 - phase * 55}, 5 + phase * 7, c.fg(),
                  1.0f - phase * 0.8f);
        }
    }
    if (ov & kSparkles) {
        const V2 at[3] = {{40, 50}, {282, 40}, {290, 150}};
        for (int i = 0; i < 3; i++) {
            float k = 0.6f + 0.4f * Wave(t, 1100, i / 3.0f);
            c.Sparkle(at[i], 16 * k, kYellow);
        }
    }
    if (ov & kDots) {
        int shown = 1 + static_cast<int>(Ramp(t, 1800) * 3);
        for (int i = 0; i < shown; i++) c.Circle({240 + i * 18.0f, 50 - i * 12.0f}, 5 + i * 2.0f, c.fg());
    }
    if (ov & kOrbitStars) {
        for (int i = 0; i < 3; i++) {
            float a = Ramp(t, 2000, i / 3.0f) * 2 * kPi;
            V2 p = {160 + 120 * std::cos(a), 40 + 14 * std::sin(a)};
            c.Star(p, 9, kYellow, a * 57.3f);
        }
    }
}

// ---- Bender style ----------------------------------------------------------
//
// A visor band holding two big round eyes with square pupils, a grille
// mouth and a small antenna. Eyelids (top coverage + slant, bottom squint)
// carry most of the emotion, as on the cartoon.

enum class BPupil : uint8_t { kSquare, kHeart, kSpiral, kStar, kNone };
enum class BMouth : uint8_t { kGrille, kOpen, kSmall, kWavy, kOh };

struct BenderSpec {
    float lid_l, lid_r;   // top-lid coverage per eye, 0 (open) .. 1 (shut)
    float lid_angle;      // deg; + lowers the inner corners (angry), - raises them (sad)
    float lid_bottom;     // bottom-lid squint coverage 0..1 (happy cheeks)
    float pupil_scale;
    V2 pupil;             // pupil offset in px (added to gaze)
    BPupil pupil_kind;
    BMouth mouth;
    float mouth_w;
    float mouth_curve;    // px; + corners up (grin), - corners down (frown)
    uint16_t overlays;
    bool antenna_glow;
};

const BenderSpec kBenderSpecs[kNumExpressions] = {
    //  lidL  lidR  angle  bot   pupil  pupil off   kind              mouth             w    curve  overlays                 glow
    {0.18f, 0.18f,   0,  0.00f, 1.0f, {0, 0},    BPupil::kSquare, BMouth::kGrille, 120,   0, kNone,                    false},  // idle
    {0.10f, 0.10f,  -6,  0.38f, 1.0f, {0, -2},   BPupil::kSquare, BMouth::kGrille, 132,  14, kBlush,                   false},  // happy
    {0.40f, 0.30f, -10,  0.00f, 0.9f, {12, -12}, BPupil::kSquare, BMouth::kSmall,   70,  -4, kDots,                    false},  // thinking
    {0.42f, 0.42f, -24,  0.00f, 1.0f, {0, 10},   BPupil::kSquare, BMouth::kGrille, 110, -12, kTear,                    false},  // sad
    {0.00f, 0.00f,   0,  0.00f, 0.6f, {0, 0},    BPupil::kSquare, BMouth::kOh,      40,   0, kExclaim,                 true},   // surprised
    {0.30f, 0.30f, -10,  0.20f, 0.9f, {-14, 8},  BPupil::kSquare, BMouth::kWavy,    80,   0, kBlush | kSweat,          false},  // embarrassed
    {0.25f, 0.25f,  -8,  0.50f, 1.0f, {0, 0},    BPupil::kSquare, BMouth::kOpen,   136,  14, kSparkles,                true},   // laughing
    {0.00f, 0.00f,   0,  0.00f, 1.0f, {0, 0},    BPupil::kStar,   BMouth::kOpen,   130,  10, kSparkles,                true},   // excited
    {0.10f, 0.10f,   0,  0.20f, 1.0f, {0, 0},    BPupil::kHeart,  BMouth::kGrille, 120,  12, kBlush | kHearts,         true},   // love
    {0.10f, 1.00f,   0,  0.20f, 1.0f, {0, 0},    BPupil::kSquare, BMouth::kGrille, 126,  12, kSparkles,                false},  // wink
    {0.48f, 0.48f,   6,  0.10f, 1.0f, {10, 0},   BPupil::kSquare, BMouth::kSmall,   96,   0, kNone,                    false},  // smug (mouth skewed below)
    {0.55f, 0.55f,   0,  0.00f, 1.0f, {0, 0},    BPupil::kSquare, BMouth::kGrille, 116,   6, kSparkles,                false},  // cool
    {0.45f, 0.45f, -24,  0.00f, 1.0f, {0, 10},   BPupil::kSquare, BMouth::kOpen,   110, -12, kTears,                   false},  // crying
    {0.42f, 0.42f,  28,  0.00f, 0.8f, {0, 4},    BPupil::kSquare, BMouth::kOpen,   120, -10, kAnger,                   false},  // angry
    {0.52f, 0.52f,  10,  0.15f, 0.9f, {-14, 4},  BPupil::kSquare, BMouth::kSmall,   70,  -2, kAnger,                   false},  // annoyed
    {0.00f, 0.00f,   0,  0.00f, 0.4f, {0, 0},    BPupil::kSquare, BMouth::kOpen,    96, -4,  kGloom | kSweat,          false},  // shocked
    {0.10f, 0.42f,  12,  0.00f, 1.0f, {0, -4},   BPupil::kSquare, BMouth::kWavy,    84,   0, kQuestion,                false},  // confused
    {0.46f, 0.30f,   8,  0.00f, 1.0f, {-12, 0},  BPupil::kSquare, BMouth::kSmall,   70,  -2, kNone,                    false},  // doubtful
    {0.82f, 0.82f,   0,  0.00f, 1.0f, {0, 6},    BPupil::kSquare, BMouth::kOh,      30,   0, kZzz,                     false},  // sleepy
    {0.58f, 0.58f,   0,  0.00f, 1.0f, {-12, 8},  BPupil::kSquare, BMouth::kSmall,   80,   0, kNone,                    false},  // bored
    {0.18f, 0.18f, -16,  0.00f, 0.7f, {8, 0},    BPupil::kSquare, BMouth::kWavy,   100,   0, kSweat,                   false},  // nervous
    {0.00f, 0.00f, -20,  0.00f, 0.45f,{0, 0},    BPupil::kSquare, BMouth::kOpen,   100, -6,  kGloom | kSweat,          false},  // scared
    {0.05f, 0.30f,  -8,  0.00f, 1.1f, {12, -6},  BPupil::kSquare, BMouth::kOh,      34,   0, kNone,                    true},   // curious
    {0.00f, 0.00f,   0,  0.00f, 1.0f, {0, 0},    BPupil::kSpiral, BMouth::kWavy,   100,   0, kOrbitStars,              false},  // dizzy
    {0.35f, 0.35f, -12,  0.30f, 1.0f, {0, 0},    BPupil::kSquare, BMouth::kGrille, 120,   8, kSweat,                   false},  // relieved
    {0.08f, 0.08f,  -4,  0.00f, 1.0f, {0, -6},   BPupil::kSquare, BMouth::kSmall,   80,   0, kNone,                    true},   // listening
    {0.75f, 0.75f,  -6,  0.25f, 1.0f, {0, 0},    BPupil::kNone,   BMouth::kOh,      28,   0, kBlush | kHearts,         false},  // kiss
};

static_assert(sizeof(kBenderSpecs) / sizeof(kBenderSpecs[0]) == kNumExpressions,
              "bender spec per expression");

// The eye plate is drawn as an outline with a black interior; lids use
// the interior colour so a closing eye simply disappears into the plate.
constexpr Rgb kVisor = {0, 0, 0};
constexpr Rgb kVisorRim = {150, 156, 166};
constexpr Rgb kEyeWhite = {250, 244, 214};
constexpr Rgb kPupil = {16, 16, 18};
constexpr Rgb kMetal = {176, 182, 192};
constexpr Rgb kGrilleFill = {206, 210, 214};
constexpr Rgb kGrilleLine = {58, 60, 66};
constexpr Rgb kOrange = {255, 150, 30};

constexpr V2 kBVisor = {160, 96};
constexpr V2 kBVisorHalf = {134, 50};
constexpr float kBEyeR = 37;
constexpr float kBEyeDx = 52;
constexpr float kBMouthY = 190;

void DrawBenderEye(Canvas& c, const BenderSpec& b, V2 center, bool is_left, float lid,
                   int blink, const State& s) {
    if (blink >= 2) lid = 1.0f;
    else if (blink == 1) lid = std::max(lid, 0.6f);
    // Closed: just the lid seam.
    if (lid >= 0.97f) {
        // Shut: the lid fills the socket, with a seam curving down a little.
        c.Ring(center, kBEyeR, 3, kVisorRim);
        c.Curve(center + V2{-kBEyeR + 6, 2}, center + V2{0, 12}, center + V2{kBEyeR - 6, 2}, 5,
                kVisorRim);
        return;
    }
    c.Circle(center, kBEyeR, kEyeWhite);

    // Pupil, kept inside the eye.
    V2 off = b.pupil + V2{s.gaze_x, s.gaze_y};
    float lim = kBEyeR - 14;
    float d = Len(off);
    if (d > lim) off = off * (lim / d);
    V2 pc = center + off;
    float ps = 10 * b.pupil_scale;
    switch (b.pupil_kind) {
        case BPupil::kSquare:
            c.RoundBox(pc, {ps, ps}, 2.5f, kPupil);
            break;
        case BPupil::kHeart: {
            float beat = 1.0f + 0.1f * Wave(s.tick_ms, 900);
            c.Heart(pc, 40 * beat, kHeartRed);
            break;
        }
        case BPupil::kSpiral: {
            constexpr int kN = 36;
            V2 pts[kN];
            float spin = (is_left ? 1 : -1) * Ramp(s.tick_ms, 1400) * 2 * kPi;
            for (int i = 0; i < kN; i++) {
                float t = static_cast<float>(i) / (kN - 1);
                float a = t * 3.0f * 2 * kPi + spin;
                float r = 2 + t * 24;
                pts[i] = center + V2{r * std::cos(a), r * std::sin(a)};
            }
            c.Polyline(pts, kN, 4, kPupil);
            break;
        }
        case BPupil::kStar:
            c.Star(pc, 22 + 2 * Wave(s.tick_ms, 700), kOrange, 8 * Wave(s.tick_ms, 1400));
            break;
        case BPupil::kNone:
            break;
    }

    // Top lid: everything inside the eye above a slanted line. The inner
    // corner is toward the face centre.
    const float dir = is_left ? 1.0f : -1.0f;
    const float slope = std::tan(b.lid_angle * kPi / 180) * dir;
    const float y0 = center.y - kBEyeR + 2 * kBEyeR * lid;
    const float norm = std::sqrt(1 + slope * slope);
    if (lid > 0.01f) {
        auto lid_sdf = [&](V2 p) {
            float above = (p.y - (y0 + slope * (p.x - center.x))) / norm;
            return std::max(Len(p - center) - kBEyeR - 1, above);
        };
        c.Fill(center - V2{kBEyeR + 2, kBEyeR + 2}, center + V2{kBEyeR + 2, kBEyeR + 2}, kVisor, 1,
               lid_sdf);
        c.Fill(center - V2{kBEyeR + 2, kBEyeR + 2}, center + V2{kBEyeR + 2, kBEyeR + 2}, kVisorRim, 1,
               [&](V2 p) {
                   float line = std::fabs((p.y - (y0 + slope * (p.x - center.x))) / norm) - 2.5f;
                   return std::max(Len(p - center) - kBEyeR, line);
               });
    }
    // Bottom lid: a cheek curving up in the middle.
    if (b.lid_bottom > 0.01f) {
        const float yb = center.y + kBEyeR - 2 * kBEyeR * b.lid_bottom;
        auto cheek = [&](V2 p) {
            float u = (p.x - center.x) / kBEyeR;
            float edge = yb + 10 * u * u;
            return std::max(Len(p - center) - kBEyeR - 1, edge - p.y);
        };
        c.Fill(center - V2{kBEyeR + 2, 0}, center + V2{kBEyeR + 2, kBEyeR + 2}, kVisor, 1, cheek);
    }
    c.Ring(center, kBEyeR, 3, kVisorRim);
}

// Grille mouth along a quadratic centreline from left to right corner.
void DrawGrille(Canvas& c, V2 center, float w, float h, float curve, float wobble,
                bool open, const State& s) {
    constexpr int kN = 24;
    V2 pts[kN];
    V2 nrm[kN];
    // Centreline offset from `center` at x (-w/2..w/2) and its slope.
    // Corners rise by `curve` relative to the middle (smile when > 0).
    const float phase = s.tick_ms * 0.006f;
    auto f = [&](float x) {
        float u = 2 * x / w;
        float y = -curve * u * u + curve * 0.35f;
        if (wobble != 0) y += wobble * std::sin((x / w + 0.5f) * 4 * kPi + phase);
        return y;
    };
    auto df = [&](float x) {
        float d = -8 * curve * x / (w * w);
        if (wobble != 0) d += wobble * 4 * kPi / w * std::cos((x / w + 0.5f) * 4 * kPi + phase);
        return d;
    };
    V2 lo = center + V2{-w / 2, 1e9f}, hi = center + V2{w / 2, -1e9f};
    for (int i = 0; i < kN; i++) {
        float x = -w / 2 + w * i / (kN - 1);
        pts[i] = center + V2{x, f(x)};
        lo.y = std::min(lo.y, pts[i].y);
        hi.y = std::max(hi.y, pts[i].y);
    }
    for (int i = 0; i < kN; i++) {
        V2 a = pts[std::max(i - 1, 0)], b = pts[std::min(i + 1, kN - 1)];
        V2 tng = b - a;
        float l = Len(tng);
        nrm[i] = l > 0 ? V2{-tng.y / l, tng.x / l} : V2{0, 1};
    }
    const float hw = w / 2;
    c.Band(center, hw, h + 7, kGrilleLine, lo, hi, f, df);
    c.Band(center, hw, h, kGrilleFill, lo, hi, f, df);
    // Teeth: vertical separators then the centre seam (or dark opening).
    for (int k = 1; k < 6; k++) {
        int i = k * (kN - 1) / 6;
        V2 p = pts[i], n = nrm[i];
        c.Line(p - n * (h / 2 - 1), p + n * (h / 2 - 1), 3, kGrilleLine);
    }
    if (open) {
        c.Band(center, hw, h * 0.42f, kPupil, lo, hi, f, df);
    } else {
        c.Band(center, hw, 3, kGrilleLine, lo, hi, f, df);
    }
}

void DrawBenderMouth(Canvas& c, const BenderSpec& b, Expression e, int lip, const State& s) {
    V2 m = {160 + s.gaze_x * 0.3f, kBMouthY};
    if (lip > 0) {
        switch (lip) {
            case 1: DrawGrille(c, m, b.mouth_w, 30, b.mouth_curve * 0.5f, 0, true, s); return;
            case 2: DrawGrille(c, m, b.mouth_w, 46, b.mouth_curve * 0.5f, 0, true, s); return;
            case 3: DrawGrille(c, m, b.mouth_w + 12, 24, 0, 0, true, s); return;
            case 4: DrawGrille(c, m, 40, 34, 0, 0, true, s); return;
            default: break;
        }
    }
    switch (b.mouth) {
        case BMouth::kGrille:
            DrawGrille(c, m, b.mouth_w, 32, b.mouth_curve, 0, false, s);
            break;
        case BMouth::kOpen: {
            float jaw = 1.0f + 0.08f * Wave(s.tick_ms, 500);
            DrawGrille(c, m, b.mouth_w, 46 * jaw, b.mouth_curve, 0, true, s);
            break;
        }
        case BMouth::kSmall: {
            // Smug / doubtful get a lopsided mouth shifted to one side.
            const bool skew = e == Expression::kSmug || e == Expression::kDoubtful;
            DrawGrille(c, m + V2{skew ? 18.0f : 0.0f, skew ? -4.0f : 0.0f}, b.mouth_w, 24,
                       b.mouth_curve, 0, false, s);
            break;
        }
        case BMouth::kWavy:
            DrawGrille(c, m, b.mouth_w, 26, 0, 2.5f, false, s);
            break;
        case BMouth::kOh:
            DrawGrille(c, m, b.mouth_w, b.mouth_w * 0.9f, 0, 0, true, s);
            break;
    }
}

void RenderBender(const State& s, const Theme& theme, Canvas& c) {
    (void)theme;
    int idx = static_cast<int>(s.expression);
    if (idx < 0 || idx >= kNumExpressions) idx = 0;
    const BenderSpec& b = kBenderSpecs[idx];
    const uint16_t overlays = b.overlays;
    const float bob = 1.2f * Wave(s.tick_ms, 4000);

    // Antenna (the ball glows and pulses on lively expressions).
    const V2 ant = {160, 18 + bob};
    c.Line(ant + V2{0, 6}, {160, kBVisor.y - kBVisorHalf.y + bob}, 5, kMetal);
    Rgb ball = b.antenna_glow
        ? Rgb{255, static_cast<uint8_t>(200 + 40 * Wave(s.tick_ms, 600)), 60}
        : kMetal;
    c.Circle(ant, 8, ball);

    DrawOverlays(c, overlays & (kGloom), s);

    // Eye plate: just a rounded outline, black inside.
    const V2 vc = kBVisor + V2{0, bob};
    c.RoundBox(vc, kBVisorHalf + V2{5, 5}, 51, kVisorRim);
    c.RoundBox(vc, kBVisorHalf, 46, kVisor);

    const V2 left = vc + V2{-kBEyeDx, 0};
    const V2 right = vc + V2{kBEyeDx, 0};
    DrawBenderEye(c, b, left, true, b.lid_l, s.eyes, s);
    DrawBenderEye(c, b, right, false, b.lid_r, s.eyes, s);

    DrawBenderMouth(c, b, s.expression, s.mouth, s);
    DrawOverlays(c, overlays & ~(kGloom), s);
}

}  // namespace

const char* StyleName(Style s) {
    switch (s) {
        case Style::kBender: return "bender";
        case Style::kStackchan:
        default: return "stackchan";
    }
}

int StyleFromName(const char* name) {
    if (name == nullptr) return -1;
    if (std::strcmp(name, "stackchan") == 0 || std::strcmp(name, "classic") == 0) {
        return static_cast<int>(Style::kStackchan);
    }
    if (std::strcmp(name, "bender") == 0) return static_cast<int>(Style::kBender);
    return -1;
}

const char* ExpressionName(Expression e) {
    int i = static_cast<int>(e);
    return (i >= 0 && i < kNumExpressions) ? kSpecs[i].name : "";
}

int ExpressionFromName(const char* name) {
    if (name == nullptr) return -1;
    for (int i = 0; i < kNumExpressions; i++) {
        if (std::strcmp(kSpecs[i].name, name) == 0) return i;
    }
    // Friendly aliases for names models tend to reach for.
    struct Alias { const char* from; Expression to; };
    static const Alias kAliases[] = {
        {"neutral", Expression::kIdle},     {"calm", Expression::kIdle},
        {"joy", Expression::kLaughing},     {"puzzled", Expression::kConfused},
        {"pouty", Expression::kAnnoyed},    {"shy", Expression::kEmbarrassed},
        {"worried", Expression::kNervous},  {"tired", Expression::kSleepy},
        {"afraid", Expression::kScared},    {"thinking_hard", Expression::kThinking},
        {"heart", Expression::kLove},       {"star", Expression::kExcited},
    };
    for (const auto& a : kAliases) {
        if (std::strcmp(a.from, name) == 0) return static_cast<int>(a.to);
    }
    return -1;
}

bool IsAnimated(Expression e) {
    int i = static_cast<int>(e);
    return i >= 0 && i < kNumExpressions && kSpecs[i].animated;
}

void RenderClock(int hour, int minute, bool colon_on, const Theme& theme,
                 uint16_t* pixels, int width, int height) {
    for (int i = 0; i < width * height; i++) pixels[i] = theme.background;
    Canvas c(pixels, width, height, FromRgb565(theme.foreground));
    const Rgb fg = c.fg();

    // Segment bits: a b c d e f g (top, top-right, bottom-right, bottom,
    // bottom-left, top-left, middle).
    static const uint8_t kDigits[10] = {0x3F, 0x06, 0x5B, 0x4F, 0x66,
                                        0x6D, 0x7D, 0x07, 0x7F, 0x6F};
    constexpr float kW = 44, kH = 92, kT = 10, kGap = 4;
    constexpr float kTop = 58;
    auto digit = [&](int d, float x) {
        const uint8_t bits = kDigits[d % 10];
        const float l = x, r = x + kW, t = kTop, m = kTop + kH / 2, b = kTop + kH;
        const V2 seg[7][2] = {
            {{l + kGap, t}, {r - kGap, t}},          // a
            {{r, t + kGap}, {r, m - kGap}},          // b
            {{r, m + kGap}, {r, b - kGap}},          // c
            {{l + kGap, b}, {r - kGap, b}},          // d
            {{l, m + kGap}, {l, b - kGap}},          // e
            {{l, t + kGap}, {l, m - kGap}},          // f
            {{l + kGap, m}, {r - kGap, m}},          // g
        };
        for (int i = 0; i < 7; i++) {
            if (bits & (1 << i)) c.Line(seg[i][0], seg[i][1], kT, fg);
        }
    };
    // Four digits plus a colon, centred on the 320-wide design space.
    const float pitch = kW + 18;
    const float colon_w = 30;
    const float total = 4 * kW + 2 * 18 + colon_w + 2 * 10;
    float x = 160 - total / 2;
    digit(hour / 10, x);
    digit(hour % 10, x + pitch);
    const float cx = x + 2 * kW + 18 + 10 + colon_w / 2;
    if (colon_on) {
        c.Circle({cx, kTop + kH * 0.3f}, 7, fg);
        c.Circle({cx, kTop + kH * 0.7f}, 7, fg);
    }
    const float x2 = cx + colon_w / 2 + 10;
    digit(minute / 10, x2);
    digit(minute % 10, x2 + pitch);
}

void Render(const State& state, const Theme& theme, uint16_t* pixels, int width, int height) {
    const uint16_t bg = theme.background;
    for (int i = 0; i < width * height; i++) pixels[i] = bg;

    int idx = static_cast<int>(state.expression);
    if (idx < 0 || idx >= kNumExpressions) idx = 0;
    const Spec& spec = kSpecs[idx];
    Canvas c(pixels, width, height, FromRgb565(theme.foreground));
    if (state.style == Style::kBender) {
        RenderBender(state, theme, c);
        return;
    }

    State s = state;
    s.gaze_x += spec.gaze.x;
    s.gaze_y += spec.gaze.y;
    // Gentle idle breathing: the whole face bobs by a pixel or two.
    float breathe = 1.5f * Wave(state.tick_ms, 4000);

    const V2 left = {160 - kEyeDx + s.gaze_x, kEyeY + s.gaze_y + breathe};
    const V2 right = {160 + kEyeDx + s.gaze_x, kEyeY + s.gaze_y + breathe};

    DrawOverlays(c, spec.overlays & (kGloom | kBlush), s);
    DrawBrow(c, spec.brow_left, left, true);
    DrawBrow(c, spec.brow_right, right, false);
    DrawEye(c, spec.left, left, true, spec.eye_open, state.eyes, s);
    DrawEye(c, spec.right, right, false, spec.eye_open, state.eyes, s);
    DrawMouth(c, spec.mouth, spec.mouth_width,
              V2{160 + spec.mouth_offset.x + s.gaze_x * 0.3f, kMouthY + spec.mouth_offset.y + breathe},
              state.mouth, s);
    DrawOverlays(c, spec.overlays & ~(kGloom | kBlush), s);
}

}  // namespace procedural_face
