/*
 * SPDX-License-Identifier: MIT
 *
 * Procedurally drawn Stack-chan face with many expressions.
 *
 * The stock avatar shows one of six pre-rendered 160x120 images. This
 * renderer draws the face from parts instead (eyes, brows, mouth and
 * overlays such as blush, tears, sweat, hearts, "?", "!" and "zzz"), so
 * new expressions are a table entry rather than new artwork, and blink /
 * lip-sync compose with any expression instead of swapping the whole
 * frame.
 *
 * Shapes are rasterised from signed distance functions with one-pixel
 * anti-aliasing straight into an RGB565 buffer. There are no ESP-IDF or
 * LVGL dependencies, so the same code builds on the host for previews
 * (firmware/host_test/face_preview.cc).
 */
#pragma once

#include <cstdint>

namespace procedural_face {

// Expression ids. The first six keep the stock avatar's names and order so
// existing set_avatar("idle" ... "embarrassed") calls keep working.
enum class Expression : uint8_t {
    kIdle = 0,
    kHappy,
    kThinking,
    kSad,
    kSurprised,
    kEmbarrassed,
    kLaughing,
    kExcited,
    kLove,
    kWink,
    kSmug,
    kCool,
    kCrying,
    kAngry,
    kAnnoyed,
    kShocked,
    kConfused,
    kDoubtful,
    kSleepy,
    kBored,
    kNervous,
    kScared,
    kCurious,
    kDizzy,
    kRelieved,
    kListening,
    kKiss,
    kCount,
};

constexpr int kNumExpressions = static_cast<int>(Expression::kCount);

// Lower-case name for an expression ("idle", "happy", ...).
const char* ExpressionName(Expression e);

// Index for a name, or -1 when unknown.
int ExpressionFromName(const char* name);

// True when the expression has moving overlays (zzz, hearts, spiral ...)
// and should be re-rendered periodically.
bool IsAnimated(Expression e);

// Visual style of the face. Expressions, blink, lip-sync and gaze mean
// the same thing in every style.
enum class Style : uint8_t {
    kStackchan = 0,  // white dot eyes and line mouth on black
    kBender,         // visor, round eyes with square pupils, grille mouth
    kHal,            // HAL 9000 lens: a red glow that swells with speech
    kCount,
};

const char* StyleName(Style s);
int StyleFromName(const char* name);

struct State {
    Style style = Style::kStackchan;
    Expression expression = Expression::kIdle;
    // 0 = open, 1 = half, 2 = closed (blink), matching the avatar's eyes axis.
    int eyes = 0;
    // -1 = the expression's own mouth; 0..4 = closed / half / open / e / u
    // for lip-sync, matching the avatar's mouth axis.
    int mouth = -1;
    // Gaze offset in pixels, added to the expression's own gaze.
    float gaze_x = 0.0f;
    float gaze_y = 0.0f;
    // Milliseconds since start, drives animated overlays.
    uint32_t tick_ms = 0;
    // True while speech is playing. Styles without a mouth (HAL) animate
    // from tick_ms instead of the coarse lip-sync steps, so every frame
    // drawn shows a fresh level however slowly frames arrive.
    bool speaking = false;
};

struct Theme {
    uint16_t background = 0x0000;  // RGB565
    uint16_t foreground = 0xFFFF;
};

// Renders `state` into `pixels` (RGB565, little-endian, row-major,
// width * height entries). Designed for 320x240 but scales to any size.
void Render(const State& state, const Theme& theme, uint16_t* pixels, int width, int height);

// Draws a large "HH:MM" clock (rounded seven-segment digits) for the
// screensaver. The date line is drawn by the caller with a text font.
void RenderClock(int hour, int minute, bool colon_on, const Theme& theme,
                 uint16_t* pixels, int width, int height);

}  // namespace procedural_face
