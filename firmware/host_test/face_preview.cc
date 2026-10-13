// Renders every procedural face expression into one contact sheet so the
// designs can be reviewed without flashing a device.
//
//   c++ -std=c++17 -O2 -I components/procedural_face \
//       components/procedural_face/procedural_face.cc host_test/face_preview.cc \
//       -o /tmp/face_preview && /tmp/face_preview faces.ppm [tick_ms] [stackchan|bender]
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "procedural_face.h"

using namespace procedural_face;

int main(int argc, char** argv) {
    const char* out = argc > 1 ? argv[1] : "faces.ppm";
    const uint32_t tick = argc > 2 ? static_cast<uint32_t>(std::atoi(argv[2])) : 700;
    const int style_idx = argc > 3 ? StyleFromName(argv[3]) : 0;
    const Style style = static_cast<Style>(style_idx < 0 ? 0 : style_idx);
    constexpr int kW = 320, kH = 240, kCols = 6, kPad = 4;
    // Expressions, then blink and lip-sync variants of idle on the last row.
    const int extra = 6;
    const int cells = kNumExpressions + extra;
    const int rows = (cells + kCols - 1) / kCols;
    const int sheet_w = kCols * (kW + kPad), sheet_h = rows * (kH + kPad);
    std::vector<uint8_t> sheet(sheet_w * sheet_h * 3, 60);
    std::vector<uint16_t> px(kW * kH);

    for (int i = 0; i < cells; i++) {
        State s;
        s.style = style;
        s.tick_ms = tick;
        if (i < kNumExpressions) {
            s.expression = static_cast<Expression>(i);
        } else {
            int v = i - kNumExpressions;
            s.expression = v < 2 ? Expression::kIdle : Expression::kHappy;
            if (v < 2) s.eyes = v + 1;           // half, closed
            else s.mouth = v - 2 + 1;            // half, open, e, u
        }
        Render(s, Theme{}, px.data(), kW, kH);
        int ox = (i % kCols) * (kW + kPad), oy = (i / kCols) * (kH + kPad);
        for (int y = 0; y < kH; y++) {
            for (int x = 0; x < kW; x++) {
                uint16_t v = px[y * kW + x];
                uint8_t* d = &sheet[((oy + y) * sheet_w + ox + x) * 3];
                d[0] = ((v >> 11) & 0x1F) << 3;
                d[1] = ((v >> 5) & 0x3F) << 2;
                d[2] = (v & 0x1F) << 3;
            }
        }
        if (i < kNumExpressions) std::printf("%2d %s\n", i, ExpressionName(s.expression));
    }
    FILE* f = std::fopen(out, "wb");
    if (!f) return 1;
    std::fprintf(f, "P6\n%d %d\n255\n", sheet_w, sheet_h);
    std::fwrite(sheet.data(), 1, sheet.size(), f);
    std::fclose(f);
    return 0;
}
