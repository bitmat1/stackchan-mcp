// Renders every expression in every style with the firmware's own face code,
// one PPM each, for the gateway's control page to show the robot's face.
//
//   c++ -std=c++17 -O2 -I components/procedural_face \
//       components/procedural_face/procedural_face.cc host_test/face_export.cc \
//       -o /tmp/face_export && /tmp/face_export out_dir
#include <cstdio>
#include <string>
#include <vector>

#include "procedural_face.h"

using namespace procedural_face;

int main(int argc, char** argv) {
    const std::string dir = argc > 1 ? argv[1] : ".";
    constexpr int kW = 320, kH = 240;
    std::vector<uint16_t> px(kW * kH);
    std::vector<uint8_t> rgb(kW * kH * 3);
    for (int st = 0; st < static_cast<int>(Style::kCount); st++) {
        for (int ex = 0; ex < kNumExpressions; ex++) {
            State s;
            s.style = static_cast<Style>(st);
            s.expression = static_cast<Expression>(ex);
            s.tick_ms = 700;
            Render(s, Theme{}, px.data(), kW, kH);
            for (int i = 0; i < kW * kH; i++) {
                const uint16_t v = px[i];
                rgb[i * 3] = ((v >> 11) & 0x1F) << 3;
                rgb[i * 3 + 1] = ((v >> 5) & 0x3F) << 2;
                rgb[i * 3 + 2] = (v & 0x1F) << 3;
            }
            const std::string path = dir + "/" + StyleName(s.style) + "-" + ExpressionName(s.expression) + ".ppm";
            FILE* f = std::fopen(path.c_str(), "wb");
            if (!f) return 1;
            std::fprintf(f, "P6\n%d %d\n255\n", kW, kH);
            std::fwrite(rgb.data(), 1, rgb.size(), f);
            std::fclose(f);
        }
    }
    return 0;
}
