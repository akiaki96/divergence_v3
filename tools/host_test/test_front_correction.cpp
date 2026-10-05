// front_correction（Core/Src/common/front_correction.cpp）のホストでの単体試験。tools/host_test/run.sh で実行する。
// 換算表は run.sh が tools/gen_front_distance.py で生成したもの（ir_calibration.json）を使う
#include <cmath>
#include <cstdio>
#include <iterator>
#include "common/front_correction.hpp"
#include "config/mouse_config.hpp"

namespace {
using namespace config::front_correction;
using config::front_distance::FRONT_LEFT;
using config::front_distance::FRONT_RIGHT;
using config::front_distance::Point;

int g_failures = 0;

void check(bool ok, const char* what) {
    std::printf("  [%s] %s\n", ok ? " ok " : "FAIL", what);
    if (!ok) ++g_failures;
}

bool near(float a, float b, float tol) {
    return std::fabs(a - b) <= tol;
}

// 距離 mm になる値（表の逆引き，表の点の間は線形）
float valueAt(const Point* table, std::size_t count, float mm) {
    for (std::size_t i = 1; i < count; ++i) {
        if (mm <= table[i].mm) {
            float frac = (mm - table[i - 1].mm) / (table[i].mm - table[i - 1].mm);
            return table[i - 1].value + frac * (table[i].value - table[i - 1].value);
        }
    }
    return NAN;
}

float leftValue(float mm) {
    return valueAt(FRONT_LEFT, std::size(FRONT_LEFT), mm);
}

float rightValue(float mm) {
    return valueAt(FRONT_RIGHT, std::size(FRONT_RIGHT), mm);
}

void testTable() {
    std::printf("table\n");
    const Point& p = FRONT_LEFT[10];
    check(near(front_correction::frontLeftMm(p.value), p.mm, 1e-3f), "a table point converts to its distance");
    check(near(front_correction::frontLeftMm(leftValue(185.f)), 185.f, 0.05f), "between points: linear");
    check(near(front_correction::frontRightMm(rightValue(200.f)), 200.f, 0.05f), "front right table");
    check(std::isnan(front_correction::frontLeftMm(FRONT_LEFT[0].value + 1.f)), "above the table (too near): NaN");
    check(std::isnan(front_correction::frontLeftMm(1.f)), "below the table (too far): NaN");
}

void testEstimate() {
    std::printf("estimate\n");
    float e = front_correction::estimateError(leftValue(REF_LEFT_MM), rightValue(REF_RIGHT_MM), 0.f);
    check(near(e, 0.f, 0.05f), "at the reference: 0");
    e = front_correction::estimateError(leftValue(REF_LEFT_MM + 12.f), rightValue(REF_RIGHT_MM + 12.f), 0.f);
    check(near(e, 12.f, 0.1f), "12 mm farther: +12 (robot behind)");
    e = front_correction::estimateError(leftValue(REF_LEFT_MM + 8.f), rightValue(REF_RIGHT_MM - 8.f), 0.f);
    check(near(e, 0.f, 0.1f), "one near, one far (yaw): averages out");
    e = front_correction::estimateError(leftValue(REF_LEFT_MM - 3.f), rightValue(REF_RIGHT_MM - 3.f), 3.f);
    check(near(e, 0.f, 0.1f), "read 3 mm late and the wall 3 mm nearer: 0");
    e = front_correction::estimateError(leftValue(REF_LEFT_MM), 1.f, 0.f);
    check(std::isnan(e), "one sensor out of the table: NaN");
    e = front_correction::estimateError(leftValue(MAX_DISTANCE_MM + 10.f), rightValue(REF_RIGHT_MM), 0.f);
    check(std::isnan(e), "beyond MAX_DISTANCE_MM: NaN");
}

void testCorrection() {
    std::printf("correction\n");
    check(front_correction::correction(NAN) == 0.f, "NaN: 0");
    check(front_correction::correction(DEADBAND_MM - 0.1f) == 0.f, "inside the deadband: 0");
    check(front_correction::correction(-(DEADBAND_MM - 0.1f)) == 0.f, "inside the deadband (negative): 0");
    check(near(front_correction::correction(DEADBAND_MM + 10.f), GAIN * 10.f, 1e-4f), "gain outside the deadband");
    check(near(front_correction::correction(-(DEADBAND_MM + 10.f)), -GAIN * 10.f, 1e-4f), "sign kept");
    check(front_correction::correction(1000.f) == MAX_MM, "clamped to MAX_MM");
    check(front_correction::correction(-1000.f) == -MAX_MM, "clamped to -MAX_MM");
}

void testSplit() {
    std::printf("split\n");
    constexpr float PRE = 6.f;
    constexpr float MIN_PRE = 1.f;
    auto s = front_correction::split(5.f, PRE, MIN_PRE);
    check(s.pre_adjust == 5.f && s.position_shift == 0.f, "behind: longer pre");
    s = front_correction::split(-4.f, PRE, MIN_PRE);
    check(s.pre_adjust == -4.f && s.position_shift == 0.f, "ahead, pre is enough: shorter pre");
    s = front_correction::split(-6.f, PRE, MIN_PRE);
    check(s.pre_adjust == -6.f && s.position_shift == 0.f, "pre exactly 0: no straight, no shift");
    s = front_correction::split(-10.f, PRE, MIN_PRE);
    check(s.pre_adjust == -6.f && near(s.position_shift, 4.f, 1e-5f), "ahead beyond pre: pre 0 and shift +4");
    s = front_correction::split(-5.5f, PRE, MIN_PRE);
    check(s.pre_adjust == -6.f && near(s.position_shift, -0.5f, 1e-5f),
          "pre shorter than min: pre 0, the 0.5 mm left goes to the shift");
}
} // namespace

int main() {
    testTable();
    testEstimate();
    testCorrection();
    testSplit();
    std::printf("%s (%d failures)\n", g_failures == 0 ? "PASS" : "FAIL", g_failures);
    return g_failures == 0 ? 0 : 1;
}
