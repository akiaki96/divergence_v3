// trapezoid::split（Core/Src/common/trapezoid.cpp）のホストでの単体試験。tools/host_test/run.sh で実行する。
//
// 区間の距離の和が d，最後の終速が v_out，区間ごとの加速度が a / b 以内，速度が v_max 以下であることを，
// 探索の既知の直進（500 → 1000 → 500）と最短走行の形でいくつかの長さについて確かめる
#include <cmath>
#include <cstdio>
#include <initializer_list>
#include "common/trapezoid.hpp"

namespace {
int g_failures = 0;

void check(bool ok, const char* what) {
    std::printf("  [%s] %s\n", ok ? " ok " : "FAIL", what);
    if (!ok) ++g_failures;
}

bool splitOk(float d, float v_in, float v_out, float v_max, float a, float b) {
    trapezoid::Part parts[trapezoid::MAX_PARTS];
    uint8_t n = trapezoid::split(d, v_in, v_out, v_max, a, b, parts);
    if (n == 0 || n > trapezoid::MAX_PARTS) return false;
    float sum = 0.f, v = v_in;
    for (uint8_t k = 0; k < n; ++k) {
        const trapezoid::Part& p = parts[k];
        if (!(p.distance > 0.f) || p.v_end > v_max + 1e-3f || (p.v_end <= 0.f && k + 1 < n)) return false;
        float acc = (p.v_end * p.v_end - v * v) / (2.f * p.distance);
        if (acc > a * 1.001f || -acc > b * 1.001f) return false;
        sum += p.distance;
        v = p.v_end;
    }
    return std::fabs(sum - d) < 1e-3f * d && std::fabs(v - v_out) < 1e-3f;
}
} // namespace

int main() {
    const float cell = 180.f;
    std::printf("search known straights: 500 -> 1000 -> 500 mm/s, accel 3000\n");
    bool all = true;
    for (int n = 1; n <= 15; ++n) all = all && splitOk(n * cell, 500.f, 500.f, 1000.f, 3000.f, 3000.f);
    check(all, "1 to 15 cells");

    std::printf("fast run: 0 / 1500 / turn speed, accel 3000, decel 3000\n");
    all = true;
    for (float d : {45.f, 90.f, 135.f, 180.f, 270.f, 540.f, 1080.f, 2700.f}) {
        all = all && splitOk(d, 0.f, 500.f, 1500.f, 3000.f, 3000.f) && splitOk(d, 500.f, 500.f, 1500.f, 3000.f, 3000.f)
                  && splitOk(d, 500.f, 0.f, 1500.f, 3000.f, 4000.f);
    }
    check(all, "45 mm to 15 cells");

    std::printf("short straight stays one part at the same speed\n");
    trapezoid::Part parts[trapezoid::MAX_PARTS];
    uint8_t n = trapezoid::split(3.f, 500.f, 500.f, 1000.f, 3000.f, 3000.f, parts);
    check(n == 1 && parts[0].v_end == 500.f && parts[0].distance == 3.f, "3 mm at 500 mm/s");

    std::printf("reverse move: 0 -> 200 -> 0 over 90 mm\n");
    n = trapezoid::split(90.f, 0.f, 0.f, 200.f, 1000.f, 1000.f, parts);
    check(n == 3 && splitOk(90.f, 0.f, 0.f, 200.f, 1000.f, 1000.f), "three parts (accelerate, cruise, stop)");

    if (g_failures > 0) {
        std::printf("test_trapezoid: %d FAILED\n", g_failures);
        return 1;
    }
    std::printf("test_trapezoid: all passed\n");
    return 0;
}
