// DiagEdge（Core/Src/common/diag_edge.cpp）のホストでの単体試験。tools/host_test/run.sh で実行する。
//
// 1kHz で一定速度で斜めの直線を走る機体を模擬する。横のセンサーの値は周期 config::diag::PILLAR_PERIOD_MM で
// 「壁が見える区間（HIGH）→ 柱の切れ目で下がる → 見えない区間（LOW）」を繰り返す波形にし，
// 下がるところで THRESH_OFF_* を横切る位置（切れ目の本当の位置）を決めておく
#include <cmath>
#include <cstdio>
#include <vector>
#include "common/diag_edge.hpp"
#include "config/mouse_config.hpp"

namespace {
using namespace config::diag;

int g_failures = 0;

void check(bool ok, const char* what) {
    std::printf("  [%s] %s\n", ok ? " ok " : "FAIL", what);
    if (!ok) ++g_failures;
}

bool near(float a, float b, float tol) {
    return std::fabs(a - b) <= tol;
}

constexpr float HIGH = 800.f;
constexpr float LOW = 50.f;

// 片側の波形：切れ目（OFF を下回る位置）が edge0 + k·period，その手前 wall_mm の間は壁が見える
struct Wave {
    float edge0;
    float period = PILLAR_PERIOD_MM;
    float wall_mm = 150.f;
    int16_t thresh_off;
    float ramp_mm = 10.f;   // 値が HIGH から LOW へ（LOW から HIGH へ）変わる幅

    float value(float x) const {
        // 切れ目で value = thresh_off になるよう，下がり始め fall0 を決める
        float cross = (HIGH - thresh_off) / (HIGH - LOW) * ramp_mm;
        float u = std::fmod(x - edge0 + cross, period);   // 下がり始めからの距離（0 … period）
        if (u < 0.f) u += period;
        if (u < ramp_mm) return HIGH - (HIGH - LOW) * u / ramp_mm;            // 下がる
        float rise0 = period - wall_mm;                                       // 上がり始め
        if (u < rise0) return LOW;
        if (u < rise0 + ramp_mm) return LOW + (HIGH - LOW) * (u - rise0) / ramp_mm;   // 上がる
        return HIGH;
    }
};

struct Run {
    Wave left{0.f, PILLAR_PERIOD_MM, 150.f, THRESH_OFF_LEFT};
    Wave right{PITCH_MM, PILLAR_PERIOD_MM, 150.f, THRESH_OFF_RIGHT};
    float v = 500.f;
    float x_start = -100.f, x_end = 700.f;
    float noise_amp = 0.f;   // 値に足すノイズ（±，tickごとに符号を変える）
};

struct Sample {
    float x, since_l, since_r;
};

std::vector<Sample> simulate(DiagEdge& d, const Run& r) {
    d.reset();
    d.start();
    std::vector<Sample> out;
    const float dx = r.v * 0.001f;
    int tick = 0;
    for (float x = r.x_start; x <= r.x_end; x += dx, ++tick) {
        float n = (tick % 2 == 0) ? r.noise_amp : -r.noise_amp;
        auto clamp16 = [](float v) { return static_cast<int16_t>(std::lround(v)); };
        d.update(clamp16(r.left.value(x) + n), clamp16(r.right.value(x) + n), x);
        out.push_back({x, d.since(DiagEdge::left), d.since(DiagEdge::right)});
    }
    d.stop();
    return out;
}

// 範囲 [x_start, x_end] に入る切れ目の数
int edgesIn(const Wave& w, float x_start, float x_end) {
    int n = 0;
    for (int k = -10; k < 20; ++k) {
        float e = w.edge0 + k * w.period;
        if (e > x_start && e <= x_end) ++n;
    }
    return n;
}

void testBasic() {
    std::printf("basic: periodic edges on both sides\n");
    DiagEdge d;
    Run r;
    auto s = simulate(d, r);
    check(std::isnan(s.front().since_l) && std::isnan(s.front().since_r), "since is NaN before the first edge");
    check(d.edgeCount(DiagEdge::left) == static_cast<uint32_t>(edgesIn(r.left, r.x_start, r.x_end)), "left edge count");
    check(d.edgeCount(DiagEdge::right) == static_cast<uint32_t>(edgesIn(r.right, r.x_start, r.x_end)), "right edge count");
    // 最後の切れ目の位置（補間）が本当の位置と 0.1 mm 以内
    float last_l = r.left.edge0 + 2.f * r.left.period;     // 509.1
    float last_r = r.right.edge0 + 2.f * r.right.period;   // 636.4
    check(near(d.lastEdge(DiagEdge::left), last_l, 0.1f), "left edge interpolated within 0.1 mm");
    check(near(d.lastEdge(DiagEdge::right), last_r, 0.1f), "right edge interpolated within 0.1 mm");
    // 切れ目の 50 mm 先で since ≈ 50（周期ごとに 0 に戻る）
    bool ok = true;
    for (const Sample& p : s) {
        for (int k = 0; k < 3; ++k) {
            float e = r.left.edge0 + k * r.left.period;
            if (near(p.x, e + 50.f, 0.25f) && !near(p.since_l, 50.f, 0.5f)) ok = false;
        }
    }
    check(ok, "since_l restarts from 0 at every left edge");
}

void testStartOnWall() {
    std::printf("start on a wall: the first edge is counted\n");
    DiagEdge d;
    Run r;
    r.x_start = -60.f;   // 左は壁が見えている途中から（下がり始めは約 −8 mm）
    simulate(d, r);
    check(r.left.value(r.x_start) > THRESH_ON_LEFT, "precondition: left starts above THRESH_ON");
    check(d.edgeCount(DiagEdge::left) == static_cast<uint32_t>(edgesIn(r.left, r.x_start, r.x_end)), "left edge count");
}

void testShortWall() {
    std::printf("short wall: an edge after less than MIN_WALL_MM of wall is ignored\n");
    DiagEdge d;
    Run r;
    r.left.ramp_mm = 2.f;
    r.left.wall_mm = 6.f;   // ON を超えている長さ 約 6.7 mm < MIN_WALL_MM
    r.x_start = 20.f;                                // 最初は壁の見えない区間から
    simulate(d, r);
    check(d.edgeCount(DiagEdge::left) == 0, "no left edges");
    check(d.edgeCount(DiagEdge::right) > 0, "right side still counts");
}

void testNoise() {
    std::printf("noise: +-40 on the value does not add edges (hysteresis)\n");
    DiagEdge d;
    Run r;
    r.noise_amp = 40.f;
    simulate(d, r);
    check(d.edgeCount(DiagEdge::left) == static_cast<uint32_t>(edgesIn(r.left, r.x_start, r.x_end)), "left edge count");
    check(d.edgeCount(DiagEdge::right) == static_cast<uint32_t>(edgesIn(r.right, r.x_start, r.x_end)), "right edge count");
}

void testInactive() {
    std::printf("inactive: reset() clears edges\n");
    DiagEdge d;
    Run r;
    simulate(d, r);
    d.reset();
    check(!d.active(), "not active after reset");
    check(d.edgeCount(DiagEdge::left) == 0 && std::isnan(d.lastEdge(DiagEdge::left)), "edges cleared");
}
} // namespace

int main() {
    testBasic();
    testStartOnWall();
    testShortWall();
    testNoise();
    testInactive();
    std::printf(g_failures == 0 ? "test_diag_edge: PASS\n" : "test_diag_edge: %d FAILED\n", g_failures);
    return g_failures == 0 ? 0 : 1;
}
