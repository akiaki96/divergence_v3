// WallEdge（Core/Src/common/wall_edge.cpp）のホストでの単体試験。tools/host_test/run.sh で実行する。
//
// 1kHz で一定速度で走る機体を模擬する。横のセンサーの値は本当の車軸の位置 x_true だけで決め，
// 壁が切れる境界 b では x_true = b + OFFSET_*（config::wall_edge）でちょうど THRESH_OFF_* を下回るようにする
// （LAG_S は 0 の前提）。実測は x_odo = x_true + err で，返ってきた補正を err に足す（Odometry::shiftPositionX と同じ）
#include <cmath>
#include <cstdio>
#include <vector>
#include "common/wall_edge.hpp"
#include "config/mouse_config.hpp"

namespace {
using namespace config::wall_edge;
static_assert(LAG_S == 0.f, "this test models no detection lag");

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
constexpr float RAMP_MM = 8.f;   // 値が HIGH から LOW へ下がる幅

// 片側の横壁：[start, end) の区間に壁がある（end が壁の切れる境界）。値は end + OFFSET の前後 RAMP_MM で下がる
struct Wall {
    float start, end;
};

// 本当の位置 x での値。壁切れの境界 b で OFF を下回る位置が b + offset になるよう，下がり始めをずらす
float sideValue(const std::vector<Wall>& walls, float x, float offset, int16_t thresh_off) {
    // 線形の下がり方：value = HIGH − (HIGH−LOW)·(x − x0)/RAMP_MM。value = thresh_off になる x を b + offset に合わせる
    float cross = (HIGH - thresh_off) / (HIGH - LOW) * RAMP_MM;
    for (const Wall& w : walls) {
        float rise0 = w.start + offset - cross;   // 立ち上がりも同じ形（使わないが連続にする）
        float fall0 = w.end + offset - cross;
        if (x < rise0 || x >= fall0 + RAMP_MM) continue;
        if (x < rise0 + RAMP_MM) return LOW + (HIGH - LOW) * (x - rise0) / RAMP_MM;
        if (x < fall0) return HIGH;
        return HIGH - (HIGH - LOW) * (x - fall0) / RAMP_MM;
    }
    return LOW;
}

struct Run {
    std::vector<Wall> left, right;
    float v = 500.f;
    float x_start = 0.f, x_end = 800.f;
    float err0 = 0.f;            // 実測の初めの誤差（x_odo − x_true）
    float omega = 0.f;           // 目標の角速度（0 以外なら旋回中）
    float noise_amp = 0.f;       // 値に足すノイズ（±，tickごとに符号を変える）
    std::vector<float> boundaries;
    bool correct = true;
    float window = WINDOW_MM;
    float window_late = -1.f;    // 0 以上なら予想より後ろの窓（start(correct, window, window_late)）
};

struct Outcome {
    float err_end;   // 走り終わったときの x_odo − x_true
    std::vector<WallEdge::Event> events;
};

Outcome simulate(WallEdge& we, const Run& r) {
    we.reset();
    for (float b : r.boundaries) we.expect(b);
    if (r.window < 0.f) {
        we.start(r.correct);   // 既定の窓（前後とも WINDOW_MM。探索）
    } else if (r.window_late >= 0.f) {
        we.start(r.correct, r.window, r.window_late);
    } else {
        we.start(r.correct, r.window);
    }
    float err = r.err0;
    int tick = 0;
    for (float x = r.x_start; x < r.x_end; x += r.v * config::control::DT_S, ++tick) {
        float n = (tick % 2 == 0) ? r.noise_amp : -r.noise_amp;
        auto value = [&](const std::vector<Wall>& w, float off, int16_t th) {
            return static_cast<int16_t>(std::lround(sideValue(w, x, off, th) + n));
        };
        float shift = we.update(value(r.left, OFFSET_LEFT_MM, THRESH_OFF_LEFT),
                                value(r.right, OFFSET_RIGHT_MM, THRESH_OFF_RIGHT), x + err, r.v, r.omega);
        err += shift;
    }
    we.stop();
    Outcome o{err, {}};
    for (uint32_t i = 0; i < we.eventCount(); ++i) o.events.push_back(we.event(i));
    return o;
}

WallEdge g_we;   // 記録の配列が大きいので静的に置く

void testCalibRecordsOffset() {
    std::printf("calib: records x - boundary = OFFSET without correcting\n");
    Run r;
    r.left = {{0.f, 312.f}};    // 左は境界 312 で切れる
    r.right = {{0.f, 492.f}};   // 右は境界 492 で切れる
    r.boundaries = {132.f, 312.f, 492.f, 672.f};
    r.err0 = 7.f;
    r.correct = false;
    r.window = 90.f;
    Outcome o = simulate(g_we, r);
    check(o.events.size() == 2, "two edges (left at 312, right at 492)");
    if (o.events.size() == 2) {
        const auto& l = o.events[0];
        const auto& rr = o.events[1];
        check(l.side == WallEdge::left && l.boundary == 312.f, "left edge matched to 312");
        check(near(l.x - l.boundary, OFFSET_LEFT_MM + 7.f, 0.3f), "left offset = OFFSET_LEFT + odometry error");
        check(rr.side == WallEdge::right && rr.boundary == 492.f, "right edge matched to 492");
        check(near(rr.x - rr.boundary, OFFSET_RIGHT_MM + 7.f, 0.3f), "right offset = OFFSET_RIGHT + odometry error");
        check(l.shift == 0.f && rr.shift == 0.f, "no shift in calib");
    }
    check(near(o.err_end, 7.f, 1e-4f), "odometry error untouched");
}

void testCorrectionRemovesError() {
    std::printf("correct: the first edge removes the odometry error, later edges shift ~0\n");
    Run r;
    r.left = {{0.f, 312.f}, {492.f, 672.f}};
    r.right = {{0.f, 492.f}};
    r.boundaries = {132.f, 312.f, 492.f, 672.f};
    r.err0 = 7.f;
    Outcome o = simulate(g_we, r);
    check(o.events.size() == 3, "three edges (L 312, R 492, L 672)");
    if (o.events.size() == 3) {
        check(near(o.events[0].shift, -7.f, 0.3f), "first edge shifts -7 mm");
        check(near(o.events[1].shift, 0.f, 0.3f) && near(o.events[2].shift, 0.f, 0.3f), "later edges shift ~0");
    }
    check(near(o.err_end, 0.f, 0.3f), "odometry error ~0 at the end");
    check(near(g_we.totalShift(), -7.f, 0.3f), "totalShift() = -7 mm");
}

void testOutsideWindowIgnored() {
    std::printf("window: an edge earlier than expected by more than WINDOW_MM is not corrected\n");
    Run r;
    r.left = {{0.f, 312.f}};
    r.boundaries = {312.f};
    r.err0 = -(WINDOW_MM + 5.f);   // 実測が遅れている → 壁切れが予想より前に見える（補正は正）
    r.window = -1.f;               // 既定の窓（start(correct)）
    Outcome o = simulate(g_we, r);
    check(o.events.size() == 1 && std::isnan(o.events[0].boundary), "edge recorded without a boundary");
    check(near(o.err_end, r.err0, 1e-4f), "no correction");
}

void testLateWindowWider() {
    std::printf("late window (fast run): an edge later than expected is corrected up to FAST_WINDOW_LATE_MM\n");
    Run r;
    r.left = {{0.f, 312.f}};
    r.boundaries = {312.f};
    r.window = -1.f;   // 既定（探索）は後ろも WINDOW_MM
    r.err0 = WINDOW_MM + 5.f;
    Outcome o = simulate(g_we, r);
    check(o.events.size() == 1 && std::isnan(o.events[0].boundary), "default (search): late beyond WINDOW_MM not matched");
    check(near(o.err_end, r.err0, 1e-4f), "default (search): no correction");
    r.window = WINDOW_MM;
    r.window_late = FAST_WINDOW_LATE_MM;
    r.err0 = FAST_WINDOW_LATE_MM - 5.f;   // 実測が進みすぎ（機体が足りない）→ 壁切れが予想より後ろに見える（補正は負）
    o = simulate(g_we, r);
    check(o.events.size() == 1 && o.events[0].boundary == 312.f, "matched beyond WINDOW_MM");
    check(near(o.err_end, 0.f, 0.3f), "corrected to ~0");
    r.err0 = FAST_WINDOW_LATE_MM + 5.f;
    o = simulate(g_we, r);
    check(o.events.size() == 1 && std::isnan(o.events[0].boundary), "beyond FAST_WINDOW_LATE_MM: no boundary");
    check(near(o.err_end, r.err0, 1e-4f), "beyond FAST_WINDOW_LATE_MM: no correction");
}

void testTurningIgnored() {
    std::printf("turning: no detection while the target omega is non-zero\n");
    Run r;
    r.left = {{0.f, 312.f}};
    r.boundaries = {312.f};
    r.omega = 100.f;
    Outcome o = simulate(g_we, r);
    check(o.events.empty(), "no edges");
}

void testSlowIgnored() {
    std::printf("slow: no detection below MIN_VELOCITY\n");
    Run r;
    r.left = {{0.f, 312.f}};
    r.boundaries = {312.f};
    r.v = MIN_VELOCITY * 0.5f;
    Outcome o = simulate(g_we, r);
    check(o.events.empty(), "no edges");
}

void testShortWallIgnored() {
    std::printf("short wall: a wall seen for less than MIN_WALL_MM is not used\n");
    Run r;
    r.left = {{312.f - MIN_WALL_MM * 0.5f, 312.f}};
    r.boundaries = {312.f};
    r.err0 = 5.f;
    Outcome o = simulate(g_we, r);
    check(o.events.empty(), "no edges");
    check(near(o.err_end, 5.f, 1e-4f), "no correction");
}

void testBothSidesOneBoundary() {
    std::printf("both sides: left and right ending at the same boundary correct once\n");
    Run r;
    r.left = {{0.f, 312.f}};
    r.right = {{0.f, 312.f}};
    r.boundaries = {312.f};
    r.err0 = 4.f;
    Outcome o = simulate(g_we, r);
    int corrected = 0;
    for (const auto& e : o.events) corrected += (e.shift != 0.f) ? 1 : 0;
    check(corrected == 1, "exactly one correcting edge");
    check(near(o.err_end, 0.f, 0.3f), "odometry error ~0");
}

void testHysteresisNoise() {
    std::printf("noise: +-40 counts of noise does not create extra edges\n");
    Run r;
    r.left = {{0.f, 312.f}};
    r.boundaries = {132.f, 312.f, 492.f};
    r.noise_amp = 40.f;
    r.err0 = 3.f;
    Outcome o = simulate(g_we, r);
    check(o.events.size() == 1, "one edge");
    check(near(o.err_end, 0.f, 1.f), "odometry error within 1 mm");
}

void testPassedBoundariesExpire() {
    std::printf("expiry: boundaries passed without an edge do not catch a later edge\n");
    Run r;
    r.left = {{0.f, 672.f}};   // 132, 312, 492 では切れず，672 で切れる
    r.boundaries = {132.f, 312.f, 492.f, 672.f};
    r.err0 = -6.f;
    Outcome o = simulate(g_we, r);
    check(o.events.size() == 1 && o.events[0].boundary == 672.f, "edge matched to 672");
    check(near(o.err_end, 0.f, 0.3f), "odometry error ~0");
}

void testInjectShiftsPosition() {
    std::printf("inject: boundaries told 10 mm ahead shift the odometry +10 (wall_edge_test inject)\n");
    Run r;
    r.left = {{0.f, 312.f}, {492.f, 672.f}};
    r.boundaries = {142.f, 322.f, 502.f, 682.f};
    Outcome o = simulate(g_we, r);
    check(!o.events.empty() && near(o.events[0].shift, 10.f, 0.3f), "first edge shifts +10 mm");
    check(near(o.err_end, 10.f, 0.3f), "odometry 10 mm ahead of the true position");
}
} // namespace

int main() {
    testCalibRecordsOffset();
    testCorrectionRemovesError();
    testOutsideWindowIgnored();
    testLateWindowWider();
    testTurningIgnored();
    testSlowIgnored();
    testShortWallIgnored();
    testBothSidesOneBoundary();
    testHysteresisNoise();
    testPassedBoundariesExpire();
    testInjectShiftsPosition();
    std::printf("%s (%d failures)\n", g_failures == 0 ? "PASS" : "FAIL", g_failures);
    return g_failures == 0 ? 0 : 1;
}
