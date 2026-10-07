// DiagControl（Core/Src/common/diag_control.cpp）のホストでの単体試験。tools/host_test/run.sh で実行する。
//
// センサーは表（config/diag_table.hpp）そのものを模型にする：切れ目からの距離 since での値 =
// 基準値 + 感度·（その側への寄り）。切れ目は左右とも周期 PILLAR_PERIOD_MM，左右で PITCH_MM ずらす。
// 機体は 1kHz で一定速度 v で走り，向きは回転の目標へ一次遅れ（TAU_S）で追い，横は y' = sin(θ)（左が正）
#include <algorithm>
#include <cmath>
#include <cstdio>
#include "common/diag_control.hpp"
#include "common/diag_edge.hpp"
#include "config/diag_table.hpp"
#include "config/mouse_config.hpp"

namespace {
using config::diag::PILLAR_PERIOD_MM;
using config::diag::PITCH_MM;
namespace table = config::diag_table;

constexpr float DT = config::control::DT_S;
constexpr float TAU_S = 0.02f;   // [s] 向きが目標を追う遅れ（角度の制御）
constexpr float DEG = 3.14159265f / 180.f;

int g_failures = 0;

void check(bool ok, const char* what) {
    std::printf("  [%s] %s\n", ok ? " ok " : "FAIL", what);
    if (!ok) ++g_failures;
}

// 表を since で線形補間（使わない区間も基準値はある）
float tableAt(const table::Entry* t, float since, bool want_sens) {
    float f = std::clamp((since - table::SINCE0_MM) / table::STEP_MM, 0.f, static_cast<float>(table::SIZE - 1));
    auto i = std::min(static_cast<std::size_t>(f), table::SIZE - 2);
    float u = f - static_cast<float>(i);
    float a = want_sens ? t[i].sens : t[i].ref;
    float b = want_sens ? t[i + 1].sens : t[i + 1].ref;
    return a + u * (b - a);
}

// 片側の値。since は切れ目の周期の中の位置，toward はその側への寄り [mm]
float sensorValue(const table::Entry* t, float since, float toward) {
    float sens = tableAt(t, since, true);
    if (sens <= 0.f) {   // 感度を測っていない区間は，近い区間の感度で（模型を連続にするため）
        sens = 2.f;
    }
    return tableAt(t, since, false) + sens * toward;
}

float wrapSince(float x, float edge0) {
    float u = std::fmod(x - edge0, PILLAR_PERIOD_MM);
    return (u < 0.f) ? u + PILLAR_PERIOD_MM : u;
}

struct Sim {
    float v = 500.f;
    float x_start = -150.f;
    float x0 = 0.f, x1 = 8.f * PITCH_MM;   // 斜めの直線（n8 と同じ長さ）
    float y0 = 0.f;                        // [mm] 始めの横のずれ（左が正）
    float inject_deg = 0.f;
    bool correction = true;
    bool left_wall = true;                 // false：左の壁がすべて抜けている（値は低いまま，切れ目なし）
    bool hold_y = false;                   // true：横のずれを y0 のまま動かさない（推定だけを見る）
    float edge0_left = 20.f;               // [mm] 左の切れ目の位相（x0 からの位置）
};

struct Result {
    float y_end = 0.f;
    float y_max = 0.f;          // |y| の最大
    float lateral_end = 0.f;    // DiagControl の推定（終わり）
    float lateral_mean = 0.f;   // 後半の平均
    uint32_t measured = 0, active = 0;
};

Result run(const Sim& s) {
    DiagEdge edge;
    DiagControl ctl;
    edge.reset();
    ctl.reset();
    DiagControl::Range r{s.x0, s.x1};
    ctl.setRanges(&r, 1);
    ctl.injectAngle(s.inject_deg);
    edge.start();
    ctl.start(s.correction);

    Result res;
    float x = s.x_start, y = s.y0, theta = 0.f;
    float lat_sum = 0.f;
    int lat_n = 0;
    while (x < s.x1 + 20.f) {
        float l = s.left_wall ? sensorValue(table::LEFT, wrapSince(x, s.x0 + s.edge0_left), y) : 80.f;
        float rr = sensorValue(table::RIGHT, wrapSince(x, s.x0 + s.edge0_left + PITCH_MM), -y);
        auto vl = static_cast<int16_t>(l);
        auto vr = static_cast<int16_t>(rr);
        edge.update(vl, vr, x);
        AxisReference ref = ctl.apply({0.f, 0.f, 0.f}, 0.f, s.v, x, vl, vr, edge);
        theta += (ref.pos - theta) * DT / TAU_S;
        x += s.v * DT;
        if (!s.hold_y) y += s.v * DT * std::sin(theta * DEG);
        if (x >= s.x0) res.y_max = std::max(res.y_max, std::fabs(y));
        if (x > s.x0 + 0.5f * (s.x1 - s.x0) && x < s.x1) {
            lat_sum += ctl.lateral();
            ++lat_n;
        }
        if (x < s.x1) res.lateral_end = ctl.lateral();
    }
    ctl.stop();
    res.y_end = y;
    res.lateral_mean = lat_n > 0 ? lat_sum / lat_n : 0.f;
    res.measured = ctl.measuredTicks();
    res.active = ctl.activeTicks();
    return res;
}

void testSideOffset() {
    std::printf("sideOffset\n");
    std::size_t i = 0;
    while (i < table::SIZE && table::LEFT[i].sens <= 0.f) ++i;
    std::size_t j = i + 10;
    float since = table::SINCE0_MM + j * table::STEP_MM;
    float ref = table::LEFT[j].ref, sens = table::LEFT[j].sens;
    check(std::fabs(DiagControl::sideOffset(DiagEdge::left, since, static_cast<int16_t>(ref))) < 0.2f,
          "the reference value reads 0 mm");
    check(std::fabs(DiagControl::sideOffset(DiagEdge::left, since, static_cast<int16_t>(std::lround(ref + 3.f * sens))) - 3.f)
              < 0.2f,
          "reference + 3 x sensitivity reads 3 mm toward that side");
    check(DiagControl::sideOffset(DiagEdge::left, since, static_cast<int16_t>(ref - 2.f * sens)) < -1.5f,
          "below the reference reads negative (away)");
    check(std::isnan(DiagControl::sideOffset(DiagEdge::left, table::SINCE0_MM, 500)), "the 'do not use' head is NaN");
    check(std::isnan(DiagControl::sideOffset(DiagEdge::left, 400.f, 500)), "past the table is NaN");
    check(std::isnan(DiagControl::sideOffset(DiagEdge::left, NAN, 500)), "no edge yet (NaN) is NaN");
}

void testGating() {
    std::printf("gating\n");
    DiagEdge edge;
    edge.reset();
    DiagControl ctl;
    ctl.reset();
    DiagControl::Range r{0.f, 500.f};
    ctl.setRanges(&r, 1);
    AxisReference in{12.f, 34.f, 56.f};
    AxisReference out = ctl.apply(in, 0.f, 500.f, 100.f, 2000, 2000, edge);
    check(out.pos == in.pos && out.vel == in.vel && out.acc == in.acc, "stopped: the reference passes unchanged");

    // 斜めの直線より前（x0 − 150）の切れ目しかない：使わない
    edge.start();
    edge.update(1500, 1500, -170.f);
    edge.update(100, 100, -150.f);
    check(edge.edgeCount(DiagEdge::left) == 1, "(setup) one edge before the diagonal");
    ctl.start(true);
    for (float x = 0.f; x < 100.f; x += 0.5f) {
        edge.update(100, 100, x);   // 低いまま：切れ目は増えない
        ctl.apply(in, 0.f, 500.f, x, 100, 100, edge);
    }
    check(ctl.measuredTicks() == 0, "an edge from before the diagonal (turn approach) is not used");

    DiagControl c2;
    c2.reset();
    c2.setRanges(&r, 1);
    c2.injectAngle(1.f);
    c2.start(true);
    out = c2.apply(in, 0.f, 500.f, -10.f, 2000, 2000, edge);
    check(out.pos == in.pos && c2.activeTicks() == 0, "before the range: nothing added");
    out = c2.apply(in, 300.f, 500.f, 10.f, 2000, 2000, edge);
    check(out.pos == in.pos && c2.activeTicks() == 0, "turning (omega target != 0): nothing added");
    out = c2.apply(in, 0.f, 50.f, 10.f, 2000, 2000, edge);
    check(c2.activeTicks() == 0, "slower than MIN_VELOCITY: nothing added");
    out = c2.apply(in, 0.f, 500.f, 10.f, 2000, 2000, edge);
    check(std::fabs(out.pos - in.pos - 1.f) < 1e-4f && c2.offset() == 0.f,
          "entering the range applies the injected 1 deg outside the control offset");
    out = c2.apply(in, 0.f, 500.f, 600.f, 2000, 2000, edge);
    check(c2.activeTicks() == 1 && std::fabs(out.pos - in.pos - 1.f) < 1e-4f, "after the range: offsets kept, no update");
}

void testClosedLoop() {
    std::printf("closed loop (table as the sensor model, n8 = %.0f mm at 500 mm/s)\n", 8.f * PITCH_MM);
    Sim s;
    s.inject_deg = 1.f;
    s.correction = false;
    Result open = run(s);
    std::printf("    1 deg, no correction: y end %+.1f mm, estimate end %+.1f mm\n", open.y_end, open.lateral_end);
    check(open.y_end > 12.f, "without correction, 1 deg drifts > 12 mm left");
    check(open.lateral_end > 6.f, "the estimate follows the drift (observe only)");

    s.correction = true;
    Result closed = run(s);
    std::printf("    1 deg, correction: y end %+.1f mm, max |y| %.1f mm, measured %u / %u ticks\n", closed.y_end,
                closed.y_max, static_cast<unsigned>(closed.measured), static_cast<unsigned>(closed.active));
    check(std::fabs(closed.y_end) < 3.f, "with correction, 1 deg ends within 3 mm");
    check(closed.y_max < 8.f, "with correction, |y| stays below 8 mm");

    s.inject_deg = -1.f;
    Result neg = run(s);
    std::printf("    -1 deg, correction: y end %+.1f mm, max |y| %.1f mm\n", neg.y_end, neg.y_max);
    check(std::fabs(neg.y_end) < 3.f && neg.y_max < 8.f, "the same to the right");

    Sim off;
    off.y0 = 5.f;
    Result rec = run(off);
    std::printf("    start 5 mm left, correction: y end %+.1f mm\n", rec.y_end);
    check(std::fabs(rec.y_end) < 2.5f, "a 5 mm start offset comes back within 2.5 mm");

    Sim zero;
    Result z = run(zero);
    std::printf("    centered: y end %+.2f mm, max |y| %.2f mm\n", z.y_end, z.y_max);
    check(z.y_max < 0.5f, "centered: stays centered");
}

void testMissingWalls() {
    std::printf("missing walls (left side all open, observe only, y held)\n");
    Sim s;
    s.left_wall = false;
    s.correction = false;
    s.hold_y = true;
    Result o = run(s);
    std::printf("    centered: estimate %+.2f mm\n", o.lateral_mean);
    check(std::fabs(o.lateral_mean) < 0.3f, "an open side is not read as 'far away' (no pull toward it)");
    s.y0 = 3.f;   // 左へ 3 mm：右が離れて読める（MAX_AWAY_MM 以内）
    Result a = run(s);
    std::printf("    3 mm left: estimate %+.2f mm\n", a.lateral_mean);
    check(std::fabs(a.lateral_mean - 3.f) < 0.6f, "3 mm left is read from the right side alone");
    s.y0 = -3.f;
    Result b = run(s);
    std::printf("    3 mm right: estimate %+.2f mm\n", b.lateral_mean);
    check(std::fabs(b.lateral_mean + 3.f) < 0.6f, "3 mm right is read from the right side alone");
    s.y0 = 8.f;   // 右が MAX_AWAY_MM より離れる：右も使えない → 読めない
    Result c = run(s);
    std::printf("    8 mm left: estimate %+.2f mm, measured %u / %u ticks\n", c.lateral_mean,
                static_cast<unsigned>(c.measured), static_cast<unsigned>(c.active));
    check(c.measured < c.active / 4, "farther than MAX_AWAY_MM on the only walled side: mostly not measured");

    Sim both;
    both.correction = false;
    both.hold_y = true;
    both.y0 = 3.f;
    Result d = run(both);
    std::printf("    walls on both sides, 3 mm left: estimate %+.2f mm\n", d.lateral_mean);
    check(std::fabs(d.lateral_mean - 3.f) < 0.6f, "both sides: about 3 mm");
}
} // namespace

int main() {
    testSideOffset();
    testGating();
    testClosedLoop();
    testMissingWalls();
    if (g_failures > 0) {
        std::printf("test_diag_control: %d FAILED\n", g_failures);
        return 1;
    }
    std::printf("test_diag_control: all PASS\n");
    return 0;
}
