// fast_plan::edgeBoundaries（Core/Src/app/fast_plan.cpp）のホストでの単体試験。tools/host_test/run.sh で実行する。
// ターンのパラメータは試験用の値（経路長が分かればよい）
#include <cmath>
#include <cstdio>
#include "app/fast_plan.hpp"
#include "config/mouse_config.hpp"

namespace {
using config::maze::CELL_MM;
using slalom::Anchor;
using slalom::TurnDir;
constexpr float HALF_MM = CELL_MM / 2.f;

int g_failures = 0;

void check(bool ok, const char* what) {
    std::printf("  [%s] %s\n", ok ? " ok " : "FAIL", what);
    if (!ok) ++g_failures;
}

bool near(float a, float b) {
    return std::fabs(a - b) <= 1e-3f;
}

constexpr slalom::Motion M{600.f, 6000.f, 10.f, 15.f};
constexpr slalom::Param S90{"S90_T", "t", 90.f, Anchor::edge, Anchor::edge, 500.f, M, M, false};
constexpr slalom::Param L90{"L90_T", "t", 90.f, Anchor::center, Anchor::center, 500.f, M, M, false};
constexpr slalom::Param T180{"T180_T", "t", 180.f, Anchor::center, Anchor::center, 500.f, M, M, false};
constexpr slalom::Param IN45{"IN45_T", "t", 45.f, Anchor::center, Anchor::diagonal, 500.f, M, M, false};
constexpr slalom::Param OUT45{"OUT45_T", "t", 45.f, Anchor::diagonal, Anchor::center, 500.f, M, M, false};
constexpr slalom::Param V90{"V90_T", "t", 90.f, Anchor::diagonal, Anchor::diagonal, 500.f, M, M, false};
constexpr slalom::Param IN135{"IN135_T", "t", 135.f, Anchor::center, Anchor::diagonal, 500.f, M, M, false};
constexpr slalom::Param OUT135{"OUT135_T", "t", 135.f, Anchor::diagonal, Anchor::center, 500.f, M, M, false};
constexpr DiagonalTurns D{&IN45, &OUT45, &V90, &IN135, &OUT135};
constexpr RunPreset P{"t", 500.f, 500.f, 1500.f, 1000.f, 3000.f, 3000.f, {&S90, &L90, &T180}, &D, false, true,
                      0.f, {nullptr, nullptr, nullptr}, nullptr};

fast_plan::Step straight(float d, bool dia = false) {
    return {nullptr, TurnDir::left, dia, d};
}
fast_plan::Step turn(const slalom::Param& t, TurnDir dir) {
    return {&t, dir, false, 0.f};
}

void testLongShortStraights() {
    std::printf("long / medium / short straights before large turns\n");
    const float d90 = slalom::totalDistance(L90, TurnDir::right);
    fast_plan::Steps steps;
    steps.push_back(straight(900.f));                 // 入口 900
    steps.push_back(turn(L90, TurnDir::right));
    steps.push_back(straight(2.f * CELL_MM));         // 360: 直前の境界だけ検出できる
    steps.push_back(turn(T180, TurnDir::left));
    steps.push_back(straight(CELL_MM));               // 180: 検出する前に入口に着く
    steps.push_back(turn(L90, TurnDir::left));
    steps.push_back(straight(41.f));

    float out[8];
    std::size_t n = fast_plan::edgeBoundaries(steps, P, 2, out, 8);
    const float x2 = 900.f + d90 + 2.f * CELL_MM;
    check(n == 3, "3 boundaries (2 + 1 + 0)");
    check(n >= 1 && near(out[0], 900.f - HALF_MM - CELL_MM), "first turn: entry - 270");
    check(n >= 2 && near(out[1], 900.f - HALF_MM), "first turn: entry - 90");
    check(n >= 3 && near(out[2], x2 - HALF_MM), "second turn: entry - 90 (after the previous turn's length)");

    n = fast_plan::edgeBoundaries(steps, P, 1, out, 8);
    check(n == 2 && near(out[0], 900.f - HALF_MM) && near(out[1], x2 - HALF_MM), "per_turn 1: last boundary only");
    n = fast_plan::edgeBoundaries(steps, P, 0, out, 8);
    check(n == 0, "per_turn 0: none");
    n = fast_plan::edgeBoundaries(steps, P, 2, out, 1);
    check(n == 1 && near(out[0], 900.f - HALF_MM - CELL_MM), "max caps the output");
}

void testOtherTurns() {
    std::printf("small turns, diagonal straights and turn-to-turn are skipped\n");
    fast_plan::Steps steps;
    steps.push_back(straight(900.f));
    steps.push_back(turn(S90, TurnDir::left));        // 小回り：教えない
    steps.push_back(straight(900.f, true));           // 斜めの直線（試験のための形）
    steps.push_back(turn(L90, TurnDir::left));        // 直前が斜め：教えない
    steps.push_back(turn(L90, TurnDir::right));       // 直前がターン：教えない
    steps.push_back(straight(41.f));
    float out[8];
    check(fast_plan::edgeBoundaries(steps, P, 2, out, 8) == 0, "no boundaries");
}

void testDiagonalTurns() {
    std::printf("diagonal turns: IN45 / IN135 (center entry) are used, OUT45 / V90 / OUT135 are not\n");
    const float dia = 3.f * HALF_MM * 1.41421356f;
    fast_plan::Steps steps;
    steps.push_back(straight(900.f));                 // 入口 900
    steps.push_back(turn(IN45, TurnDir::right));
    steps.push_back(straight(dia, true));
    steps.push_back(turn(V90, TurnDir::left));        // 直前が斜め：教えない
    steps.push_back(straight(dia, true));
    steps.push_back(turn(OUT45, TurnDir::right));     // 直前が斜め：教えない
    steps.push_back(straight(3.f * CELL_MM));         // 540
    steps.push_back(turn(IN135, TurnDir::left));
    steps.push_back(straight(dia, true));
    steps.push_back(turn(OUT135, TurnDir::left));     // 直前が斜め：教えない
    steps.push_back(straight(41.f));

    float x = 900.f;
    for (std::size_t i = 1; i <= 6; ++i) {
        x += (steps[i].turn != nullptr) ? slalom::totalDistance(*steps[i].turn, steps[i].dir) : steps[i].distance;
    }
    float out[8];
    std::size_t n = fast_plan::edgeBoundaries(steps, P, 2, out, 8);
    check(n == 4, "4 boundaries (IN45 2 + IN135 2)");
    check(n >= 2 && near(out[0], 900.f - HALF_MM - CELL_MM) && near(out[1], 900.f - HALF_MM), "IN45: entry - 270, - 90");
    check(n >= 4 && near(out[2], x - HALF_MM - CELL_MM) && near(out[3], x - HALF_MM), "IN135: entry - 270, - 90");
}

void testFirstStraightLimit() {
    std::printf("the first straight must be long enough to see the wall before the edge\n");
    // 経路の始めから入口まで 220: 境界 130 の検出 130 − 91 − 20 = 19 < MIN_WALL_MM
    fast_plan::Steps steps;
    steps.push_back(straight(220.f));
    steps.push_back(turn(L90, TurnDir::right));
    steps.push_back(straight(41.f));
    float out[8];
    check(fast_plan::edgeBoundaries(steps, P, 2, out, 8) == 0, "220 mm: none");
    steps[0].distance = 240.f;   // 150 − 111 = 39 ≥ 20
    std::size_t n = fast_plan::edgeBoundaries(steps, P, 2, out, 8);
    check(n == 1 && near(out[0], 150.f), "240 mm: entry - 90");
}
} // namespace

int main() {
    testLongShortStraights();
    testOtherTurns();
    testDiagonalTurns();
    testFirstStraightLimit();
    std::printf("test_fast_plan: %s (%d failures)\n", g_failures == 0 ? "PASS" : "FAIL", g_failures);
    return g_failures == 0 ? 0 : 1;
}
