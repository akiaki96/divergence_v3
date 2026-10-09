// fast_plan::edgeBoundaries（Core/Src/app/fast_plan.cpp）のホストでの単体試験。tools/host_test/run.sh で実行する。
// ターンのパラメータは試験用の値（経路長が分かればよい）
#include <cmath>
#include <cstdio>
#include <initializer_list>
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
template <std::size_t N>
constexpr TurnLadder ladder(const slalom::Param* const (&list)[N]) {
    return {list, static_cast<uint8_t>(N)};
}
constexpr const slalom::Param* LS90[] = {&S90};
constexpr const slalom::Param* LL90[] = {&L90};
constexpr const slalom::Param* LT180[] = {&T180};
constexpr const slalom::Param* LIN45[] = {&IN45};
constexpr const slalom::Param* LOUT45[] = {&OUT45};
constexpr const slalom::Param* LV90[] = {&V90};
constexpr const slalom::Param* LIN135[] = {&IN135};
constexpr const slalom::Param* LOUT135[] = {&OUT135};
// RunPreset::turns の並び：L90, T180, IN45, OUT45, IN135, OUT135, V90, S90（TurnKind）
constexpr RunPreset P{"t", 500.f, 1500.f, 1000.f, 3000.f, 3000.f,
                      {ladder(LL90), ladder(LT180), ladder(LIN45), ladder(LOUT45), ladder(LIN135), ladder(LOUT135),
                       ladder(LV90), ladder(LS90)},
                      true, false, true, false};

// 速度の違うターン（fitSpeeds の試験）。経路長は使わない
constexpr slalom::Param L90_1500{"L90_1500", "t", 90.f, Anchor::center, Anchor::center, 1500.f, M, M, false};
constexpr slalom::Param L90_1200{"L90_1200", "t", 90.f, Anchor::center, Anchor::center, 1200.f, M, M, false};
constexpr slalom::Param L90_900{"L90_900", "t", 90.f, Anchor::center, Anchor::center, 900.f, M, M, false};
constexpr slalom::Param T180_1200{"T180_1200", "t", 180.f, Anchor::center, Anchor::center, 1200.f, M, M, false};
constexpr slalom::Param T180_900{"T180_900", "t", 180.f, Anchor::center, Anchor::center, 900.f, M, M, false};
constexpr slalom::Param S90_900{"S90_900", "t", 90.f, Anchor::edge, Anchor::edge, 900.f, M, M, false};
constexpr const slalom::Param* ML90[] = {&L90_1500, &L90_1200, &L90_900};
constexpr const slalom::Param* MT180[] = {&T180_1200, &T180_900};
constexpr const slalom::Param* MS90[] = {&S90_900};
constexpr RunPreset PM{"m", 1500.f, 2000.f, 1500.f, 4000.f, 4000.f,
                       {ladder(ML90), ladder(MT180), {nullptr, 0}, {nullptr, 0}, {nullptr, 0}, {nullptr, 0}, {nullptr, 0},
                        ladder(MS90)},
                       false, false, true, false};

fast_plan::Step straight(float d, bool dia = false) {
    return {nullptr, TurnDir::left, dia, TURN_L90, d};
}
fast_plan::Step turn(const slalom::Param& t, TurnDir dir, TurnKind kind = TURN_L90) {
    return {&t, dir, false, kind, 0.f};
}

// fitSpeeds の後の各ターンの速度（直線は飛ばす）
bool speedsAre(const fast_plan::Steps& steps, std::initializer_list<float> want) {
    auto it = want.begin();
    for (const auto& s : steps) {
        if (s.turn == nullptr) continue;
        if (it == want.end() || !near(s.turn->speed, *it)) return false;
        ++it;
    }
    return it == want.end();
}

void testFitSpeeds() {
    std::printf("fitSpeeds: turns are lowered where the straight between them is too short\n");
    using fast_plan::Error;
    // 基準は機体の上限（加速 A，減速 B）。v から w へ距離 d：|v² − w²| ≤ 2·A·d（または 2·B·d）
    constexpr float A = config::profile_limit::MAX_ACCEL_X;   // 約 14710
    constexpr float B = config::profile_limit::MAX_DECEL_X;   // 約 19613
    static_assert(1500.f * 1500.f - 1200.f * 1200.f <= 2.f * B * 360.f, "the long-straight case must fit");
    {
        fast_plan::Steps s;
        s.push_back(straight(1000.f));
        s.push_back(turn(L90_1500, TurnDir::right, TURN_L90));
        s.push_back(straight(360.f));
        s.push_back(turn(T180_1200, TurnDir::left, TURN_180));
        s.push_back(straight(1000.f));
        check(fast_plan::fitSpeeds(&s, PM) == Error::none && speedsAre(s, {1500.f, 1200.f}), "long straights: unchanged");
    }
    {
        // 直線なしで続く：速い L90 1500 を T180 と同じ 1200 に
        fast_plan::Steps s;
        s.push_back(straight(1000.f));
        s.push_back(turn(L90_1500, TurnDir::right, TURN_L90));
        s.push_back(turn(T180_1200, TurnDir::left, TURN_180));
        s.push_back(straight(1000.f));
        check(fast_plan::fitSpeeds(&s, PM) == Error::none && speedsAre(s, {1200.f, 1200.f}), "turn-to-turn: equal speeds");
    }
    {
        // S90 900 の後 30mm で L90：√(900² + 2·A·30) ≈ 1301 → L90 1200
        static_assert(1200.f * 1200.f <= 900.f * 900.f + 2.f * A * 30.f && 1500.f * 1500.f > 900.f * 900.f + 2.f * A * 30.f);
        fast_plan::Steps s;
        s.push_back(straight(1000.f));
        s.push_back(turn(S90_900, TurnDir::left, TURN_S90));
        s.push_back(straight(30.f));
        s.push_back(turn(L90_1500, TurnDir::right, TURN_L90));
        s.push_back(straight(1000.f));
        check(fast_plan::fitSpeeds(&s, PM) == Error::none && speedsAre(s, {900.f, 1200.f}), "30 mm after S90 900: L90 1200");
    }
    {
        // スタートから 20mm で L90：√(2·A·20) ≈ 767 < 900（いちばん遅い候補）→ speedUnfit
        static_assert(900.f * 900.f > 2.f * A * 20.f && 900.f * 900.f <= 2.f * A * 40.f && 1200.f * 1200.f > 2.f * A * 40.f);
        fast_plan::Steps s;
        s.push_back(straight(20.f));
        s.push_back(turn(L90_1500, TurnDir::right, TURN_L90));
        s.push_back(straight(1000.f));
        check(fast_plan::fitSpeeds(&s, PM) == Error::speedUnfit, "start too short even for the slowest: speedUnfit");
        // 40mm なら √(2·A·40) ≈ 1085 → 900
        s[0].distance = 40.f;
        s[1].turn = &L90_1500;
        check(fast_plan::fitSpeeds(&s, PM) == Error::none && speedsAre(s, {900.f}), "start 40 mm: L90 900");
    }
    {
        // 連鎖：ゴールまで 25mm で止まる：√(2·B·25) ≈ 990 → T180 900，直線なしで続く L90 も 900
        static_assert(900.f * 900.f > 2.f * B * 10.f && 900.f * 900.f <= 2.f * B * 25.f && 1200.f * 1200.f > 2.f * B * 25.f);
        fast_plan::Steps s;
        s.push_back(straight(1000.f));
        s.push_back(turn(L90_1500, TurnDir::right, TURN_L90));
        s.push_back(turn(T180_1200, TurnDir::left, TURN_180));
        s.push_back(straight(10.f));                   // 10mm：√(2·B·10) ≈ 626 → 900 にも落とせない
        check(fast_plan::fitSpeeds(&s, PM) == Error::speedUnfit, "goal too close after the turn: speedUnfit");
        s[3].distance = 25.f;
        s[1].turn = &L90_1500;
        s[2].turn = &T180_1200;
        check(fast_plan::fitSpeeds(&s, PM) == Error::none && speedsAre(s, {900.f, 900.f}), "chain: both lowered to 900");
    }
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
    std::size_t n = fast_plan::edgeBoundaries(steps, P, 2, false, out, 8);
    const float x2 = 900.f + d90 + 2.f * CELL_MM;
    check(n == 3, "3 boundaries (2 + 1 + 0)");
    check(n >= 1 && near(out[0], 900.f - HALF_MM - CELL_MM), "first turn: entry - 270");
    check(n >= 2 && near(out[1], 900.f - HALF_MM), "first turn: entry - 90");
    check(n >= 3 && near(out[2], x2 - HALF_MM), "second turn: entry - 90 (after the previous turn's length)");

    n = fast_plan::edgeBoundaries(steps, P, 1, false, out, 8);
    check(n == 2 && near(out[0], 900.f - HALF_MM) && near(out[1], x2 - HALF_MM), "per_turn 1: last boundary only");
    n = fast_plan::edgeBoundaries(steps, P, 0, false, out, 8);
    check(n == 0, "per_turn 0: none");
    n = fast_plan::edgeBoundaries(steps, P, 2, false, out, 1);
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
    check(fast_plan::edgeBoundaries(steps, P, 2, false, out, 8) == 0, "no boundaries");
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
    std::size_t n = fast_plan::edgeBoundaries(steps, P, 2, false, out, 8);
    check(n == 4, "4 boundaries (IN45 2 + IN135 2)");
    check(n >= 2 && near(out[0], 900.f - HALF_MM - CELL_MM) && near(out[1], 900.f - HALF_MM), "IN45: entry - 270, - 90");
    check(n >= 4 && near(out[2], x - HALF_MM - CELL_MM) && near(out[3], x - HALF_MM), "IN135: entry - 270, - 90");

    // 斜めの直線の範囲（DiagControl に教える）：ターンの出口の基準点から次のターンの入口の基準点まで
    std::printf("diagonal ranges: the three diagonal straights, from the turn exit to the next entry\n");
    float x0[8], x1[8];
    float a = 900.f + slalom::totalDistance(IN45, TurnDir::right);
    float b = a + dia + slalom::totalDistance(V90, TurnDir::left);
    float c = x + slalom::totalDistance(IN135, TurnDir::left);
    std::size_t nd = fast_plan::diagonalRanges(steps, x0, x1, 8);
    check(nd == 3, "3 diagonal straights");
    check(nd >= 1 && near(x0[0], a) && near(x1[0], a + dia), "after IN45");
    check(nd >= 2 && near(x0[1], b) && near(x1[1], b + dia), "after V90");
    check(nd >= 3 && near(x0[2], c) && near(x1[2], c + dia), "after IN135");
    check(fast_plan::diagonalRanges(steps, x0, x1, 2) == 2, "max caps the output");

    // 縦横の直線の範囲（WallControl に教える）：斜めの直線とスラロームのオフセットは含まない
    std::printf("orthogonal ranges: the three orthogonal straights, without the slalom offsets\n");
    float d = x + slalom::totalDistance(IN135, TurnDir::left) + dia + slalom::totalDistance(OUT135, TurnDir::left);
    float o0[8], o1[8];
    std::size_t no = fast_plan::orthogonalRanges(steps, o0, o1, 8);
    check(no == 3, "3 orthogonal straights");
    check(no >= 1 && near(o0[0], 0.f) && near(o1[0], 900.f), "first straight: 0 - entry of IN45");
    check(no >= 2 && near(o0[1], x - 3.f * CELL_MM) && near(o1[1], x), "after OUT45: exit - entry of IN135");
    check(no >= 3 && near(o0[2], d) && near(o1[2], d + 41.f), "after OUT135");
    // ターンからターンへ（長さ0の直線）は範囲にしない
    fast_plan::Steps tt;
    tt.push_back(straight(360.f));
    tt.push_back(turn(OUT135, TurnDir::left));
    tt.push_back(straight(0.f));
    tt.push_back(turn(L90, TurnDir::right));
    check(fast_plan::orthogonalRanges(tt, o0, o1, 8) == 1, "a zero-length straight is not a range");
}

void testFirstStraightLimit() {
    std::printf("the first straight must be long enough to see the wall before the edge\n");
    // 経路の始めから入口まで 220: 境界 130 の検出 130 − 91 − 20 = 19 < MIN_WALL_MM
    fast_plan::Steps steps;
    steps.push_back(straight(220.f));
    steps.push_back(turn(L90, TurnDir::right));
    steps.push_back(straight(41.f));
    float out[8];
    check(fast_plan::edgeBoundaries(steps, P, 2, false, out, 8) == 0, "220 mm: none");
    steps[0].distance = 240.f;   // 150 − 111 = 39 ≥ 20
    std::size_t n = fast_plan::edgeBoundaries(steps, P, 2, false, out, 8);
    check(n == 1 && near(out[0], 150.f), "240 mm: entry - 90");
}
void testCenterBoundary() {
    std::printf("center: the boundary half a cell past the entry (seen when the axle reaches the cell center)\n");
    const float d90 = slalom::totalDistance(L90, TurnDir::right);
    const float d180 = slalom::totalDistance(T180, TurnDir::left);
    fast_plan::Steps steps;
    steps.push_back(straight(900.f));                 // 入口 900：手前 2 + 入口 + 90
    steps.push_back(turn(L90, TurnDir::right));
    steps.push_back(straight(CELL_MM));               // 180：手前はなし，入口 + 90 だけ
    steps.push_back(turn(T180, TurnDir::left));
    steps.push_back(straight(40.f));                  // 40：入口 + 90 の窓の始まり（入口 − 21）− 20 が直線の前 → なし
    steps.push_back(turn(L90, TurnDir::left));
    steps.push_back(straight(41.f));                  // 41：ちょうど入る
    steps.push_back(turn(L90, TurnDir::right));
    steps.push_back(straight(41.f));

    const float x2 = 900.f + d90 + CELL_MM;
    const float x3 = x2 + d180 + 40.f;
    const float x4 = x3 + d90 + 41.f;
    float out[8];
    std::size_t n = fast_plan::edgeBoundaries(steps, P, 2, true, out, 8);
    check(n == 5, "5 boundaries (2 + 1 + 1, 0, 1)");
    check(n >= 3 && near(out[0], 900.f - HALF_MM - CELL_MM) && near(out[1], 900.f - HALF_MM) &&
              near(out[2], 900.f + HALF_MM), "first turn: entry - 270, - 90, + 90 (ascending)");
    check(n >= 4 && near(out[3], x2 + HALF_MM), "1-cell straight: entry + 90 only");
    check(n >= 5 && near(out[4], x4 + HALF_MM), "41 mm straight: entry + 90 (40 mm: none)");
    (void)x3;
    check(fast_plan::edgeBoundaries(steps, P, 2, false, out, 8) == 2, "center false: entry - 270, - 90 only");
    check(fast_plan::edgeBoundaries(steps, P, 0, true, out, 8) == 3, "per_turn 0: entry + 90 only");
    check(fast_plan::edgeBoundaries(steps, P, 2, true, out, 2) == 2, "max caps the output");

    // 小回り・斜めから入るターン・ターンの直後のターンには，入口 + 90 も教えない
    fast_plan::Steps other;
    other.push_back(straight(900.f));
    other.push_back(turn(S90, TurnDir::left));
    other.push_back(straight(900.f, true));
    other.push_back(turn(OUT45, TurnDir::right));
    other.push_back(turn(L90, TurnDir::right));
    other.push_back(straight(41.f));
    check(fast_plan::edgeBoundaries(other, P, 2, true, out, 8) == 0, "S90 / OUT45 / turn-to-turn: none");
}
} // namespace

int main() {
    testLongShortStraights();
    testOtherTurns();
    testDiagonalTurns();
    testFirstStraightLimit();
    testCenterBoundary();
    testFitSpeeds();
    std::printf("test_fast_plan: %s (%d failures)\n", g_failures == 0 ? "PASS" : "FAIL", g_failures);
    return g_failures == 0 ? 0 : 1;
}
