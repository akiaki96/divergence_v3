// 前壁の読み落とし・自己位置のずれの手がかり（search_lookahead::redoWithFrontWall / shiftMatch，
// front_correction::bothCloserThan / frontWallMm）のホストでの単体試験。tools/host_test/run.sh で実行する。
//
// 2026-10-06 の探索 700_s1100_4（ゴール (1,0)）を再現する：step 13 で前壁を読み落として (2,4) の南の壁にぶつかり，
// 行き止まりを (2,3) と思って戻った後，機体は実際には1区画先にいた。step 15・16 で読み直しても地図と食い違った
// 壁は，1区画先の区画の地図と一致する。ログの壁（ソルバーに渡した左・前・右）をそのまま渡して確かめる
#include <cmath>
#include <cstdio>
#include <random>
#include "adachi.hpp"
#include "adachi_return.hpp"
#include "app/search_lookahead.hpp"
#include "common/front_correction.hpp"
#include "config/mouse_config.hpp"

namespace {
int g_failures = 0;

void check(bool ok, const char* what) {
    std::printf("  [%s] %s\n", ok ? " ok " : "FAIL", what);
    if (!ok) ++g_failures;
}

struct LogStep {
    uint8_t x, y, dir;
    uint8_t walls;    // bit0: 左, bit1: 前, bit2: 右（ソルバーに渡した壁）
    uint8_t action;
};

// tools/log/search/700_s1100_4.csv の step 0〜14（x, y, dir, 左前右, action）
constexpr LogStep LOG[] = {
    {0, 1, 2, 0b101, 45}, {0, 2, 2, 0b101, 45}, {0, 3, 2, 0b101, 45}, {0, 4, 2, 0b001, 47},
    {1, 4, 0, 0b010, 47}, {1, 3, 6, 0b101, 45}, {1, 2, 6, 0b111, 48}, {1, 3, 2, 0b101, 45},
    {1, 4, 2, 0b100, 45}, {1, 5, 2, 0b101, 45}, {1, 6, 2, 0b010, 47}, {2, 6, 0, 0b001, 47},
    {2, 5, 6, 0b101, 45}, {2, 4, 6, 0b100, 45}, {2, 3, 6, 0b111, 48},
};

void init() {
    solver_options_reset();
    solver_options.goal_x = 1;
    solver_options.goal_y = 0;
    adachi_return::solver_adachi_return_init();
}

// LOG の先頭 n 歩をソルバーに渡す。位置と動作がログと同じなら true
bool replay(int n) {
    bool same = true;
    for (int i = 0; i < n; ++i) {
        const LogStep& s = LOG[i];
        search_lookahead::prepare();
        same = same && mousePos.x == s.x && mousePos.y == s.y && mousePos.dir == s.dir;
        uint8_t a = search_lookahead::take((s.walls & 1) != 0, (s.walls & 2) != 0, (s.walls & 4) != 0);
        same = same && a == s.action;
    }
    return same;
}

uint8_t mapBits() {
    bool l = false, f = false, r = false;
    if (!search_lookahead::knownWalls(&l, &f, &r)) return 0xFF;
    return search_lookahead::wallBits(l, f, r);
}

void testShiftFromLog() {
    std::printf("position shift (700_s1100_4 step 15-16)\n");
    init();
    check(replay(15), "steps 0-14 replay with the logged positions and actions");
    // step 15：(2,4) 北向き。地図は 100，読み直しても 101（1区画先 (2,5) の北向きの地図が 101）
    check(mousePos.x == 2 && mousePos.y == 4 && mousePos.dir == Nth, "step 15 at (2,4) facing north");
    check(mapBits() == 0b001, "step 15 map is 100 (left only)");
    // 1区画手前 (2,3) も（読み落としで作った行き止まりが）北向きで 101 なので両方と一致する
    check(search_lookahead::shiftMatch(0b101) == (search_lookahead::SHIFT_AHEAD | search_lookahead::SHIFT_BEHIND),
          "step 15 sensor 101 matches the cell ahead (and the fake dead end behind)");
    check(search_lookahead::shiftMatch(0b001) == 0, "the map's own walls match no shifted cell");
    search_lookahead::prepare();
    uint8_t a = search_lookahead::take(true, false, true);   // ファームはセンサーの壁で進んだ（地図を書き換えた）
    check(a == ACT_MOVE_1CELL, "step 15 action is move");
    // step 16：(2,5) 北向き。地図は 101，読み直しても 010（1区画先 (2,6) の北向きの地図が 010）
    check(mousePos.x == 2 && mousePos.y == 5, "step 16 at (2,5)");
    check(mapBits() == 0b101, "step 16 map is 101");
    check(search_lookahead::shiftMatch(0b010) == search_lookahead::SHIFT_AHEAD,
          "step 16 sensor 010 matches only the cell ahead: second vote ahead -> position lost");
}

void testShiftBehind() {
    std::printf("position shift behind\n");
    init();
    replay(3);   // (0,4) 北向きで読む前。(0,3) は北向きで 101 と分かっている
    check(mousePos.x == 0 && mousePos.y == 4 && mousePos.dir == Nth, "at (0,4) facing north");
    check(search_lookahead::shiftMatch(0b101) == search_lookahead::SHIFT_BEHIND, "walls of the cell behind");
    check(search_lookahead::shiftMatch(0b111) == 0, "walls matching no cell give 0");
}

void testRedoMissedWall() {
    std::printf("redo with the missed front wall (700_s1100_4 step 13)\n");
    init();
    check(replay(14), "steps 0-13 replay (step 13 read the front as open and moved)");
    check(mousePos.x == 2 && mousePos.y == 3 && mousePos.dir == Sth, "solver moved on to (2,3)");
    uint8_t a = search_lookahead::redoWithFrontWall(false, true);   // step 13 の左右（左なし・右あり）と前壁あり
    check(get_wall_abs(&wallzero, 2, 4, Sth) && get_wall_abs(&wallone, 2, 4, Sth), "south wall of (2,4) is now set");
    check(get_wall_abs(&wallzero, 2, 4, Wst) && !get_wall_abs(&wallone, 2, 4, Est), "left/right of (2,4) kept");
    // (2,4) 南向き：左（東）だけ開いている → 左に曲がって (3,4) 東向き
    check(a == ACT_TURN_LEFT_MOVE, "redo turns left (east is the only opening)");
    check(mousePos.x == 3 && mousePos.y == 4 && mousePos.dir == Est, "solver moved to (3,4) facing east");
    check(!search_lookahead::ready(), "lookahead must be prepared again");
}

void testFrontNear() {
    std::printf("front wall too close (config::search::MISSED_WALL_NEAR_MM)\n");
    const float near = config::search::MISSED_WALL_NEAR_MM;
    // 700_s1100_4 step 14（ぶつかった後）と，読み落としの読み（step 13）
    check(front_correction::bothCloserThan(2876, 1735, near), "step 14 (2876/1735) is too close");
    check(!front_correction::bothCloserThan(338, 203, near), "step 13 (338/203, wall one cell ahead) is not");
    // これまでのログの直進の後の読みで最も近かった正常な読み（500_fc_goal10_6 step 9：953/643 ≈ 151 mm）
    check(!front_correction::bothCloserThan(953, 643, near), "closest normal read (953/643) is not");
    // 片方だけ近い（斜めの壁・柱）：500_goal10_7 step 7（1258/547）
    check(!front_correction::bothCloserThan(1258, 547, near), "only FL close (1258/547) is not");
    check(front_correction::bothCloserThan(4000, 4000, near), "above the table (closer than the table) is too close");
    float d = front_correction::frontWallMm(2876, 1735);
    check(d > 70.f && d < 90.f, "front wall at step 14 is about 80 mm");
    check(std::isnan(front_correction::frontWallMm(50, 50)), "beyond the table is NaN");
}

// ---- 乱数の迷路での見積もり（search.cpp の runSteps と同じ壁の使い方・ずれの数え方）----
// 本当の迷路（外周とスタート区画の東は壁，ほかは乱数）
Wall makeMaze(std::mt19937& rng, float density) {
    Wall w;
    wall_reset_zero(&w);
    std::bernoulli_distribution has_wall(density);
    for (uint8_t y = 0; y < MAZE_SIZE; ++y) {
        for (uint8_t x = 0; x < MAZE_SIZE; ++x) {
            if (has_wall(rng)) set_wall(&w, x, y, Est, true);
            if (has_wall(rng)) set_wall(&w, x, y, Nth, true);
        }
    }
    set_wall(&w, 0, 0, Est, true);
    set_wall(&w, 0, 0, Nth, false);
    return w;
}

enum class End { finished, positionLost, crashed, other };

struct Physical {
    int x, y;
    AbsDir dir;
};

void forward(Physical* p) {
    p->x += (p->dir == Est) ? 1 : (p->dir == Wst) ? -1 : 0;
    p->y += (p->dir == Nth) ? 1 : (p->dir == Sth) ? -1 : 0;
}

bool inside(const Physical& p) {
    return p.x >= 0 && p.y >= 0 && p.x < MAZE_SIZE && p.y < MAZE_SIZE;
}

// unknown_misread: 未知の壁を含む区画で壁を1枚読み違える確率（地図が誤ったまま残り，後で読み直して書き換わる）。
// shift_at: この歩で機体だけを1区画先へずらす（-1 ならずらさない。先が壁なら次の歩で試す）
// front_only: 読み違えるのは前壁だけ（壁を無しと読む）。redo: 前壁の読み落としでぶつかる前に気づき，
// redoWithFrontWall() でやり直す（search.cpp の handleMissedWall）
End simulate(const Wall& maze, uint32_t seed, float unknown_misread, int shift_at, int* shift_steps,
             bool front_only = false, bool redo = false, int* redos = nullptr) {
    std::mt19937 rng(seed);
    std::bernoulli_distribution misread(unknown_misread);
    std::uniform_int_distribution<int> pick(0, 2);
    solver_options_reset();
    solver_options.goal_x = 7;
    solver_options.goal_y = 7;
    adachi_return::solver_adachi_return_init();
    Physical phys{0, 1, Nth};   // 最初の半区画の後（mousePos と同じ）
    bool rechecked = false, shifted = false;
    uint8_t ahead = 0, behind = 0;
    *shift_steps = 0;
    for (int step = 0; step < 3000; ++step) {
        if (!shifted && shift_at >= 0 && step >= shift_at) {
            Physical next = phys;
            forward(&next);
            if (inside(next) && !get_wall_abs(&maze, phys.x, phys.y, phys.dir)) {
                phys = next;
                shifted = true;
            }
        }
        if (shifted) ++*shift_steps;
        if (!inside(phys)) return End::crashed;
        const RelDir rel[3] = {Rl90, R0, Rr90};
        bool w[3];
        for (int i = 0; i < 3; ++i) w[i] = get_wall_abs(&maze, phys.x, phys.y, relToAbsDir(phys.dir, rel[i]));
        bool ml = false, mf = false, mr = false;
        const bool known = search_lookahead::knownWalls(&ml, &mf, &mr);
        if (!known && misread(rng)) {
            int k = front_only ? 1 : pick(rng);
            if (!front_only || w[1]) w[k] = !w[k];
        }
        const uint8_t sensor = search_lookahead::wallBits(w[0], w[1], w[2]);
        const uint8_t map = search_lookahead::wallBits(ml, mf, mr);
        auto d = search_lookahead::decide(known, map, sensor, rechecked);
        if (d == search_lookahead::WallDecision::recheck) {
            rechecked = true;
            continue;
        }
        const bool use_map = (d == search_lookahead::WallDecision::useMap);
        if (rechecked && !use_map) {
            uint8_t m = search_lookahead::shiftMatch(sensor);
            ahead = (m & search_lookahead::SHIFT_AHEAD) ? ahead + 1 : 0;
            behind = (m & search_lookahead::SHIFT_BEHIND) ? behind + 1 : 0;
            if (ahead >= config::search::SHIFT_VOTES_TO_STOP || behind >= config::search::SHIFT_VOTES_TO_STOP) {
                return End::positionLost;
            }
        }
        if (!rechecked || use_map) ahead = behind = 0;
        rechecked = false;
        const uint8_t walls = use_map ? map : sensor;
        search_lookahead::prepare();
        uint8_t action = search_lookahead::take((walls & 1) != 0, (walls & 2) != 0, (walls & 4) != 0);
        if (redo && action == ACT_MOVE_1CELL && get_wall_abs(&maze, phys.x, phys.y, phys.dir)) {
            // 直進した先の境界に壁：区画の中で止まり，前壁ありでやり直す（機体はまだこの区画にいる）
            action = search_lookahead::redoWithFrontWall((walls & 1) != 0, (walls & 4) != 0);
            if (redos != nullptr) ++*redos;
        }
        // 機体を動かす（本当の迷路の壁を通れなければぶつかる）
        switch (action) {
        case ACT_MOVE_1CELL: break;
        case ACT_TURN_LEFT_MOVE: phys.dir = relToAbsDir(phys.dir, Rl90); break;
        case ACT_TURN_RIGHT_MOVE: phys.dir = relToAbsDir(phys.dir, Rr90); break;
        case ACT_TURN_BACK: phys.dir = relToAbsDir(phys.dir, R180); break;
        case ACT_FINISH: return End::finished;
        default: return End::other;
        }
        if (get_wall_abs(&maze, phys.x, phys.y, phys.dir)) return End::crashed;
        forward(&phys);
        if (mousePos.x >= MAZE_SIZE || mousePos.y >= MAZE_SIZE) return End::other;
    }
    return End::other;
}

struct Tally {
    int finished = 0, lost = 0, crashed = 0, other = 0, steps_to_lost = 0;
};

// 読み違い・ずれがなければ往復できる迷路だけを count 個使う（ゴールへの道がない迷路ではソルバーが壁へ進む）
Tally simulateMany(int count, float density, float unknown_misread, bool shift) {
    std::mt19937 rng(2026);
    Tally t;
    for (int k = 0; k < count;) {
        Wall maze = makeMaze(rng, density);
        uint32_t seed = rng();
        int unused = 0;
        if (simulate(maze, seed, 0.f, -1, &unused) != End::finished) continue;
        ++k;
        int shift_at = shift ? std::uniform_int_distribution<int>(20, 120)(rng) : -1;
        int steps = 0;
        End e = simulate(maze, seed, unknown_misread, shift_at, &steps);
        switch (e) {
        case End::finished: ++t.finished; break;
        case End::positionLost: ++t.lost; t.steps_to_lost += steps; break;
        case End::crashed: ++t.crashed; break;
        case End::other: ++t.other; break;
        }
    }
    return t;
}

void testFalseAlarms() {
    std::printf("no position loss: unknown-cell misreads rewrite the map later\n");
    for (float r : {0.f, 0.02f, 0.05f}) {
        Tally t = simulateMany(300, 0.35f, r, false);
        std::printf("  misread %.0f%%: finished %d, position lost %d, crashed %d, other %d\n", r * 100.f, t.finished,
                    t.lost, t.crashed, t.other);
        if (r == 0.f) check(t.finished == 300, "no misread: every run finishes");
        else check(t.lost <= 3, "misreads stop for position loss in at most 1% of runs");
    }
}

void testMissedFrontRedo() {
    std::printf("missed front walls (unknown cells read as open), with and without the redo\n");
    for (bool redo : {false, true}) {
        std::mt19937 rng(4242);
        int finished = 0, crashed = 0, other = 0, redos = 0;
        for (int k = 0; k < 300;) {
            Wall maze = makeMaze(rng, 0.35f);
            uint32_t seed = rng();
            int unused = 0;
            if (simulate(maze, seed, 0.f, -1, &unused) != End::finished) continue;
            ++k;
            End e = simulate(maze, seed, 0.05f, -1, &unused, true, redo, &redos);
            (e == End::finished ? finished : e == End::crashed ? crashed : other)++;
        }
        std::printf("  %s: finished %d, crashed %d, other %d (redos %d)\n", redo ? "redo" : "no redo", finished,
                    crashed, other, redos);
        if (redo) check(finished == 300, "with the redo every run finishes");
    }
}

void testShiftDetection() {
    std::printf("robot moved one cell ahead of the solver (no misreads)\n");
    Tally t = simulateMany(300, 0.35f, 0.f, true);
    std::printf("  position lost %d (%.1f steps after the shift avg), crashed %d, finished %d, other %d\n", t.lost,
                t.lost ? static_cast<double>(t.steps_to_lost) / t.lost : 0., t.crashed, t.finished, t.other);
    check(t.lost > 0, "some shifted runs stop for position loss");
}
} // namespace

int main() {
    testShiftFromLog();
    testShiftBehind();
    testRedoMissedWall();
    testFrontNear();
    testFalseAlarms();
    testShiftDetection();
    testMissedFrontRedo();
    if (g_failures > 0) {
        std::printf("test_position_loss: %d FAILED\n", g_failures);
        return 1;
    }
    std::printf("test_position_loss: all passed\n");
    return 0;
}
