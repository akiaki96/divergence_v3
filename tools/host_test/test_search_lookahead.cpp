// search_lookahead（Core/Src/app/search_lookahead.cpp）のホストでの単体試験。tools/host_test/run.sh で実行する。
//
// 乱数の迷路で往復探索（adachi_return）を2回走らせる：(a) 壁を読むたびにソルバーを直接呼ぶ（以前の探索），
// (b) 1歩ごとに prepare() し，壁を読んだら take() で取り出す（今の探索）。毎歩の動作・位置・帰り探索中かと，
// 最後の壁の地図が一致することを確かめる。ときどき壁を読み違える（反転した壁を渡す）場面も混ぜる
#include <chrono>
#include <cstdio>
#include <cstring>
#include <random>
#include "adachi.hpp"
#include "adachi_return.hpp"
#include "app/search_lookahead.hpp"
#include "config/mouse_config.hpp"

namespace {
int g_failures = 0;

void check(bool ok, const char* what) {
    std::printf("  [%s] %s\n", ok ? " ok " : "FAIL", what);
    if (!ok) ++g_failures;
}

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

struct Step {
    uint8_t action;
    MousePos pos;   // ソルバーが進めた後
    bool returning;
};

constexpr int MAX_RUN_STEPS = 2000;

struct Run {
    Step steps[MAX_RUN_STEPS];
    int count = 0;
    Wall wallzero, wallone;
    double solver_us = 0.;   // 壁を読んでから動作が決まるまでの合計
    double prepare_us = 0.;  // 先読みの合計
    bool blocked = false;    // 壁のある向きへ進もうとして止まった
};

// 読み違いを決める乱数は (a)(b) で同じ並びにするため，seed から毎回作り直す
void search(const Wall& maze, uint32_t misread_seed, float misread_rate, bool lookahead, Run* run) {
    using clock = std::chrono::steady_clock;
    std::mt19937 rng(misread_seed);
    std::bernoulli_distribution misread(misread_rate);

    solver_options_reset();
    solver_options.goal_x = 7;
    solver_options.goal_y = 7;
    adachi_return::solver_adachi_return_init();
    run->count = 0;
    while (run->count < MAX_RUN_STEPS) {
        if (lookahead) {
            auto t0 = clock::now();
            search_lookahead::prepare();
            run->prepare_us += std::chrono::duration<double, std::micro>(clock::now() - t0).count();
        }
        AbsDir dir = static_cast<AbsDir>(mousePos.dir);
        bool walls[3];
        const RelDir rel[3] = {Rl90, R0, Rr90};
        for (int i = 0; i < 3; ++i) {
            walls[i] = get_wall_abs(&maze, mousePos.x, mousePos.y, relToAbsDir(dir, rel[i]));
            if (misread(rng)) walls[i] = !walls[i];
        }
        auto t0 = clock::now();
        uint8_t action = lookahead
            ? search_lookahead::take(walls[0], walls[1], walls[2])
            : search_lookahead::firstMotion(adachi_return::solver_adachi_return(walls[0], walls[1], walls[2]));
        run->solver_us += std::chrono::duration<double, std::micro>(clock::now() - t0).count();
        run->steps[run->count++] = {action, mousePos, adachi_return::is_returning()};
        if (action == ACT_FINISH) break;
        // 実機（search.cpp の runSteps）と同じく，壁を読んだ向きへ進もうとしたら止める（ゴールへの経路が
        // ないとソルバーは直進を返す）。読み違えて外周を開いたと読んだときも，迷路の外へは出さない
        bool blocked = (action == ACT_MOVE_1CELL && walls[1]) || (action == ACT_TURN_LEFT_MOVE && walls[0]) ||
                       (action == ACT_TURN_RIGHT_MOVE && walls[2]);
        if (blocked || mousePos.x >= MAZE_SIZE || mousePos.y >= MAZE_SIZE) {
            run->blocked = true;
            break;
        }
    }
    run->wallzero = wallzero;
    run->wallone = wallone;
}

bool sameSteps(const Run& a, const Run& b) {
    if (a.count != b.count) return false;
    for (int i = 0; i < a.count; ++i) {
        const Step& s = a.steps[i];
        const Step& t = b.steps[i];
        if (s.action != t.action || s.pos.x != t.pos.x || s.pos.y != t.pos.y || s.pos.dir != t.pos.dir ||
            s.returning != t.returning) {
            return false;
        }
    }
    return true;
}

bool sameWalls(const Wall& a, const Wall& b) {
    return std::memcmp(&a, &b, sizeof(Wall)) == 0;
}

Run g_direct, g_ahead;

void testMazes(const char* name, float density, float misread_rate, int count) {
    std::mt19937 rng(12345);
    int same = 0, finished = 0, blocked = 0, total_steps = 0, max_saves = 0;
    double direct_us = 0., take_us = 0., prepare_us = 0.;
    for (int k = 0; k < count; ++k) {
        Wall maze = makeMaze(rng, density);
        uint32_t seed = rng();
        g_direct = Run{};
        g_ahead = Run{};
        search(maze, seed, misread_rate, false, &g_direct);
        search(maze, seed, misread_rate, true, &g_ahead);
        if (sameSteps(g_direct, g_ahead) && g_direct.blocked == g_ahead.blocked && sameWalls(g_direct.wallzero, g_ahead.wallzero) &&
            sameWalls(g_direct.wallone, g_ahead.wallone)) {
            ++same;
        }
        if (g_direct.count > 0 && g_direct.steps[g_direct.count - 1].action == ACT_FINISH) ++finished;
        if (g_direct.blocked) ++blocked;
        total_steps += g_direct.count;
        // 探索中の迷路の保存（app/search.cpp の updateSaveDue）：ゴールに着いた歩で1回，その後 EVERY_STEPS 歩ごと。
        // 迷路が変わらず見送る分は数えない（多めに見積もる）
        int saves = 0, since = 0;
        bool goal = false;
        for (int i = 0; i < g_direct.count; ++i) {
            if (!g_direct.steps[i].returning) continue;
            if (!goal) {
                goal = true;
                ++saves;
            } else if (++since >= config::maze_save::EVERY_STEPS) {
                since = 0;
                ++saves;
            }
        }
        if (saves + 1 > max_saves) max_saves = saves + 1;   // +1：スタートに戻ったときの記録
        direct_us += g_direct.solver_us;
        take_us += g_ahead.solver_us;
        prepare_us += g_ahead.prepare_us;
    }
    std::printf("%s: %d mazes, %d finished, %d stopped at a wall, %d steps\n", name, count, finished, blocked,
                total_steps);
    std::printf("  host time per step: direct %.2f us, take %.3f us, prepare %.2f us\n",
                direct_us / total_steps, take_us / total_steps, prepare_us / total_steps);
    std::printf("  maze saves per search: at most %d (RESERVE_SLOTS %lu)\n", max_saves,
                static_cast<unsigned long>(config::maze_save::RESERVE_SLOTS));
    check(static_cast<uint32_t>(max_saves) <= config::maze_save::RESERVE_SLOTS,
          "the maze saves of one search fit in RESERVE_SLOTS");
    char what[96];
    std::snprintf(what, sizeof(what), "lookahead matches the direct solver in all %d mazes (%d did)", count, same);
    check(same == count, what);
}

void testTakeWithoutPrepare() {
    std::printf("take() without prepare() calls the solver\n");
    solver_options_reset();
    adachi_return::solver_adachi_return_init();
    search_lookahead::prepare();
    search_lookahead::take(false, false, true);   // 使い切る
    check(!search_lookahead::ready(), "not ready after take()");
    adachi::SearchState before;
    adachi::search_save(&before);
    uint8_t direct = search_lookahead::firstMotion(adachi_return::solver_adachi_return(true, false, true));
    MousePos direct_pos = mousePos;
    adachi::search_restore(before);
    uint8_t taken = search_lookahead::take(true, false, true);
    check(taken == direct && mousePos.x == direct_pos.x && mousePos.y == direct_pos.y &&
              mousePos.dir == direct_pos.dir,
          "same action and position as the solver");
}

void testPrepareKeepsState() {
    std::printf("prepare() leaves the solver state as it was\n");
    solver_options_reset();
    adachi_return::solver_adachi_return_init();
    adachi_return::solver_adachi_return(true, false, true);
    adachi::SearchState before, after;
    adachi::search_save(&before);
    search_lookahead::prepare();
    adachi::search_save(&after);
    check(std::memcmp(&before.pos, &after.pos, sizeof(MousePos)) == 0 && sameWalls(before.wallzero, after.wallzero) &&
              sameWalls(before.wallone, after.wallone) && before.returning == after.returning,
          "position, walls and returning unchanged");
    check(search_lookahead::ready(), "ready after prepare()");
}
} // namespace

int main() {
    testPrepareKeepsState();
    testTakeWithoutPrepare();
    testMazes("random mazes", 0.35f, 0.f, 100);
    testMazes("random mazes, misread 3%", 0.35f, 0.03f, 100);
    testMazes("sparse mazes", 0.15f, 0.f, 50);
    if (g_failures > 0) {
        std::printf("test_search_lookahead: %d FAILED\n", g_failures);
        return 1;
    }
    std::printf("test_search_lookahead: all passed\n");
    return 0;
}
