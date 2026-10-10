// 最短走行の経路で確かめる探索（adachi::SearchKind::to_goal_fast_confirm_back，探索プリセットの "confirm"）の
// ホストでの単体試験。tools/host_test/run.sh で実行する。
//
// 乱数の迷路で2通りに走らせる：(a) シミュレータと同じく，壁を読むたびに adachi_fast_confirm のソルバーを直接呼ぶ
// （確認の歩は search_step と search_check を続けて呼ぶ），(b) 実機の探索（search.cpp の runSteps）と同じく，
// 1歩ごとに search_lookahead::prepare()（既知の直進も数える）し，壁を読んだら take() で取り出し，確認の歩
// （adachi::search_check_pending()）なら止まったつもりで adachi::search_check() を呼ぶ。
// 毎歩の動作・位置が一致すること，加速区間の途中で確認が来ないこと，スタートに戻って終わったときに
// 最後の確認で確かめる壁が0（未知の壁を通れるとみなした最短走行の経路に未知の壁がない）であることを確かめる。
// 確認1回の経路計算の時間（ホスト）も出す
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <random>
#include "adachi.hpp"
#include "adachi_fast_confirm.hpp"
#include "app/search_lookahead.hpp"
#include "time_based_dijkstra.hpp"

namespace {
int g_failures = 0;

void check(bool ok, const char* what) {
    std::printf("  [%s] %s\n", ok ? " ok " : "FAIL", what);
    if (!ok) ++g_failures;
}

// 本当の迷路（外周とスタート区画の東は壁，ほかは乱数。test_search_lookahead.cpp と同じ）
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
    MousePos pos;
};

constexpr int MAX_RUN_STEPS = 3000;

struct Run {
    Step steps[MAX_RUN_STEPS];
    int count = 0;
    bool finished = false;
    int checks = 0;
    int check_in_run = 0;      // 加速区間の途中で確認が来た（あってはならない）
    int plan_mismatch = 0;     // 加速区間の途中で直進以外が返った（あってはならない）
    double check_us_max = 0.;
    double check_us_total = 0.;
    adachi::CheckResult last{};
};

void readWalls(const Wall& maze, bool walls[3]) {
    AbsDir dir = static_cast<AbsDir>(mousePos.dir);
    const RelDir rel[3] = {Rl90, R0, Rr90};
    for (int i = 0; i < 3; ++i) walls[i] = get_wall_abs(&maze, mousePos.x, mousePos.y, relToAbsDir(dir, rel[i]));
}

bool blockedMove(uint8_t action, const bool walls[3]) {
    return (action == ACT_MOVE_1CELL && walls[1]) || (action == ACT_TURN_LEFT_MOVE && walls[0]) ||
           (action == ACT_TURN_RIGHT_MOVE && walls[2]);
}

void initSolver() {
    solver_options_reset();
    solver_options.goal_x = 7;
    solver_options.goal_y = 7;
    adachi_fast_confirm::solver_adachi_fast_confirm_init();
}

// (a) シミュレータと同じ呼び方
void searchDirect(const Wall& maze, Run* run) {
    initSolver();
    while (run->count < MAX_RUN_STEPS) {
        bool w[3];
        readWalls(maze, w);
        uint8_t action = search_lookahead::firstMotion(adachi_fast_confirm::solver_adachi_fast_confirm(w[0], w[1], w[2]));
        run->steps[run->count++] = {action, mousePos};
        if (action == ACT_FINISH) {
            run->finished = true;
            break;
        }
        if (blockedMove(action, w)) break;
    }
    run->checks = adachi_fast_confirm::check_count();
    run->last = adachi::search_last_check();
}

// (b) 実機と同じ呼び方（先読み，既知の直進の加速区間，確認の歩で止まって search_check）
void searchFirmware(const Wall& maze, Run* run) {
    using clock = std::chrono::steady_clock;
    initSolver();
    int committed = 0;
    search_lookahead::PrepareOptions chains;
    chains.chains = true;
    search_lookahead::prepare(chains);
    while (run->count < MAX_RUN_STEPS) {
        bool w[3];
        readWalls(maze, w);
        uint8_t action = search_lookahead::take(w[0], w[1], w[2]);
        if (adachi::search_check_pending()) {
            if (committed > 0) ++run->check_in_run;
            auto t0 = clock::now();
            action = search_lookahead::firstMotion(adachi::search_check());
            double us = std::chrono::duration<double, std::micro>(clock::now() - t0).count();
            run->check_us_total += us;
            run->check_us_max = std::max(run->check_us_max, us);
            ++run->checks;
            committed = 0;
        } else if (committed > 0) {
            if (action != ACT_MOVE_1CELL) ++run->plan_mismatch;
            --committed;
        } else if (action == ACT_MOVE_1CELL && search_lookahead::straightCells() >= 2) {
            committed = search_lookahead::straightCells() - 1;
        }
        run->steps[run->count++] = {action, mousePos};
        if (action == ACT_FINISH) {
            run->finished = true;
            break;
        }
        if (blockedMove(action, w)) break;
        search_lookahead::PrepareOptions o;
        o.chains = (committed == 0);
        search_lookahead::prepare(o);
    }
    run->last = adachi::search_last_check();
}

bool sameSteps(const Run& a, const Run& b) {
    if (a.count != b.count) return false;
    for (int i = 0; i < a.count; ++i) {
        const Step& s = a.steps[i];
        const Step& t = b.steps[i];
        if (s.action != t.action || s.pos.x != t.pos.x || s.pos.y != t.pos.y || s.pos.dir != t.pos.dir) return false;
    }
    return true;
}

Run g_direct, g_firmware;

void testMazes(const char* name, float density, int count) {
    std::mt19937 rng(2026);
    int same = 0, finished = 0, fixed = 0, bounds_equal = 0, total_steps = 0, total_checks = 0, max_checks = 0;
    int check_in_run = 0, plan_mismatch = 0, overflowed = 0;
    double check_us_max = 0., check_us_total = 0.;
    for (int k = 0; k < count; ++k) {
        Wall maze = makeMaze(rng, density);
        g_direct = Run{};
        g_firmware = Run{};
        searchDirect(maze, &g_direct);
        searchFirmware(maze, &g_firmware);
        if (sameSteps(g_direct, g_firmware) && g_direct.checks == g_firmware.checks) ++same;
        check_in_run += g_firmware.check_in_run;
        plan_mismatch += g_firmware.plan_mismatch;
        if (g_firmware.last.overflowed) ++overflowed;
        if (g_firmware.finished) {
            ++finished;
            if (g_firmware.last.targets == 0) ++fixed;
            // 最後の地図で，未知の壁を通れるとみなした最短時間（下界）と，壁とみなした最短時間（上界）
            uint16_t t_zero = time_based_dijkstra::shortest_time(&wallzero, nullptr);
            uint16_t t_one = time_based_dijkstra::shortest_time(&wallone, nullptr);
            if (t_zero == t_one) ++bounds_equal;
        }
        total_steps += g_firmware.count;
        total_checks += g_firmware.checks;
        max_checks = std::max(max_checks, g_firmware.checks);
        check_us_total += g_firmware.check_us_total;
        check_us_max = std::max(check_us_max, g_firmware.check_us_max);
    }
    std::printf("%s: %d mazes, %d finished, %d steps (%.1f per maze), checks %.1f per maze (max %d)\n", name, count,
                finished, total_steps, static_cast<double>(total_steps) / count,
                static_cast<double>(total_checks) / count, max_checks);
    std::printf("  host time per check: mean %.0f us, max %.0f us; lower bound = upper bound in %d of %d\n",
                total_checks > 0 ? check_us_total / total_checks : 0., check_us_max, bounds_equal, finished);
    char what[112];
    std::snprintf(what, sizeof(what), "the firmware flow matches the simulator solver in all %d mazes (%d did)", count,
                  same);
    check(same == count, what);
    check(check_in_run == 0, "no check arrives inside an accelerated known straight");
    check(plan_mismatch == 0, "accelerated known straights get only straight moves");
    check(overflowed == 0, "the optimistic path search never overflows its queue");
    std::snprintf(what, sizeof(what), "every finished search ends with no unknown wall on the path (%d of %d)", fixed,
                  finished);
    check(fixed == finished, what);
}
} // namespace

int main() {
    std::printf("search confirm (fast-run path):\n");
    testMazes("sparse walls (0.30)", 0.30f, 60);
    testMazes("dense walls (0.45)", 0.45f, 60);
    std::printf("%s\n", g_failures == 0 ? "all passed" : "FAILED");
    return g_failures == 0 ? 0 : 1;
}
