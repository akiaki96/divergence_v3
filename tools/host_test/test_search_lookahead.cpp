// search_lookahead（Core/Src/app/search_lookahead.cpp）のホストでの単体試験。tools/host_test/run.sh で実行する。
//
// 乱数の迷路で往復探索（adachi_return）を2回走らせる：(a) 壁を読むたびにソルバーを直接呼ぶ（以前の探索），
// (b) 1歩ごとに prepare() し，壁を読んだら take() で取り出す（今の探索）。毎歩の動作・位置・帰り探索中かと，
// 最後の壁の地図が一致することを確かめる。ときどき壁を読み違える（反転した壁を渡す）場面も混ぜる。
//
// 後半は実機の探索（search.cpp の runSteps）と同じ壁の使い方で走らせる：3辺とも既知の区画は地図の壁を使い，
// センサーと食い違えば読み直す（decide()），既知の直進を数えて（chains）加速区間にする。
// 加速区間の途中でソルバーが直進以外を返さないこと，読み直しで正しい壁に戻ること，迷路が地図と変わっていても
// 地図が書き換わって探索が終わることを確かめる
#include <chrono>
#include <cstdio>
#include <cstring>
#include <random>
#include "adachi.hpp"
#include "adachi_return.hpp"
#include "app/search_lookahead.hpp"

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
    int same = 0, finished = 0, blocked = 0, total_steps = 0;
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
        direct_us += g_direct.solver_us;
        take_us += g_ahead.solver_us;
        prepare_us += g_ahead.prepare_us;
    }
    std::printf("%s: %d mazes, %d finished, %d stopped at a wall, %d steps\n", name, count, finished, blocked,
                total_steps);
    std::printf("  host time per step: direct %.2f us, take %.3f us, prepare %.2f us\n",
                direct_us / total_steps, take_us / total_steps, prepare_us / total_steps);
    char what[96];
    std::snprintf(what, sizeof(what), "lookahead matches the direct solver in all %d mazes (%d did)", count, same);
    check(same == count, what);
}

// ---- 実機の探索と同じ壁の使い方 ----
struct FirmwareRun {
    Step steps[MAX_RUN_STEPS];
    int count = 0;
    int runs = 0;            // 加速して積んだ区間の数
    int run_cells = 0;       // その区画数の合計
    int max_run = 0;
    int rechecks = 0;        // 食い違って読み直した回数
    int map_updates = 0;     // 読み直しても食い違い，センサーの壁で地図を書き換えた回数
    int plan_mismatch = 0;   // 加速区間の途中でソルバーが直進以外を返した（あってはならない）
    int unknown_in_run = 0;  // 加速区間の途中の区画が既知でなかった（あってはならない）
    bool blocked = false;
    double prepare_us = 0.;
    Wall wallzero, wallone;
};

// first_misread_rate: 3辺とも既知の区画で，1回目の読みだけ壁を1枚読み違える確率（読み直すと正しく読める）。
// open_after_goal: ゴールに着いたら，迷路の壁をこの数だけ取り除く（地図より迷路が新しい：読み直しても食い違う）
void searchFirmware(Wall maze, uint32_t seed, float first_misread_rate, int open_after_goal, FirmwareRun* run) {
    using clock = std::chrono::steady_clock;
    std::mt19937 rng(seed);
    std::bernoulli_distribution misread(first_misread_rate);
    std::uniform_int_distribution<int> pick_wall(0, 2);

    solver_options_reset();
    solver_options.goal_x = 7;
    solver_options.goal_y = 7;
    adachi_return::solver_adachi_return_init();
    run->count = 0;
    int committed = 0;
    bool rechecked = false;
    bool opened = false;
    search_lookahead::PrepareOptions o;
    o.chains = true;
    auto t0 = clock::now();
    search_lookahead::prepare(o);
    run->prepare_us += std::chrono::duration<double, std::micro>(clock::now() - t0).count();
    while (run->count < MAX_RUN_STEPS) {
        AbsDir dir = static_cast<AbsDir>(mousePos.dir);
        const RelDir rel[3] = {Rl90, R0, Rr90};
        bool w[3];
        for (int i = 0; i < 3; ++i) w[i] = get_wall_abs(&maze, mousePos.x, mousePos.y, relToAbsDir(dir, rel[i]));
        bool ml = false, mf = false, mr = false;
        bool known = search_lookahead::knownWalls(&ml, &mf, &mr);
        if (known && !rechecked && misread(rng)) {
            int k = pick_wall(rng);
            w[k] = !w[k];
        }
        uint8_t sensor = search_lookahead::wallBits(w[0], w[1], w[2]);
        uint8_t map = search_lookahead::wallBits(ml, mf, mr);
        search_lookahead::WallDecision d = search_lookahead::decide(known, map, sensor, rechecked);
        if (d == search_lookahead::WallDecision::recheck) {
            // 実機は止まって下がり，止まっている間に8通りで先読みを作り直してから読み直す
            ++run->rechecks;
            rechecked = true;
            committed = 0;
            search_lookahead::PrepareOptions all = o;
            all.all = true;
            search_lookahead::prepare(all);
            continue;
        }
        bool use_map = (d == search_lookahead::WallDecision::useMap);
        uint8_t walls = use_map ? map : sensor;
        if (rechecked && !use_map) ++run->map_updates;
        rechecked = false;
        if (committed > 0 && !known) ++run->unknown_in_run;

        uint8_t action = search_lookahead::take((walls & 1) != 0, (walls & 2) != 0, (walls & 4) != 0);
        run->steps[run->count++] = {action, mousePos, adachi_return::is_returning()};
        if (action == ACT_FINISH) break;
        bool blocked = (action == ACT_MOVE_1CELL && (walls & 2)) || (action == ACT_TURN_LEFT_MOVE && (walls & 1)) ||
                       (action == ACT_TURN_RIGHT_MOVE && (walls & 4));
        if (blocked || mousePos.x >= MAZE_SIZE || mousePos.y >= MAZE_SIZE) {
            run->blocked = true;
            break;
        }

        if (committed > 0) {
            if (action != ACT_MOVE_1CELL) {
                ++run->plan_mismatch;
                committed = 0;
            } else {
                --committed;
            }
        } else if (action == ACT_MOVE_1CELL) {
            int n = search_lookahead::straightCells();
            if (n >= 2) {
                committed = n - 1;
                ++run->runs;
                run->run_cells += n;
                if (n > run->max_run) run->max_run = n;
            }
        }

        if (!opened && open_after_goal > 0 && adachi_return::is_returning()) {
            // 迷路の内側の壁を取り除く（外周とスタート区画は残す）。通路が増えるだけなので，ゴール・スタートへの道は残る
            opened = true;
            std::uniform_int_distribution<int> cell(1, MAZE_SIZE - 2);
            for (int k = 0; k < open_after_goal; ++k) {
                uint8_t x = static_cast<uint8_t>(cell(rng)), y = static_cast<uint8_t>(cell(rng));
                set_wall(&maze, x, y, (k & 1) ? Est : Nth, false);
            }
        }

        o.chains = (committed == 0);
        t0 = clock::now();
        search_lookahead::prepare(o);
        run->prepare_us += std::chrono::duration<double, std::micro>(clock::now() - t0).count();
    }
    run->wallzero = wallzero;
    run->wallone = wallone;
}

FirmwareRun g_fw;

bool sameStepsFw(const Run& a, const FirmwareRun& b) {
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

// 迷路が変わらず読み違いもなければ，地図の壁を使っても読み直しても，以前の探索と同じ動作列・地図になる。
// 加速区間の途中の動作はすべて直進で，区画はすべて既知
void testFirmwareSearch(const char* name, float density, float first_misread_rate, int count) {
    std::mt19937 rng(777);
    int same = 0, plan_mismatch = 0, unknown_in_run = 0, runs = 0, run_cells = 0, max_run = 0, rechecks = 0,
        updates = 0, steps = 0;
    double prepare_us = 0.;
    for (int k = 0; k < count; ++k) {
        Wall maze = makeMaze(rng, density);
        uint32_t seed = rng();
        g_direct = Run{};
        g_fw = FirmwareRun{};
        search(maze, seed, 0.f, false, &g_direct);
        searchFirmware(maze, seed, first_misread_rate, 0, &g_fw);
        if (sameStepsFw(g_direct, g_fw) && g_direct.blocked == g_fw.blocked &&
            sameWalls(g_direct.wallzero, g_fw.wallzero) && sameWalls(g_direct.wallone, g_fw.wallone)) {
            ++same;
        }
        plan_mismatch += g_fw.plan_mismatch;
        unknown_in_run += g_fw.unknown_in_run;
        runs += g_fw.runs;
        run_cells += g_fw.run_cells;
        if (g_fw.max_run > max_run) max_run = g_fw.max_run;
        rechecks += g_fw.rechecks;
        updates += g_fw.map_updates;
        steps += g_fw.count;
        prepare_us += g_fw.prepare_us;
    }
    std::printf("%s: %d mazes, %d steps, %d accelerated runs (%.1f cells avg, max %d, %.0f%% of steps), "
                "%d rechecks, host prepare %.2f us/step\n",
                name, count, steps, runs, runs ? static_cast<double>(run_cells) / runs : 0., max_run,
                steps ? 100. * run_cells / steps : 0., rechecks, prepare_us / steps);
    char what[128];
    std::snprintf(what, sizeof(what), "same steps and walls as the plain search in all %d mazes (%d did)", count, same);
    check(same == count, what);
    check(plan_mismatch == 0, "every read inside an accelerated run gets a straight");
    check(unknown_in_run == 0, "every cell inside an accelerated run has known walls");
    check(updates == 0, "a recheck reads the true walls, so the map is never rewritten");
    if (first_misread_rate > 0.f) check(rechecks > 0, "first-read misreads on known cells cause rechecks");
}

// 帰り探索で迷路の壁を取り除く（地図には壁が残っている）：読み直しても食い違うので地図を書き換え，探索は終わる。
// 迷路を変えなくても往復できる迷路だけを使う
void testChangedMaze(int count) {
    std::mt19937 rng(4242);
    int finished = 0, updates = 0, plan_mismatch = 0, unknown_in_run = 0;
    int k = 0;
    while (k < count) {
        Wall maze = makeMaze(rng, 0.35f);
        uint32_t seed = rng();
        // 変えなくてもゴールへの道がない迷路（乱数の迷路の一部）は使わない
        g_direct = Run{};
        search(maze, seed, 0.f, false, &g_direct);
        if (g_direct.count == 0 || g_direct.steps[g_direct.count - 1].action != ACT_FINISH) continue;
        ++k;
        g_fw = FirmwareRun{};
        searchFirmware(maze, seed, 0.f, 30, &g_fw);
        if (g_fw.count > 0 && g_fw.steps[g_fw.count - 1].action == ACT_FINISH) ++finished;
        updates += g_fw.map_updates;
        plan_mismatch += g_fw.plan_mismatch;
        unknown_in_run += g_fw.unknown_in_run;
    }
    std::printf("changed mazes: %d mazes, %d finished, %d map rewrites\n", count, finished, updates);
    char what[96];
    std::snprintf(what, sizeof(what), "every search returns to the start (%d of %d)", finished, count);
    check(finished == count, what);
    check(updates > 0, "walls that differ from the map are rewritten after a recheck");
    check(plan_mismatch == 0 && unknown_in_run == 0, "accelerated runs stay straight over known cells");
}

void testDecide() {
    std::printf("decide()\n");
    using search_lookahead::WallDecision;
    using search_lookahead::decide;
    check(decide(false, 0, 5, false) == WallDecision::useSensor, "unknown walls: sensor");
    check(decide(true, 5, 5, false) == WallDecision::useMap, "known and equal: map");
    check(decide(true, 5, 4, false) == WallDecision::recheck, "known and different: recheck");
    check(decide(true, 5, 4, true) == WallDecision::useSensor, "different after a recheck: sensor");
    check(decide(true, 5, 5, true) == WallDecision::useMap, "equal after a recheck: map");
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
    testDecide();
    testFirmwareSearch("firmware search", 0.35f, 0.f, 100);
    testFirmwareSearch("firmware search, sparse", 0.15f, 0.f, 50);
    testFirmwareSearch("firmware search, first-read misread 5%", 0.35f, 0.05f, 100);
    testChangedMaze(50);
    if (g_failures > 0) {
        std::printf("test_search_lookahead: %d FAILED\n", g_failures);
        return 1;
    }
    std::printf("test_search_lookahead: all passed\n");
    return 0;
}
