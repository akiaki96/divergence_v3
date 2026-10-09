// 最短走行のゴール領域（solver_options.goal_size、time_based_dijkstra）のホストでの単体試験。tools/host_test/run.sh で実行する。
//
// ゴールを (7,7)〜(8,8) の 2×2 にすると、領域の各区画の中央に着く最短の時間のうち最も長い区画で経路が終わる。
// 領域の入口を1つにした迷路で、終わる区画が奥の区画になること、領域の中の壁で入れない区画は選ばれないこと、
// 入れるのが入口の区画だけなら goal_size=1 と同じ経路になることを確かめる。乱数の迷路では、1区画のゴールに
// 経路があれば領域にも経路があり、終わる区画が領域の中にあることを確かめる
#include <cstdio>
#include <random>
#include "time_based_dijkstra.hpp"
#include "solver_options.h"

namespace {
int g_failures = 0;

void check(bool ok, const char* what) {
    std::printf("  [%s] %s\n", ok ? " ok " : "FAIL", what);
    if (!ok) ++g_failures;
}

// 外周とスタート区画の東だけが壁の迷路
Wall openMaze() {
    Wall w;
    wall_reset_zero(&w);
    for (uint8_t i = 0; i < MAZE_SIZE; ++i) {
        set_wall(&w, i, 0, Sth, true);
        set_wall(&w, i, MAZE_SIZE - 1, Nth, true);
        set_wall(&w, 0, i, Wst, true);
        set_wall(&w, MAZE_SIZE - 1, i, Est, true);
    }
    set_wall(&w, 0, 0, Est, true);
    return w;
}

// ゴール領域 (7,7)〜(8,8) を壁で囲み、(7,7) の南だけを開ける
void encloseGoal(Wall* w) {
    for (uint8_t i = 7; i <= 8; ++i) {
        set_wall(w, i, 7, Sth, true);
        set_wall(w, i, 8, Nth, true);
        set_wall(w, 7, i, Wst, true);
        set_wall(w, 8, i, Est, true);
    }
    set_wall(w, 7, 7, Sth, false);
}

struct Result {
    bool found;
    uint16_t time_ms;
    uint8_t end_x, end_y;
    uint8_vector actions;
};

Result solve(const Wall& w, uint8_t size, bool diagonal) {
    solver_options_reset();
    solver_options.goal_x = 7;
    solver_options.goal_y = 7;
    solver_options.goal_size = size;
    solver_options.diagonal = diagonal;
    wallone = w;
    Result r;
    r.actions = time_based_dijkstra::solver_time_based_dijekstra_init();
    r.found = time_based_dijkstra::last_path_found();
    r.time_ms = time_based_dijkstra::last_path_time_ms();
    time_based_dijkstra::last_goal_cell(&r.end_x, &r.end_y);
    return r;
}

bool sameActions(const Result& a, const Result& b) {
    if (a.actions.size() != b.actions.size()) return false;
    for (std::size_t i = 0; i < a.actions.size(); ++i) {
        if (a.actions[i] != b.actions[i]) return false;
    }
    return true;
}

void print(const char* name, const Result& r) {
    std::printf("  %s: found %d, %u ms, end (%u,%u), actions", name, r.found, r.time_ms, r.end_x, r.end_y);
    for (uint8_t a : r.actions) std::printf(" %u", a);
    std::printf("\n");
}
}  // namespace

int main() {
    std::printf("goal region: entrance only on the south of (7,7)\n");
    for (bool dia : {false, true}) {
        Wall w = openMaze();
        encloseGoal(&w);
        Result one = solve(w, 1, dia);
        Result two = solve(w, 2, dia);
        print("size 1", one);
        print("size 2", two);
        check(one.found && two.found, "both find a path");
        check(one.end_x == 7 && one.end_y == 7, "size 1 ends in (7,7)");
        check(!(two.end_x == 7 && two.end_y == 7), "size 2 ends past the entrance cell");
        check(two.time_ms > one.time_ms, "the end cell is reached later than the entrance cell");
        // 縦横だけなら、(7,7) の中央から直進で (7,8)、大回り90°で (8,8)。ターンのほうが遅いので (8,8)。
        // 斜めありでは、斜めから出45°などで (8,8) に入るほうが (7,8) まで直進するより早いことがある
        if (!dia) check(two.end_x == 8 && two.end_y == 8, "without diagonals it ends in the far cell (8,8)");
    }

    std::printf("goal region: (8,8) walled off inside (no diagonals)\n");
    {
        Wall w = openMaze();
        encloseGoal(&w);
        set_wall(&w, 8, 8, Sth, true);
        set_wall(&w, 8, 8, Wst, true);
        Result two = solve(w, 2, false);
        print("size 2", two);
        check(two.found, "finds a path");
        check(two.end_x == 8 && two.end_y == 7, "ends in (8,7) (a turn after the entrance, later than straight to (7,8))");
    }

    std::printf("goal region: only the entrance cell is reachable\n");
    for (bool dia : {false, true}) {
        Wall w = openMaze();
        encloseGoal(&w);
        set_wall(&w, 7, 7, Nth, true);
        set_wall(&w, 7, 7, Est, true);
        Result one = solve(w, 1, dia);
        Result two = solve(w, 2, dia);
        print("size 1", one);
        print("size 2", two);
        check(two.found && two.end_x == 7 && two.end_y == 7, "ends in (7,7)");
        // size 1 は辺のノードの時間をそのまま比べる（区画中央までの残りの半区画を足さない）ので、
        // 斜めありでは別の着き方を選ぶことがある。縦横だけなら同じ経路
        if (!dia) check(sameActions(one, two), "same path as size 1 (no diagonals)");
    }

    std::printf("goal region: start-adjacent region (0,0)-(1,1)\n");
    {
        Wall w = openMaze();
        solver_options_reset();
        solver_options.goal_x = 0;
        solver_options.goal_y = 0;
        solver_options.goal_size = 2;
        wallone = w;
        time_based_dijkstra::solver_time_based_dijekstra_init();
        uint8_t x = 0, y = 0;
        time_based_dijkstra::last_goal_cell(&x, &y);
        std::printf("  end (%u,%u)\n", x, y);
        check(time_based_dijkstra::last_path_found(), "finds a path");
        check(x <= 1 && y <= 1 && !(x == 0 && y == 0), "ends in the region, not in the start cell");
    }

    std::printf("goal region: random mazes\n");
    {
        std::mt19937 rng(12345);
        std::bernoulli_distribution has_wall(0.35);
        int both = 0, region_only = 0, lost = 0, outside = 0;
        for (int n = 0; n < 300; ++n) {
            Wall w = openMaze();
            for (uint8_t y = 0; y < MAZE_SIZE; ++y) {
                for (uint8_t x = 0; x < MAZE_SIZE; ++x) {
                    if (x + 1 < MAZE_SIZE && has_wall(rng)) set_wall(&w, x, y, Est, true);
                    if (y + 1 < MAZE_SIZE && has_wall(rng)) set_wall(&w, x, y, Nth, true);
                }
            }
            set_wall(&w, 0, 0, Nth, false);
            Result one = solve(w, 1, true);
            Result two = solve(w, 2, true);
            if (one.found && !two.found) ++lost;
            if (two.found && (two.end_x < 7 || two.end_x > 8 || two.end_y < 7 || two.end_y > 8)) ++outside;
            if (one.found && two.found) ++both;
            if (!one.found && two.found) ++region_only;
        }
        std::printf("  %d with both, %d only via the region\n", both, region_only);
        check(lost == 0, "a path to (7,7) always means a path to the region");
        check(outside == 0, "the path always ends inside the region");
    }

    if (g_failures != 0) {
        std::printf("goal region: %d FAILED\n", g_failures);
        return 1;
    }
    std::printf("goal region: all passed\n");
    return 0;
}
