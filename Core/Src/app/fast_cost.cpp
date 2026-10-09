// 最短走行のソルバーのコスト（fast_plan::applySolverCosts）。HAL に依存しない（ホストの確認でも同じものを使う）
#include "app/fast_plan.hpp"
#include "config/mouse_config.hpp"
#include "solver_options.h"

namespace fast_plan {

namespace {
// ターンの経路長（左右の平均）
float turnDist(const slalom::Param& t) {
    return 0.5f * (slalom::totalDistance(t, slalom::TurnDir::left) + slalom::totalDistance(t, slalom::TurnDir::right));
}
} // namespace

bool applySolverCosts(const RunPreset& p) {
    solver_options.diagonal = p.diagonal;
    RunProfile prof;
    prof.cell_mm = config::maze::CELL_MM;
    prof.turn_speed = p.turn_speed;
    prof.max_speed = p.max_speed;
    prof.max_speed_dia = p.max_speed_dia;
    prof.accel = p.accel;
    prof.decel = p.decel;
    for (uint8_t k = 0; k < TURN_KIND_COUNT; ++k) {
        if (const slalom::Param* t = p.turns[k].top()) prof.turn_dist[k] = turnDist(*t);
    }
    if (!solver_options_apply_profile(prof)) return false;

    // コスト表のターンは turn_speed で見積もるので，種類ごとに自分の速度で見積もり直す
    // （ターンの前後の直線は turn_speed で入って出る近似のまま）
    for (uint8_t k = 0; k < TURN_KIND_COUNT; ++k) {
        const slalom::Param* t = p.turns[k].top();
        if (t == nullptr) {
            solver_options.turn_ms[k] = DISABLED_TURN_MS;   // ほかの経路があれば必ずそちらを選ぶ
            continue;
        }
        float ms = 1000.f * prof.turn_dist[k] / t->speed;
        solver_options.turn_ms[k] = static_cast<uint16_t>((ms < 60000.f) ? ms + 0.5f : 60000.f);
    }
    return true;
}

} // namespace fast_plan
