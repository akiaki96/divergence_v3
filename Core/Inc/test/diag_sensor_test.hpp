#pragma once

#include <cstdint>
#include "common/slalom.hpp"
#include "config/slalom_params.hpp"

// 斜め走行のセンサーのデータ収集（naophis「斜めの姿勢制御をするには」の「理想のリファレンス値をとる」）。
// 手順・壁の並べ方・解析は tools/DIAGONAL.md。
//
// 置き方は探索と同じ：機体の後端を区画の後壁に当てて北へ向ける。1区画先の区画中央まで加速し，入45°（in45）で
// dir へ曲がって斜めの直線に入る。入45°の出口の基準点（区画の辺の中点）から，辺の中点の間隔（90√2 mm）を
// half_steps 区間ぶん走り，最後の区間の後半で減速して half_steps 個先の辺の中点で止まる。
//
// ログ（diag/<IN45の名前>_<left|right>_n<half_steps>）は斜めの直線の手前で取り直す（バッファを斜めの区間に使う）。
// 列：diag_x（出口の基準点からの実測の距離），target_diag_x，angle_error（実測 − 目標の角度），
//     ir_l / ir_fl / ir_fr / ir_r，since_l / since_r（DiagEdge：最後の切れ目からの距離，まだなければ NaN）
void runDiagSensorTest(const slalom::Param& in45, slalom::TurnDir dir, uint32_t half_steps);

// メニューから呼ぶ（入45°は 500 mm/s の設計）
template <slalom::TurnDir Dir, uint32_t N>
void diag_sensor_test_onenter() {
    runDiagSensorTest(config::slalom::IN45_500, Dir, N);
}
