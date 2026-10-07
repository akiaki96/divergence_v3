#pragma once

#include <cstdint>
#include <numbers>

namespace config::mouse {
inline constexpr float ENCODER_RES = 4096.f;
inline constexpr float GEAR_RATIO = 13.f / 42.f;
inline constexpr float WHEEL_RADIUS_MM = 23.6f / 2;
inline constexpr float TREAD_MM = 60.f;
// 機体の後端から車軸（左右の車輪の中心を結ぶ線＝回転中心，スラロームのシミュレータの基準点）までの距離。
// 後端を壁に当てて置く試験の開始位置に使う。確かめ方は test/axle_check_test.hpp
inline constexpr float BACK_TO_AXLE_MM = 42.f;   // [mm] 2026-10-02に25から変更

// 車輪の速度差の半分 w = (v_R − v_L)/2 [mm/s] と車体の角速度 ω [dps] の換算（滑りなし）：w = ω·π/180·TREAD/2
inline constexpr float WHEEL_DIFF_PER_DPS = std::numbers::pi_v<float> / 180.f * TREAD_MM / 2.f;   // [mm/s/dps] ≈ 0.524
} // namespace config::mouse

namespace config::control {
inline constexpr float DT_S = 0.001f;
}

namespace config::imu {
inline constexpr uint16_t REFFERENCE_NUM = 1000; // 何回の平均をもってジャイロのリファレンス電圧とするか

// ジャイロ関連マクロ
inline constexpr float GYRO_Z_SIGN = 1.f;
inline constexpr float GYRO_SENSITIVITY = 16.4f;

// 加速度計関連マクロ
inline constexpr float ACCEL_X_SIGN = 1.f;
inline constexpr float ACCEL_SENSITIVITY = 4096.f;

inline constexpr float G = 9.80665f * 1000.f; // mm/s^2
} // namespace config::imu

namespace config::battery {
inline constexpr float IIR_ALPHA = 0.1f;
}

// 吸引ファンを回して走るときのduty（スラロームの試験など。plan_profile_testの高速試験と同じ20%）
namespace config::fan {
inline constexpr float RUN_DUTY = 0.20f;
inline constexpr uint32_t SPINUP_MS = 1000;   // ファンのスピンアップ待ち（吸着力が立ち上がるまで）
}

namespace config::motor {
inline constexpr float MAX_DUTY = 0.95;
inline constexpr uint16_t MAX_PWM = 1000;
inline constexpr float kVbattMinSafe = 5.0f;
}


// 並進速度 PI + feedforward 制御（translational_gain_tuning.md）
//
// プラントモデル: G_V(s) = K_p / (T_p1*s + 1)　（PIゲイン設計用の局所モデル）
// 不感帯込みアフィンモデル: v = A_GAIN * (u - U0_DEADZONE * sign(v))　（feedforward用）
//
// K_p / T_p1 は data_analysis2/prbs_identification_report.md のPRBS本同定結果
// （t01〜t08統合, holdoutフィット94.2%）を採用。
// A_GAIN / U0_DEADZONE は data_analysis2/step_identification_report.md
// （duty10/15/20%のアフィンモデル）に基づく値のまま（PRBSでは不感帯を跨ぐ多点同定は未実施）。
namespace config::pid_velocity_x {
// --- プラントモデル ---
inline constexpr float K_p  = 1518.9f;  // [mm/s/V] PRBS本同定(procest P1)
inline constexpr float T_p1 = 0.4451f;  // [s]      同上
inline constexpr float A_GAIN        = 1738.0f;  // [mm/s/V] アフィンモデルの真の速度ゲイン a（step応答由来）
inline constexpr float U0_DEADZONE   = 0.1360f;  // [V]      不感帯電圧 u0（step応答由来）

// --- PIフィードバックゲイン（IMC/λ整定, §3.1）---
// [要調整] λは未実験。実機でオーバーシュート/整定時間を見ながら調整する（目安 T_p1/2〜2*T_p1）
inline constexpr float LAMBDA = 0.03f;  // [s] 閉ループ時定数
inline constexpr float Kc = T_p1 / (K_p * LAMBDA);
inline constexpr float kp = Kc;
inline constexpr float ki = Kc / T_p1;

// --- アンチワインドアップ back-calculation（§3.2）---
// Tt初期値はKp/Ki(=T_p1)を目安とする。飽和からの復帰が遅い/速すぎる場合はここを調整
inline constexpr float BACK_CALC_TT = T_p1;

// --- 出力飽和 ---
inline constexpr float voltage_limit_ratio = 0.95f;

// --- 電圧FFの静的成分（車輪ごと）---
// u_static(v_wheel) = v_wheel/A_GAIN + U0_DEADZONE·sign(v_wheel)。v_wheelは目標軌道から運動学で求めた車輪の目標速度
// （v_ref ∓ ω_ref·WHEEL_DIFF_PER_DPS）。モータ1つの性質なので並進・回転で共通に使う（MotorDriver::update()）。
// |v_wheel|がこれ未満なら不感帯補償を入れない（停止指令時のビビリ防止）
inline constexpr float ZERO_VELOCITY_EPS = 1.0f;  // [mm/s]

// --- 電圧FFの慣性成分（並進，1次遅れモデル T_p1·dv/dt + v = K_p·u の逆モデルの微分項）---
// u_acc = (T_p1/K_p)·a_ref。目標加速度a_refはPlanProfileの軌道から与える（加減速区間だけ非0）。
// 2000mm/sまで180mmで加速（約11,100mm/s^2）すると約3.3V。0にすると加速度FFなし（静的FFのみ）
inline constexpr float ACCEL_FF_GAIN = T_p1 / K_p;  // [V/(mm/s^2)] ≈ 2.93e-4
}

// プロファイルの加速度の上限（PlanProfileが区間を積むときにvalidateSegment()で検査する。固定の試験はstatic_assertでも検査する）。
// 根拠：ファン20%のスリップ試験（plan_slip_limit_fan020, 2026-09-29）で，車輪と車体（IMU）の加速度の比が
// 較正範囲（0.93〜1.08）を超えたのは加速で約1.9G以上，減速で約2.5G以上。車体が実際に出せた加速度は
// 加速約1.7G，減速約2.4〜2.6G。電池の電圧降下（加速中7.5→約5.8V）も考え，余裕をとって加速1.5G，減速2.0Gとする
namespace config::profile_limit {
inline constexpr float G = 9806.65f;                   // [mm/s^2]
inline constexpr float MAX_ACCEL_X = 1.5f * G;         // [mm/s^2] 加速の上限
inline constexpr float MAX_DECEL_X = 2.0f * G;         // [mm/s^2] 減速の上限（大きさ）

// 回転の角加速度の上限（積むときに検査する）。[要調整] 同定していない。桁違いの指定を弾くための上限で，
// その場旋回の試験は2500dps/s，スラロームの設計値は最大10750dps/s（小回り90° 700mm/s）。
// スラロームが収まるよう10000から20000へ上げた（2026-10-02）
inline constexpr float MAX_ALPHA = 20000.f;   // [dps/s] 増速の上限
inline constexpr float MAX_ALPHA_DECEL = 20000.f;   // [dps/s] 減速の上限（大きさ）
}

// 位置のP制御（並進の外側ループ）：v_cmd = v_ref + kp(x_ref − x)。積分は持たない（AxisControllerの外側PIをki=0で使う）
namespace config::pid_position_x {
inline constexpr float ZETA = 1.0f;
inline constexpr float kp = 1.f / (4 * config::pid_velocity_x::LAMBDA * ZETA * ZETA);
inline constexpr float VELOCITY_CMD_LIMIT = 10000.f;   // [mm/s] 速度指令の上限（実質制限しない）
}

// 回転の制御（divergence_v2 8199a2f の構成をもとに，ジャイロで閉じて角速度の単位で書いたもの）：
//   ω_cmd = ω_ref + 角度PI（ジャイロの積分角度）→ 角速度PI（ジャイロ）＋電圧FF → 左右の電圧差
// 電圧FFは車輪ごとの静的成分（config::pid_velocity_xのモータモデル）と回転の慣性成分（ALPHA_FF_GAIN）。
// その場旋回の摩擦（約1.7V）はモデルより大きいので，残りの偏差は内側の積分で消す
// （rot_angle_pi_gyro/rot_pivot_pos90：内側がPだけだとω_cmdに対してジャイロが約100〜160dps遅れ，最終角度88.7°）。
// 比例ゲインはv2の値（角度[deg]→車輪の速度差 w [mm/s]，車輪速度[mm/s]→電圧）を w = ω·π/180·TREAD/2 で角速度へ換算したもの
namespace config::pid_rotation {
using config::mouse::WHEEL_DIFF_PER_DPS;

// --- 角度PI：出力は角速度の指令 ω_cmd [dps]（目標角速度ω_refをFFとして足す）---
inline constexpr float ANGLE_KP = 17.45f / WHEEL_DIFF_PER_DPS;   // [dps/deg] ≈ 33.3（v2 pid_ang.kp=17.45 mm/s/deg）
inline constexpr float ANGLE_KI = 5.0f / WHEEL_DIFF_PER_DPS;     // [dps/(deg·s)] ≈ 9.55（v2 pid_ang.ki=5.0）
inline constexpr float ANGLE_BACK_CALC_TT = ANGLE_KP / ANGLE_KI; // [s] アンチワインドアップ（Ti）
inline constexpr float OMEGA_CMD_LIMIT = 1000.f / WHEEL_DIFF_PER_DPS;   // [dps] ≈ 1910（車輪の速度差1000mm/s相当）

// --- 角速度PI（ジャイロ）：左右の電圧差 R − L [V] ---
// v2の車輪速度P（kp=2, duty[‰]=u/ad_batt·4000, ad_batt=Vbatt·4096/3.3·20k/53k）を電圧へ換算すると
// 片輪 0.0171 V/(mm/s)。左右の差（×2）と角速度への換算（×WHEEL_DIFF_PER_DPS）をかける
inline constexpr float WHEEL_KP = 2.f * 4000.f / 1000.f / (4096.f / 3.3f * 20000.f / 53000.f);   // [V/(mm/s)] 片輪
inline constexpr float OMEGA_KP = 2.f * WHEEL_KP * WHEEL_DIFF_PER_DPS;   // [V/dps] ≈ 0.0179
// [要調整] 積分時間。旧方式のTiスケジュール（360〜430dpsで約0.06s）を目安に固定値とした。
// 振動するなら長く，止まる前の残差が大きいなら短くする
inline constexpr float OMEGA_TI = 0.05f;                    // [s]
inline constexpr float OMEGA_KI = OMEGA_KP / OMEGA_TI;      // [V/(dps·s)]

// --- 電圧FFの慣性成分（回転）：左右の電圧差 [V] = ALPHA_FF_GAIN·α_ref ---
// 旧方式（角速度PI+FF，並進700mm/s・|ω|≦430dpsで同定）のA_FF = 3.0e-5 duty/(dps/s) を7.9Vで電圧へ換算した値。
// 同定表の高速側の値（不足側に弱く過大側に寛容）
inline constexpr float ALPHA_FF_GAIN = 3.0e-5f * 7.9f;   // [V/(dps/s)] ≈ 2.37e-4

// --- 出力飽和（左右の電圧差 R − L）：並進と同じく電池電圧に比例させる ---
// 旧来の固定2.6V（片輪1.3V）ではその場旋回の摩擦に足りず，pivot +90で約30dpsしか出なかった（rot_angle_pi/rot_pivot_pos90）。
// 並進と合わせた片輪の電圧はMotorDriver::dutyFromVoltage()でMAX_DUTYに収める
inline constexpr float VOLTAGE_LIMIT_RATIO = 0.95f;   // [V/V]
}

// 迷路と置き方（クラシック迷路，1区画180mm）。スラロームの試験と探索で共有する
namespace config::maze {
inline constexpr float CELL_MM = 180.f;
inline constexpr float WALL_HALF_MM = 6.f;   // 壁の厚さ12mmの半分（境界＝壁の中央から壁の面まで）

// 機体の後端を区画の後壁に当てて置いたとき，区画の後ろの境界から車軸まで（BACK_TO_AXLE_MM は config::mouse）
inline constexpr float START_MM = WALL_HALF_MM + config::mouse::BACK_TO_AXLE_MM;
}

// 壁の判定と横壁による向きの補正（common/wall_sensor.hpp, common/wall_control.hpp）。
// [要調整] 仮の値。Device → IR → Wall check で実機の値を見て決める：
//   THRESH_* … 区画境界の config::search::READ_LEAD_MM 手前（探索で壁を読む位置）で，壁があるときとないときの値の中間。
//              前は Device → IR → Front check で前左・前右それぞれの値を測り，出てきた閾値を使う
//   REF_*    … 区画の中心線上で両側に壁があるときの左右の値
namespace config::wall {
inline constexpr int16_t THRESH_LEFT = 300;
inline constexpr int16_t THRESH_RIGHT = 300;
// 前壁：前左・前右のどちらかが自分の閾値を超えたら壁あり（2つを同じ値にすると「大きい方の値 > 閾値」と同じ）。
// 2026-10-03 平均で判定していたら，探索で前壁を読み落として衝突した（平均 284 < 300）。大きい方で判定するよう変更
// 2026-10-03 Front check（静止，wall / no wall 各3回）：FL 壁あり ≥515・壁なし ≤199，FR 壁あり ≥241・壁なし ≤109。
// 探索中は前壁が弱く出る（FL 310〜460）ので，FR は探索のログ（壁なし最大 195，壁あり最小 214）の間に置いた。
// この組で探索のログ100回（衝突前）を誤りなく判定できる。[要調整] FR の余裕は約20。向きをずらした Front check で確かめる
inline constexpr int16_t THRESH_FRONT_LEFT = 357;    // 静止の中間値（余裕 316）
inline constexpr int16_t THRESH_FRONT_RIGHT = 205;

inline constexpr int16_t REF_LEFT = 744;
inline constexpr int16_t REF_RIGHT = 694;

// 補正のゲインは並進の目標速度 v に比例させる（KP = KP_PER_VELOCITY·v）。
// 横ずれ y [mm]・壁に対する向き θ [rad] で error ≈ KS·(y + L·θ)（横のセンサーは車軸の約 L 先の壁を見るので，
// 向きも効く。これが減衰になる）。走った距離 x で書くと y'' + G·L·y' + G·y = 0，G = KP·KS·(π/180)/v。
// KP ∝ v なら G が一定で，速度によらず同じ距離で収束する。
// 2026-10-06 探索のトレース（275区間）から KS ≈ 31〜32 count/mm，L ≈ 80〜120 mm（壁切れの OFFSET −91 とも合う）。
// 2% 収束 ≈ 4/(G·L/2) を 270 mm（1.5区画）にして G = 2.96e-4 /mm²，ζ ≈ 0.86（tools/wall_control_design.py）。
// 前の固定 KP = 0.02 は 500 mm/s で KP_PER_VELOCITY ≈ 4e-5 相当（収束に約 3.7 m）
inline constexpr float KP_PER_VELOCITY = 5.5e-4f;   // [dps/count per mm/s] 中心線からのずれ（センサー値の差）→ 補正の角速度
// 補正の角速度の上限も v に比例させる（曲率の上限。500 mm/s で前と同じ 90 dps）
inline constexpr float MAX_OMEGA_PER_VELOCITY = 0.18f;   // [dps per mm/s]
inline constexpr float MIN_VELOCITY = 100.f;    // [mm/s] これより遅いとき（停止・超信地旋回）は補正しない
}

// 壁切れによる距離の補正（common/wall_edge.hpp）。直進中に横のセンサー（左・右）の値が下がった
// （横壁が切れた）ところで，車軸の位置を区画境界から決まる位置にそろえる。
// 検出したときの車軸の位置 = 区画境界 + OFFSET_* + LAG_S·（目標速度）
// 横のセンサーは斜め前を向いていて，車軸の約 100 mm 先の壁を見ている。
// LAG_S は calib 300/500/700 各3回（tools/WALL_EDGE.md の R2）から。OFFSET_* は探索2回の壁切れの中央値から
// （calib の左 −101 / 右 −85 は，試験のコースで機体が横に寄っていた分。探索では左右とも約 −91）。
// [要調整] THRESH_* は仮の値のまま
namespace config::wall_edge {
// 壁ありとみなす値・壁が切れたとみなす値（ヒステリシス）。壁切れの位置は OFF を下回った位置
inline constexpr int16_t THRESH_ON_LEFT = 350;
inline constexpr int16_t THRESH_OFF_LEFT = 250;
inline constexpr int16_t THRESH_ON_RIGHT = 350;
inline constexpr int16_t THRESH_OFF_RIGHT = 250;

// [mm] 壁切れを検出したときの車軸の位置 − 区画境界（センサーの光が柱の向こう端を過ぎるので負，前を見るほど大きく負）
// [試験中 2026-10-03] 探索の中央値 左 −93.5 / −90.5，右 −89.9 / −88.9。calib は左 −101，右 −85
inline constexpr float OFFSET_LEFT_MM = -91.f;
inline constexpr float OFFSET_RIGHT_MM = -91.f;
// [s] 検出の遅れ（速いほど先で検出する分）。300〜700 mm/s で傾きが ±1 mm 以内に収まったので 0
inline constexpr float LAG_S = 0.f;

inline constexpr float MIN_WALL_MM = 20.f;     // [mm] これより短く見えた壁の切れ目は使わない（ノイズ・柱だけの反射）
// [mm] 予想位置からこれ以上ずれた壁切れは補正に使わない。
// [試験中 2026-10-03] 30 にしたら，探索で間違った壁切れが 24〜26 mm のずれで2回受け入れられ，機体が約 50 mm
// 前にずれて横壁を読み落とし衝突した。探索で使う S90 の出口のずれ（約 10 mm）に余裕を足して 20 にする
inline constexpr float WINDOW_MM = 20.f;
inline constexpr float MIN_VELOCITY = 100.f;   // [mm/s] これより遅いとき（加速の始め・停止・超信地旋回）は見ない

// 探索で補正をかけるか。false でも壁切れは検出してログ（search/<preset>_edges）に残す。
// 2026-10-03 探索で挙動を見るため true にした（R8）。おかしければ false に戻すと記録だけになる
inline constexpr bool SEARCH_CORRECTION = true;

// 最短走行（app/fast_run.cpp）：入口が区画中央のターン（L90・T180・IN45・IN135）の手前の区画境界をいくつ教えるか。
// 直前の境界で横壁が切れなくても，その1つ前で合わせられるように2つ。補正をかけるかはプリセットの "wall_edge"
inline constexpr int FAST_BOUNDARIES_PER_TURN = 2;
}

// 前壁の距離による S90 の入口の補正（common/front_correction.hpp）。探索で S90 を積むとき，読み位置
// （区画境界の config::search::READ_LEAD_MM 手前）で前壁があれば，前左・前右の値を換算表
// （tools/ir_calibration.json → config/front_distance_table.hpp）で距離にして，基準 REF_* とのずれ e を求める。
// e > 0 は機体が実際は後ろにいる（旋回を遅らせる）。補正 δ = clamp(GAIN·(|e| − DEADBAND_MM)·sign(e), ±MAX_MM)
// を pre-offset に足す。pre が負になる分は実測位置をずらして（Odometry::requestShiftX）位置制御で戻す。
// 補正するかはプリセットの front_correction。しないときも e と δ は探索のログ（front_err / front_corr）に残す。
// 2026-10-05 既存の探索ログでは e の σ≈11 mm，その後の壁切れのずれとの相関はほぼ 0（1回の読みではノイズが大きい）。
// 大きなずれだけを小さくするよう，不感帯と上限をつけている。tools/front_correction.py でログから確かめる
namespace config::front_correction {
// [mm] 読み位置での前壁の距離（換算表の値）の基準。Device → IR → Front check（wall）が静止で測った候補を出す。
// 2026-10-05 ゴール (1,0) の探索5走行の step 0（スタートから1区画目の読み。直後の壁切れのずれ約 2 mm で，
// 本当のずれはほぼ 0）の平均：FL 186.6〜187.3，FR 194.0〜195.9。前の値 198.5 / 204.9（前の迷路の探索ログの
// 「直進の後」の中央値）では step 0 で e ≈ −10 mm と出て，補正が負側にかかりすぎていた
inline constexpr float REF_LEFT_MM = 187.3f;
inline constexpr float REF_RIGHT_MM = 195.2f;
// [mm] 換算した距離がこの範囲にあるときだけ使う（両方とも）。読み位置の前壁は約 184 mm
inline constexpr float MIN_DISTANCE_MM = 140.f;
inline constexpr float MAX_DISTANCE_MM = 260.f;
inline constexpr float GAIN = 0.5f;
inline constexpr float DEADBAND_MM = 10.f;   // [mm] |e| がこれより小さければ補正しない
inline constexpr float MAX_MM = 15.f;        // [mm] 補正 δ の上限
}

// 探索（app/search.hpp）。速度・使うスラロームはプリセット（tools/search_presets.json）で選ぶ
namespace config::search {
// [mm] 区画境界のこれだけ手前で壁を読み，次の動作を積む。プリセットによらず同じ。
// 短すぎるとソルバーの計算が間に合わない（500mm/sで10mmなら20ms）
inline constexpr float READ_LEAD_MM = 10.f;
inline constexpr uint8_t GOAL_X = 7;
inline constexpr uint8_t GOAL_Y = 7;
inline constexpr uint16_t MAX_STEPS = 2048;     // 壁を読む回数の上限（ログの行数。往復でも16×16なら足りる）
inline constexpr float MIN_BATTERY_V = 7.4f;    // [V] これより低ければ走らない

// 3辺とも既知の区画で読んだ壁が地図と食い違ったときの立て直し（app/search.hpp）：
// 減速して止まり，1つ手前の区画の中央まで下がり，加速し直して同じ位置で壁を読み直す
inline constexpr float BRAKE_DECEL = 8000.f;        // [mm/s^2] 止まるときの減速度（1000mm/sから約63mm）
inline constexpr float BACK_SPEED = 200.f;          // [mm/s] 下がるときの最高速度
inline constexpr float BACK_ACCEL = 1000.f;         // [mm/s^2] 下がるときの加速度・減速度
inline constexpr uint32_t RECHECK_SETTLE_MS = 200;  // [ms] 止まった・下がった後に待つ時間
inline constexpr uint8_t MAX_MISMATCH_LOG = 32;     // 食い違いを覚えておく件数（終わりに LOG で出す）
// 既知の区間の直進を数える先読みにかけてよい時間（次の壁を読むまでの時間に対する割合）。
// 超えたら数えるのをやめる（加速する区間が短くなるだけ）
inline constexpr float CHAIN_BUDGET_RATIO = 0.4f;
}

// 探索中の迷路の保存（app/maze_store の journal）。ゴールに着いた後，途中で止まっても最短走行できるように
// 定期的に今の迷路をフラッシュへ追記する。消去は走る前だけ（走行中は消した後の枠に書くだけ）
namespace config::maze_save {
// true：走りながら1語ずつ書く（1語で CPU が典型 16us・最大 100us 止まり，1kHz の制御の割り込みがその分遅れる）。
// false：保存する歩で区画中央に止まってから書く（直進・行き止まりの歩だけ。ターンの歩なら次の機会に回す）
inline constexpr bool WHILE_RUNNING = true;
inline constexpr uint16_t EVERY_STEPS = 4;      // ゴールに着いた歩で1回，その後はこの歩数ごとに保存する（迷路が変わっていれば）
inline constexpr uint32_t RESERVE_SLOTS = 160;  // 探索を始めるとき，書く面にこれだけの空き枠がなければ面を消す（1面728枠）
}

namespace config::menu {
inline constexpr uint8_t MAX_CHILDREN = 10;  // 子ノード数の上限（現在の最大はFanの9項目）
// 全ノードの子の数の合計の上限（MenuNode が子へのポインタを詰めて置く共有の表の大きさ。1つ 4 バイト）。
// 足りなければ起動時に "menu: child pool full" と出て，あふれた子が出なくなる
inline constexpr uint16_t MAX_CHILD_LINKS = 512;
}

namespace config::mode_selector {
inline constexpr float ENC_THRESH = 10.f;
inline constexpr float IR_THRESH = 500.f;
inline constexpr float KORIKORI = 0.1f;

inline constexpr float ACC_THRESH = 0.f * config::imu::G; // mm/s^2
}