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
//   THRESH_* … 区画境界の config::search::READ_LEAD_MM 手前（探索で壁を読む位置）で，壁があるときとないときの値の中間
//   REF_*    … 区画の中心線上で両側に壁があるときの左右の値
namespace config::wall {
inline constexpr int16_t THRESH_LEFT = 300;
inline constexpr int16_t THRESH_RIGHT = 300;
inline constexpr int16_t THRESH_FRONT = 300;    // 前の左右（FL, FR）の平均と比べる

inline constexpr int16_t REF_LEFT = 1000;
inline constexpr int16_t REF_RIGHT = 1000;

inline constexpr float KP = 0.02f;              // [dps/count] 中心線からのずれ（センサー値の差）→ 補正の角速度
inline constexpr float MAX_OMEGA = 90.f;        // [dps] 補正の角速度の上限
inline constexpr float MIN_VELOCITY = 100.f;    // [mm/s] これより遅いとき（停止・超信地旋回）は補正しない
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
}

namespace config::menu {
inline constexpr uint8_t MAX_CHILDREN = 10;  // 子ノード数の上限（現在の最大はFanの9項目）
}

namespace config::mode_selector {
inline constexpr float ENC_THRESH = 10.f;
inline constexpr float IR_THRESH = 500.f;
inline constexpr float KORIKORI = 0.1f;

inline constexpr float ACC_THRESH = 0.f * config::imu::G; // mm/s^2
}