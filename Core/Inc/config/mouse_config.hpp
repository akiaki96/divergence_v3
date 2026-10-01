#pragma once

#include <cstdint>
#include <numbers>

namespace config::mouse {
inline constexpr float ENCODER_RES = 4096.f;
inline constexpr float GEAR_RATIO = 13.f / 42.f;
inline constexpr float WHEEL_RADIUS_MM = 23.6f / 2;
inline constexpr float TREAD_MM = 60.f;

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

// --- feedforwardのゼロ速度judgement（停止指令時に不感帯補償を入れずビビリを防ぐ）---
inline constexpr float ZERO_VELOCITY_EPS = 1.0f;  // [mm/s]

// --- 加速度feedforward（1次遅れモデル T_p1·dv/dt + v = K_p·u の逆モデルの微分項）---
// u_acc = (T_p1/K_p)·a_ref。目標加速度a_refはPlanProfileの軌道から与える（加減速区間だけ非0）。
// 2000mm/sまで180mmで加速（約11,100mm/s^2）すると約3.3V。0にすると加速度FFなし（静的FFのみ）
inline constexpr float ACCEL_FF_GAIN = T_p1 / K_p;  // [V/(mm/s^2)] ≈ 2.93e-4
}

// 並進プロファイルの加速度の上限（コンパイル時にstatic_assertで検査する）。
// 根拠：ファン20%のスリップ試験（plan_slip_limit_fan020, 2026-09-29）で，車輪と車体（IMU）の加速度の比が
// 較正範囲（0.93〜1.08）を超えたのは加速で約1.9G以上，減速で約2.5G以上。車体が実際に出せた加速度は
// 加速約1.7G，減速約2.4〜2.6G。電池の電圧降下（加速中7.5→約5.8V）も考え，余裕をとって加速1.5G，減速2.0Gとする
namespace config::profile_limit {
inline constexpr float G = 9806.65f;                   // [mm/s^2]
inline constexpr float MAX_ACCEL_X = 1.5f * G;         // [mm/s^2] 加速の上限
inline constexpr float MAX_DECEL_X = 2.0f * G;         // [mm/s^2] 減速の上限（大きさ）

// v0→v1をdistance[mm]で等加速度に変化させる区間（vel2vel）の加速度 [mm/s^2]（v1^2 = v0^2 + 2·a·d）
constexpr float segmentAccel(float v0, float v1, float distance) {
    return (v1 * v1 - v0 * v0) / (2.f * distance);
}

// その区間の加速度が上限内か（加速はMAX_ACCEL_X，減速はMAX_DECEL_X）
constexpr bool withinAccelLimit(float v0, float v1, float distance) {
    float a = segmentAccel(v0, v1, distance);
    return (a >= 0.f) ? (a <= MAX_ACCEL_X) : (-a <= MAX_DECEL_X);
}
}

namespace config::pid_position_x {
inline constexpr float ZETA = 1.0f;
inline constexpr float kp = 1.f / (4 * config::pid_velocity_x::LAMBDA * ZETA * ZETA);
// inline constexpr float kp = 0.f;

}

// 回転の制御（divergence_v2 8199a2f の構成をもとに，ジャイロで閉じて角速度の単位で書いたもの）：
//   ω_cmd = ω_ref + 角度PI（ジャイロの積分角度）→ 角速度PI（ジャイロ）→ 左右の電圧差
// 電圧のFF・Tiスケジュールは持たない。摩擦（その場旋回で約1.7V）による偏差は内側の積分で消す
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

// --- 出力飽和（左右の電圧差 R − L）：並進と同じく電池電圧に比例させる ---
// 旧来の固定2.6V（片輪1.3V）ではその場旋回の摩擦に足りず，pivot +90で約30dpsしか出なかった（rot_angle_pi/rot_pivot_pos90）。
// 並進と合わせた片輪の電圧はMotorDriver::dutyFromVoltage()でMAX_DUTYに収める
inline constexpr float VOLTAGE_LIMIT_RATIO = 0.95f;   // [V/V]
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