#pragma once

#include <cstdint>

namespace config::mouse {
inline constexpr float ENCODER_RES = 4096.f;
inline constexpr float GEAR_RATIO = 13.f / 42.f;
inline constexpr float WHEEL_RADIUS_MM = 23.6f / 2;
inline constexpr float TREAD_MM = 60.f;
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
inline constexpr float kd = 0.0f;

// --- アンチワインドアップ back-calculation（§3.2）---
// Tt初期値はKp/Ki(=T_p1)を目安とする。飽和からの復帰が遅い/速すぎる場合はここを調整
inline constexpr float BACK_CALC_TT = T_p1;

// --- 出力飽和 ---
inline constexpr float voltage_limit_ratio = 0.95f;

// --- feedforwardのゼロ速度judgement（停止指令時に不感帯補償を入れずビビリを防ぐ）---
inline constexpr float ZERO_VELOCITY_EPS = 1.0f;  // [mm/s]
}

namespace config::pid_position_x {
inline constexpr float ZETA = 1.0f;
inline constexpr float kp = 1.f / (4 * config::pid_velocity_x::LAMBDA * ZETA * ZETA);
// inline constexpr float kp = 0.f;

}

// PRBS入力設計（並進方向, data_analysis2/prbs_design.m）
// Tc下限(LFSRカバレッジ): 2.5*tau_slow/n, Tc上限(速い極を粗く均さない): tau_fast/2.8
// 採用: Tc=0.145s, n=8(PRBSクラスのタップ多項式に対応した固定値), duty=[0.08,0.16]
// 1試行の長さは走行距離を抑えるため3.0s→2.4s(80%)へ短縮。1試行あたりの励振ビット数が
// 減る分，同定用試行を6→8に増やしてデータの多様性を確保している（検証用2試行は据え置き）
namespace config::prbs_trans {
inline constexpr float TC_SEC       = 0.145f;
inline constexpr float DUTY_MIN     = 0.08f;
inline constexpr float DUTY_MAX     = 0.16f;
inline constexpr float DURATION_SEC = 2.4f;

// 同定用8試行 + 検証(holdout)用2試行のPRBSシード
inline constexpr uint16_t SEED_T01   = 0x1A2B;
inline constexpr uint16_t SEED_T02   = 0x3C4D;
inline constexpr uint16_t SEED_T03   = 0x5E6F;
inline constexpr uint16_t SEED_T04   = 0x7890;
inline constexpr uint16_t SEED_T05   = 0xABCD;
inline constexpr uint16_t SEED_T06   = 0xEF01;
inline constexpr uint16_t SEED_T07   = 0x4E2A;
inline constexpr uint16_t SEED_T08   = 0x9D31;
inline constexpr uint16_t SEED_VAL01 = 0x2468;
inline constexpr uint16_t SEED_VAL02 = 0x1357;
}

namespace config::menu {
inline constexpr uint8_t MAX_CHILDREN = 10;  // motor_sysid_prbs_(prbs 0~9)が最大
}

namespace config::mode_selector {
inline constexpr float ENC_THRESH = 10.f;
inline constexpr float IR_THRESH = 500.f;
inline constexpr float KORIKORI = 0.1f;

inline constexpr float ACC_THRESH = 0.f * config::imu::G; // mm/s^2
}