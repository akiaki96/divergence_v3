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

// 回転方向のstep応答事前同定（system_identification_flow.md §2 [2]）
// 並進速度を閉ループでTRANSLATION_VELOCITY_MM_Sに固定した状態で，左右duty差
// （DUTY_DIFF_1〜5, R-L）をステップ印加し非線形性・IMU飽和有無を確認する。
//
// duty_diff0.02/0.04/0.06実測結果（data_analysis2/rot_step_v700_report.md）：
//  - 定常ヨーレートは60dps止まり（IMU飽和±2000dpsの3%）で大きく余裕あり
//  - duty水準が大きいほどゲインも大きくなる非線形性を確認（707→804 dps/duty）
// 上記を踏まえ0.10, 0.14を追加。並進700mm/s時の基準duty実測値は約0.0714で，
// DUTY_DIFF_5=0.14は左右差の半分(0.07)が基準dutyとほぼ一致する上限
// （これを超えると片輪のduty符号が反転し，動作レジームが大きく変わる）。
namespace config::rot_step_v700 {
inline constexpr float TRANSLATION_VELOCITY_MM_S = 700.f;
inline constexpr uint32_t ACCEL_MS = 800;   // 並進速度700mm/sを閉ループで立ち上げる時間
inline constexpr uint32_t TEST_MS  = 600;   // duty差を保持する時間（励振時間）

inline constexpr float DUTY_DIFF_1 = 0.02f;
inline constexpr float DUTY_DIFF_2 = 0.04f;
inline constexpr float DUTY_DIFF_3 = 0.06f;
inline constexpr float DUTY_DIFF_4 = 0.10f;
inline constexpr float DUTY_DIFF_5 = 0.14f;
// 実運用目標（700mm/s時：角加速度目安2500deg/s^2，最高角速度目安430deg/s）に対し，
// duty_diff<=0.14までの実測ではヨーレートが最大でも171dps程度までしか届いておらず
// 大幅に不足している。運用域に向けた特性把握のため追加した水準（片輪はさらに深く
// 負転する領域に入る）
inline constexpr float DUTY_DIFF_6 = 0.20f;
inline constexpr float DUTY_DIFF_7 = 0.28f;
}

// 回転方向PRBS本同定（data_analysis2/prbs_rot_design.m，system_identification_flow.md §2 [3]）
// duty_diff振幅はrot_step_v700で確認した「クリーン」線形領域の上限(±0.06)を採用。
// Tc=4msはn=8固定のLFSRで高域分解能(tau_fast/2.8)側を優先した値
// （下限2.5*tau_slow/nとは両立しないため）。全周期255クロックが約1.02sに収まるため，
// 1試行1.1sでほぼ全周期を励振できる。
namespace config::prbs_rot {
inline constexpr float TRANSLATION_VELOCITY_MM_S = 700.f;
inline constexpr uint32_t ACCEL_MS   = 800;    // 並進速度700mm/sを閉ループで立ち上げる時間
inline constexpr float TC_SEC        = 0.004f; // クロック周期 [s]（1ms tick整数化）
inline constexpr float DUTY_DIFF_AMP = 0.06f;  // PRBS振幅（±）
inline constexpr float DURATION_SEC  = 1.1f;   // 1試行の励振時間

// 同定用4試行 + 検証(holdout)用2試行のPRBSシード
inline constexpr uint16_t SEED_T01   = 0x6A2D;
inline constexpr uint16_t SEED_T02   = 0x3F17;
inline constexpr uint16_t SEED_T03   = 0x9C84;
inline constexpr uint16_t SEED_T04   = 0x1E5B;
inline constexpr uint16_t SEED_VAL01 = 0x7D93;
inline constexpr uint16_t SEED_VAL02 = 0x4B26;
}

// 回転角速度PI制御（2自由度(FF)ではなく純粋PI。gain不確かさへの頑健性を優先）
//
// [設計方針] rot_step_v700_report.mdより，回転方向のプラントゲインKpは707〜2815dps/duty
// と振幅依存で大きく変動し（±正負非対称・duty不感帯突入による構造変化あり），単一のFFでは
// モデル誤差が大きい。PI（特に積分項）は定常ゲインの不確かさに対してロバストなため，FFを
// 使わずI主体で目標角速度へ追従させる。IMC整定 Kc=Tp1/(Kp*λ), Ki=Kc/Tp1 において
// Tp1_ROTが23ms程度と非常に小さいため，Ki/Kp比は自然に約1/Tp1≈43倍となり，
// 結果的に「Iゲインが大きい」制御になる。
//
// [要調整] KP_ROTはduty_diff=0.20水準（整定確認済みの中では最大）の実測平均を保守的に採用。
// TP1_ROTはprbs_rot_identification_report.mdのPRBS本同定値。LAMBDA_ROTは未実験の初期値
// （やや保守的に設定）。DUTY_DIFF_LIMITはrot_step_v700で線形性・整定を確認済みの範囲。
// いずれも実機でオーバーシュート・整定時間・430dps付近での挙動を見ながら調整すること。
namespace config::pid_omega {
inline constexpr float KP_ROT  = 1250.f;   // [dps/duty] duty_diff=±0.20実測平均（保守的）
inline constexpr float TP1_ROT = 0.0231f;  // [s] PRBS本同定（prbs_rot_identification_report.md）

inline constexpr float LAMBDA_ROT = 0.05f;  // [s] 閉ループ時定数（初期値，要実機調整）
inline constexpr float Kc_rot = TP1_ROT / (KP_ROT * LAMBDA_ROT);
inline constexpr float kp = Kc_rot;
inline constexpr float ki = Kc_rot / TP1_ROT;
inline constexpr float kd = 0.0f;

inline constexpr float BACK_CALC_TT = TP1_ROT;

// 出力(duty_diff)飽和：rot_step_v700で線形性・整定を確認済みの範囲に制限
// （±0.20は整定を確認済み。±0.28は600msで未整定のため含めない）
inline constexpr float DUTY_DIFF_LIMIT = 0.20f;
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