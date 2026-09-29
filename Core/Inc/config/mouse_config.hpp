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

// 回転角速度 PI + feedforward 制御（data_analysis2/04_rot_omega_control）
//
// 並進と同じく電圧空間で計算する。同定・FF設計はduty空間（実測の中央付近 7.9V）で行ったので，
// duty値×BATT_V_REFで電圧へ換算する（回転の感度は電圧にほぼ比例：rot_gain_scheduling_plan.md §20.3, §23）。
// プラントは動作点で T 10→160ms, K 550→3200dps/duty と非線形（並進700mm/s, |ω|≦430dpsで同定）。
// FFは線形近似 u_ff = ω_ref/K± + A_FF·α_ref。K±は430dpsで実測u_ssの下限付近（保守側：FF過大はOSになる）で，
// 100〜250dpsで不足する分（約0.05duty）はPIの積分が補う。
namespace config::pid_omega {
inline constexpr float BATT_V_REF = 7.9f;  // [V] duty値→電圧の換算基準

// --- feedforward（線形）---
inline constexpr float K_FF_POS = 2400.f / BATT_V_REF;   // [dps/V] ω_ref >= 0
inline constexpr float K_FF_NEG = 1950.f / BATT_V_REF;   // [dps/V] ω_ref < 0（-側は約0.03duty多く要る）
inline constexpr float A_FF = 3.0e-5f * BATT_V_REF;      // [V/(dps/s)] 加速度FF係数 T/K（F5表の高速側。不足側に弱く過大側に寛容）

// --- PIフィードバック ---
// Kc=5e-4duty/dps（実機E3〜E11で使用）。Kcは動作点によらずほぼ一定でよく，Ti(=Kc/Ki)のみ|ω_ref|でスケジュールする
inline constexpr float kp = 5.0e-4f * BATT_V_REF;        // [V/dps]
inline constexpr float kd = 0.0f;
inline constexpr int TI_TABLE_SIZE = 6;
inline constexpr float TI_OMEGA_BP[TI_TABLE_SIZE] = {0.f, 100.f, 200.f, 250.f, 400.f, 430.f};   // [dps]
inline constexpr float TI_S_BP[TI_TABLE_SIZE]     = {0.0090f, 0.01725f, 0.0345f, 0.0420f, 0.0615f, 0.06525f};   // [s]

// --- 出力飽和（duty_diffの電圧換算。7.9Vで約0.33duty。0.28duty超は開ループ未検証：§23.1）---
inline constexpr float VOLTAGE_LIMIT = 2.6f;   // [V]

// 閉ループ時定数の目安（Kc×接線ゲイン/T ≈ 5〜9万dps/s/duty から λ≈0.02〜0.04s）。角度ループの設計に使う
inline constexpr float LAMBDA = 0.03f;   // [s]
}

// 角度のP制御（外側ループ）。位置と同じ設計則：ω_cmd = ω_ref + kp(θ_ref − θ)
namespace config::pid_angle {
inline constexpr float ZETA = 1.0f;
inline constexpr float kp = 1.f / (4 * config::pid_omega::LAMBDA * ZETA * ZETA);   // [dps/deg]
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