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


// lambda = 0.10 s
// K_p = 1562.7 mm/s/V
// T_{p1} = 0.459 s
// K_c = \frac{T_{p1}}{K_p \lambda}
namespace config::pid_velocity_x {
inline constexpr float LAMBDA = 0.10f;
inline constexpr float K_p = 1562.7f;
inline constexpr float T_p1 = 0.459f;
inline constexpr float Kc = T_p1 / (K_p * LAMBDA);
inline constexpr float kp = Kc;
inline constexpr float ki = Kc / T_p1;
inline constexpr float kd = 0.0f;

inline float a_gain = 1831.07f;
inline float u0_deadzone = 0.141;
inline float voltage_limit_ratio = 0.95f;
}

// PRBS入力設計（並進方向, data_analysis2/prbs_design.m）
// Tc下限(LFSRカバレッジ): 2.5*tau_slow/n, Tc上限(速い極を粗く均さない): tau_fast/2.8
// 採用: Tc=0.145s, n=8(PRBSクラスのタップ多項式に対応した固定値), duty=[0.08,0.16], 1試行3.0s
namespace config::prbs_trans {
inline constexpr float TC_SEC       = 0.145f;
inline constexpr float DUTY_MIN     = 0.08f;
inline constexpr float DUTY_MAX     = 0.16f;
inline constexpr float DURATION_SEC = 3.0f;

// 同定用6試行 + 検証(holdout)用2試行のPRBSシード
inline constexpr uint16_t SEED_T01   = 0x1A2B;
inline constexpr uint16_t SEED_T02   = 0x3C4D;
inline constexpr uint16_t SEED_T03   = 0x5E6F;
inline constexpr uint16_t SEED_T04   = 0x7890;
inline constexpr uint16_t SEED_T05   = 0xABCD;
inline constexpr uint16_t SEED_T06   = 0xEF01;
inline constexpr uint16_t SEED_VAL01 = 0x2468;
inline constexpr uint16_t SEED_VAL02 = 0x1357;
}

namespace config::menu {
inline constexpr uint8_t MAX_CHILDREN = 8;
}

namespace config::mode_selector {
inline constexpr float ENC_THRESH = 10.f;
inline constexpr float IR_THRESH = 500.f;
inline constexpr float KORIKORI = 0.1f;

inline constexpr float ACC_THRESH = 0.f * config::imu::G; // mm/s^2
}