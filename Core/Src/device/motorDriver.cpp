#include "device/motorDriver.hpp"
#include "device/device_instance.hpp"
#include "stm32f4xx_hal.h"
#include "tim.h"
#include "config/mouse_config.hpp"

MotorDriver::MotorDriver(Motor& left, Motor& right)
    :motorLeft_(left),
    motorRight_(right)
{}

// 並進速度feedforward（translational_gain_tuning.md §2.2の定常成分のみの簡易版）:
//   u_ff(v) = v/A_GAIN + U0_DEADZONE*sign(v)
// 目標軌道がstep（滑らかな加減速プロファイルでない）前提のため微分項(T/K * v_dot)は省略し，
// 過渡応答はPIフィードバックに委ねる。台形加減速プロファイルを導入する場合はここを拡張する。
float velocity_x_ff(float velocity_x) {
    float abs_v = (velocity_x < 0.f) ? -velocity_x : velocity_x;
    if (abs_v < config::pid_velocity_x::ZERO_VELOCITY_EPS) return 0.f;

    float sign = (velocity_x > 0.f) ? 1.f : -1.f;
    return (velocity_x / config::pid_velocity_x::A_GAIN) + sign * config::pid_velocity_x::U0_DEADZONE;
}

static float abs_f(float x) {
    return (x < 0.f) ? -x : x;
}

// 回転角速度feedforward [V]（data_analysis2/04_rot_omega_control, 線形近似）:
//   u_ff(ω_ref, α_ref) = ω_ref/K_FF± + A_FF·α_ref
// 静的項の傾きはω_refの符号で正/負を選ぶ（-側は同じ角速度でより大きな出力が要る）。0では両側とも0で連続。
// 並進と違い加速度項を入れる：回転は必要出力の大部分を積分で作るとOSになり，加速度FFで遅れを消すのが本質（§17）
float omega_ff(float omega, float alpha) {
    float k = (omega >= 0.f) ? config::pid_omega::K_FF_POS : config::pid_omega::K_FF_NEG;
    return (omega / k) + config::pid_omega::A_FF * alpha;
}

// 指令角速度|ω_ref|に対する積分時間Ti [s]（区分線形。表の範囲外は端の値）
static float omega_ti_schedule(float omega) {
    using namespace config::pid_omega;
    float x = abs_f(omega);
    if (x <= TI_OMEGA_BP[0]) return TI_S_BP[0];
    for (int i = 1; i < TI_TABLE_SIZE; i++) {
        if (x <= TI_OMEGA_BP[i]) {
            float r = (x - TI_OMEGA_BP[i - 1]) / (TI_OMEGA_BP[i] - TI_OMEGA_BP[i - 1]);
            return TI_S_BP[i - 1] + r * (TI_S_BP[i] - TI_S_BP[i - 1]);
        }
    }
    return TI_S_BP[TI_TABLE_SIZE - 1];
}

void MotorDriver::init() {
    pid_velocity_x_.setGains(
        config::pid_velocity_x::kp,
        config::pid_velocity_x::ki,
        config::pid_velocity_x::kd,
        velocity_x_ff,
        config::pid_velocity_x::BACK_CALC_TT
    );

    pid_position_x_.setGains(
        config::pid_position_x::kp,
        0.f,
        0.f,
        [](float) { return motorDriver.getTargetVelocityX(); },
        0.f
    );

    // Ki・back-calculation時定数はupdate()で|ω_ref|に応じて毎tick差し替える（初期値はω_ref=0の値）
    float ti0 = omega_ti_schedule(0.f);
    pid_omega_.setGains(
        config::pid_omega::kp,
        config::pid_omega::kp / ti0,
        config::pid_omega::kd,
        [](float) { return omega_ff(motorDriver.getTargetOmega(), motorDriver.getTargetAlpha()); },
        ti0
    );

    pid_angle_.setGains(
        config::pid_angle::kp,
        0.f,
        0.f,
        [](float) { return motorDriver.getTargetOmega(); },
        0.f
    );
}

void MotorDriver::enable() {
    HAL_GPIO_WritePin(MOTOR_STBY_GPIO_Port, MOTOR_STBY_Pin, GPIO_PIN_SET);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
}

void MotorDriver::disable() {
    HAL_GPIO_WritePin(MOTOR_STBY_GPIO_Port, MOTOR_STBY_Pin, GPIO_PIN_RESET);
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);
}

float MotorDriver::getLeftDuty(void) const {
    return motorLeft_.getDuty();
}
float MotorDriver::getRightDuty(void) const {
    return motorRight_.getDuty();
}

float MotorDriver::dutyFromVoltage(float voltage) const {
    float vbatt = battery.voltage();
    // 安全下限：Vbatt異常低下（センサ異常・切断等）時のゼロ割り/暴走防止
    if (vbatt < config::motor::kVbattMinSafe) vbatt = config::motor::kVbattMinSafe;

    float duty = voltage / vbatt;
    // duty飽和処理
    if (duty >  config::motor::MAX_DUTY) duty =  config::motor::MAX_DUTY;
    if (duty < -config::motor::MAX_DUTY) duty = -config::motor::MAX_DUTY;
    return duty;
}

void MotorDriver::switchToVelocityX() {
    pid_velocity_x_.reset();
    pid_omega_.reset();
    pid_angle_.reset();
    state = MotorDriverState::setVelocity;
}

void MotorDriver::update(float current_velocity_x, float current_position_x, float current_omega, float current_angle) {
    switch (state) {
        case MotorDriverState::off:
            setDuty(0.f, 0.f);
        break;

        case MotorDriverState::setDuty:
        break;

        case MotorDriverState::modeSelecting:
        break;

        case MotorDriverState::setVelocity: {

            target_velocity_x_ += target_accel_x_ * config::control::DT_S;  // 速度指令を積分して目標速度を更新
            // 終端速度に達したら固定して加速度を切る（減速で0を越えて負になる・加速で行き過ぎるのを防ぐ）
            if ((target_accel_x_ > 0.f && target_velocity_x_ >= end_velocity_x_) ||
                (target_accel_x_ < 0.f && target_velocity_x_ <= end_velocity_x_)) {
                target_velocity_x_ = end_velocity_x_;
                target_accel_x_ = 0.f;
            }
            target_position_x_ += target_velocity_x_ * config::control::DT_S;  // 速度指令を積分して目標位置を更新

            float local_target_velocity_x = pid_position_x_.update(target_position_x_, current_position_x);

            bool saturated = false;
            float limit = config::pid_velocity_x::voltage_limit_ratio * battery.voltage();
            float base_batt = pid_velocity_x_.update(local_target_velocity_x, current_velocity_x, limit, saturated);
            velocity_pid_saturated_ = saturated;

            // ---- 回転：角度P（外側）→ 角速度PI+FF（内側）。並進と同じ2自由度カスケード ----

            target_omega_ += target_alpha_ * config::control::DT_S;  // 角加速度指令を積分して目標角速度を更新
            // 終端角速度に達したら固定して角加速度を切る（並進と同じ）
            if ((target_alpha_ > 0.f && target_omega_ >= end_omega_) ||
                (target_alpha_ < 0.f && target_omega_ <= end_omega_)) {
                target_omega_ = end_omega_;
                target_alpha_ = 0.f;
            }
            target_angle_ += target_omega_ * config::control::DT_S;  // 目標角速度を積分して目標角度を更新

            float local_target_omega = pid_angle_.update(target_angle_, current_angle);

            // FF・Tiのスケジュール変数は補正を含まない目標角速度ω_ref（既知・無雑音）
            float ti = omega_ti_schedule(target_omega_);
            pid_omega_.ki = config::pid_omega::kp / ti;   // 積分項は出力単位で保持されるのでKiを変えても出力は跳ばない
            pid_omega_.back_calc_tt = ti;

            bool omega_saturated = false;
            float diff_batt = pid_omega_.update(local_target_omega, current_omega, config::pid_omega::VOLTAGE_LIMIT, omega_saturated);
            omega_pid_saturated_ = omega_saturated;

            // diff = R − L（正でω正）。v_L = v − diff/2, v_R = v + diff/2 のkinematic配分
            float half_diff = diff_batt / 2.f;
            setDuty(dutyFromVoltage(base_batt - half_diff), dutyFromVoltage(base_batt + half_diff));
            break;
        }
    }
}