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

// 回転角速度PI制御用feedforward：2自由度制御ではなく純粋PIとするため常に0を返す
// （config::pid_omega参照：Kp_rotの不確かさが大きく，FFのモデル誤差より
// 積分によるロバストな追従を優先する設計）
float omega_ff_zero(float) {
    return 0.f;
}

// 指令角速度ω_refに対する積分時間Ti [s]（config::pid_omega::TI_*の区分線形補間）。
// ω_refの符号で正/負の表を選ぶ（方向別Ti，config::pid_omega参照）
static float omega_ti_schedule(float omega_dps) {
    using namespace config::pid_omega;
    const float* ti_tbl = (omega_dps >= 0.f) ? TI_S_BP_POS : TI_S_BP_NEG;
    float w = (omega_dps < 0.f) ? -omega_dps : omega_dps;
    if (w <= TI_OMEGA_BP[0]) return ti_tbl[0];
    for (int i = 1; i < TI_TABLE_SIZE; ++i) {
        if (w <= TI_OMEGA_BP[i]) {
            float r = (w - TI_OMEGA_BP[i - 1]) / (TI_OMEGA_BP[i] - TI_OMEGA_BP[i - 1]);
            return ti_tbl[i - 1] + r * (ti_tbl[i] - ti_tbl[i - 1]);
        }
    }
    return ti_tbl[TI_TABLE_SIZE - 1];
}

// Ki=Kc/Ti, Tt=Ti をスケジュールに合わせて設定する。積分項は出力単位で保持されるので
// Kiを変えても出力は跳ばない（バンプレス）
static void set_omega_gains(PIDController& pid, float omega_ref_dps) {
    float ti = omega_ti_schedule(omega_ref_dps);
    pid.setGains(
        config::pid_omega::kp,
        config::pid_omega::kp / ti,
        config::pid_omega::kd,
        omega_ff_zero,
        ti
    );
}

void MotorDriver::enableOmegaControl() {
    omega_control_enabled_ = true;
    pid_omega_.reset();
    omega_ref_ = imu.gyroZ();   // 指令ランプの起点を現在の角速度に合わせる
}

void MotorDriver::init() {
    pid_velocity_x_.setGains(
        config::pid_velocity_x::kp,
        config::pid_velocity_x::ki,
        config::pid_velocity_x::kd,
        velocity_x_ff,
        config::pid_velocity_x::BACK_CALC_TT
    );
    set_omega_gains(pid_omega_, 0.f);   // 角速度PIの初期ゲイン（Ti(0)）を設定
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

void MotorDriver::setLampGrad(float lamp) {
    lamp_grad_ = lamp;
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
    duty_diff_ = 0.f;
    applied_duty_diff_ = 0.f;
    prbs_rot_diff_ = nullptr;
    omega_control_enabled_ = false;
    target_omega_ = 0.f;
    omega_ref_ = 0.f;
    pid_omega_.reset();
    state = MotorDriverState::setVelocity;
}


void MotorDriver::update() {
    switch (state) {
        case MotorDriverState::off:
            setDuty(0.f, 0.f);
        break;

        case MotorDriverState::setDuty:
        break;

        case MotorDriverState::lampDuty:
            setDuty(
                getLeftDuty() + lamp_grad_*config::control::DT_S,
                getRightDuty() + lamp_grad_*config::control::DT_S
            );
        break;

        case MotorDriverState::setVoltage:
            setDuty(
                dutyFromVoltage(target_voltage_L_),
                dutyFromVoltage(target_voltage_R_)
            );
        break;

        case MotorDriverState::prbsDuty: {
            if (prbs_ == nullptr || prbs_->isFinished()) {
                setBreak();
                state = MotorDriverState::off;
                break;
            }
            float duty = prbs_->update();
            setDuty(duty, duty);   // 並進方向：左右同相
            break;
        }

        case MotorDriverState::modeSelecting:
        break;

        case MotorDriverState::setVelocity: {
            bool saturated = false;
            float limit = config::pid_velocity_x::voltage_limit_ratio * battery.voltage();
            float base_batt = pid_velocity_x_.update(velocity_x_, (encoderLeft.velocity() + encoderRight.velocity()) / 2.f, limit, saturated);
            velocity_pid_saturated_ = saturated;

            // 注意：setVoltage()はstateをMotorDriverState::setVoltageへ書き換えてしまうため，
            // ここで呼ぶとPIDが次tickから二度と回らなくなる（固定電圧のstep入力に化ける）。
            // stateをsetVelocityに保ったまま，直接duty変換のみ行う。
            // duty_diff_（回転方向のstep/PRBS同定用）はduty空間で左右に重畳する：
            // v_L = v* - diff/2, v_R = v* + diff/2 のkinematic配分に対応。
            // 優先度：PRBS励振(同定用) > 角速度PI閉ループ(enableOmegaControl) > 静的setDutyDiff()
            float diff = duty_diff_;
            if (prbs_rot_diff_ != nullptr && !prbs_rot_diff_->isFinished()) {
                diff = prbs_rot_diff_->update();
            } else if (omega_control_enabled_) {
                // 目標角速度を最大角加速度でレート制限し，その指令値でPIとTiスケジュールを回す
                float max_step = omega_accel_max_ * config::control::DT_S;
                float d = target_omega_ - omega_ref_;
                if (d > max_step) d = max_step;
                else if (d < -max_step) d = -max_step;
                omega_ref_ += d;
                set_omega_gains(pid_omega_, omega_ref_);

                bool omega_sat = false;
                diff = pid_omega_.update(omega_ref_, imu.gyroZ(), config::pid_omega::DUTY_DIFF_LIMIT, omega_sat);
                omega_saturated_ = omega_sat;
            }
            applied_duty_diff_ = diff;   // ログ用：PRBS/PI駆動時もgetDutyDiff()で実値を参照できるようにする
            float base_duty = dutyFromVoltage(base_batt);
            float half_diff = diff / 2.f;
            setDuty(base_duty - half_diff, base_duty + half_diff);
            break;
        }
    }
}