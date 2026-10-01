#include "device/motorDriver.hpp"
#include "device/device_instance.hpp"
#include "stm32f4xx_hal.h"
#include "tim.h"
#include "config/mouse_config.hpp"

MotorDriver::MotorDriver(Motor& left, Motor& right)
    :motorLeft_(left),
    motorRight_(right)
{}

// 電圧FFの静的成分 [V]（車輪1つ）：u = v/A_GAIN + U0_DEADZONE·sign(v)。
// vは目標軌道から求めた車輪の目標速度。|v|がZERO_VELOCITY_EPS未満なら0（停止指令時に不感帯補償を入れない）
static float wheel_static_ff(float v) {
    using namespace config::pid_velocity_x;
    if (v > ZERO_VELOCITY_EPS)  return v / A_GAIN + U0_DEADZONE;
    if (v < -ZERO_VELOCITY_EPS) return v / A_GAIN - U0_DEADZONE;
    return 0.f;
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
    trans_.reset();
    rot_.reset();
    state = MotorDriverState::setVelocity;
}

void MotorDriver::update(const AxisReference& trans_ref, const AxisMeasurement& trans,
                         const AxisReference& rot_ref, const AxisMeasurement& rot) {
    if (state != MotorDriverState::setVelocity) return;

    // ---- 電圧FF：目標軌道だけから作る（実測値を通さない）----
    // 静的成分は車輪ごと（目標軌道から運動学で求めた車輪の目標速度にモータモデルを当てる），
    // 慣性成分は軸ごと（並進は車体の質量，回転はヨーの慣性で係数が違う）。
    // 車輪ごとの成分は左右の平均（並進）と差 R − L（回転）に直して各軸の内側PIに渡す
    float wheel_diff_ref = rot_ref.vel * config::mouse::WHEEL_DIFF_PER_DPS;   // (v_R − v_L)/2 [mm/s]
    float ff_left  = wheel_static_ff(trans_ref.vel - wheel_diff_ref);
    float ff_right = wheel_static_ff(trans_ref.vel + wheel_diff_ref);
    float trans_ff = (ff_left + ff_right) / 2.f + config::pid_velocity_x::ACCEL_FF_GAIN * trans_ref.acc;
    float rot_ff   = (ff_right - ff_left) + config::pid_rotation::ALPHA_FF_GAIN * rot_ref.acc;

    // ---- 追従制御：並進・回転とも 外側（位置/角度）→ 内側（速度/角速度）のカスケード ----
    float vbatt = battery.voltage();
    float base_batt = trans_.update(trans_ref, trans, trans_ff, config::pid_velocity_x::voltage_limit_ratio * vbatt);
    float diff_batt = rot_.update(rot_ref, rot, rot_ff, config::pid_rotation::VOLTAGE_LIMIT_RATIO * vbatt);

    // diff = R − L（正でω正）。v_L = v − diff/2, v_R = v + diff/2 のkinematic配分
    float half_diff = diff_batt / 2.f;
    setDuty(dutyFromVoltage(base_batt - half_diff), dutyFromVoltage(base_batt + half_diff));
}
