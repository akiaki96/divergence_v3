#include "common/front_correction.hpp"
#include <algorithm>
#include <cmath>
#include <iterator>
#include "config/mouse_config.hpp"

namespace front_correction {

namespace {
using namespace config::front_correction;
using config::front_distance::Point;
} // namespace

float tableDistance(const Point* table, std::size_t count, float value) {
    if (count < 2 || value > table[0].value || value < table[count - 1].value) return NAN;
    for (std::size_t i = 1; i < count; ++i) {
        const Point& near = table[i - 1];
        const Point& far = table[i];
        if (value >= far.value) {
            float span = near.value - far.value;
            float frac = (span > 0.f) ? (near.value - value) / span : 0.f;
            return near.mm + frac * (far.mm - near.mm);
        }
    }
    return NAN;
}

float frontLeftMm(float value) {
    return tableDistance(config::front_distance::FRONT_LEFT, std::size(config::front_distance::FRONT_LEFT), value);
}

float frontRightMm(float value) {
    return tableDistance(config::front_distance::FRONT_RIGHT, std::size(config::front_distance::FRONT_RIGHT),
                         value);
}

namespace {
float nearMm(const Point* table, std::size_t count, float value) {
    if (count > 0 && value > table[0].value) return table[0].mm;
    return tableDistance(table, count, value);
}
} // namespace

float frontLeftNearMm(float value) {
    return nearMm(config::front_distance::FRONT_LEFT, std::size(config::front_distance::FRONT_LEFT), value);
}

float frontRightNearMm(float value) {
    return nearMm(config::front_distance::FRONT_RIGHT, std::size(config::front_distance::FRONT_RIGHT), value);
}

bool bothCloserThan(float value_left, float value_right, float near_mm) {
    return frontLeftNearMm(value_left) < near_mm && frontRightNearMm(value_right) < near_mm;   // NaN は false
}

float frontWallMm(float value_left, float value_right) {
    return 0.5f * (frontLeftNearMm(value_left) + frontRightNearMm(value_right));
}

float estimateError(float value_left, float value_right, float past_read) {
    float left = frontLeftMm(value_left);
    float right = frontRightMm(value_right);
    auto usable = [](float d) { return d >= MIN_DISTANCE_MM && d <= MAX_DISTANCE_MM; };   // NaN は false
    if (!usable(left) || !usable(right)) return NAN;
    // 2つの平均：機体が少し斜めでも，片方が近く・もう片方が遠く見える分が打ち消し合う。
    // 読む位置を過ぎていれば，その分だけ前壁は近く見えるはず
    return 0.5f * ((left - REF_LEFT_MM) + (right - REF_RIGHT_MM)) + past_read;
}

float correction(float error) {
    if (!(std::fabs(error) > DEADBAND_MM)) return 0.f;   // NaN も補正しない
    float magnitude = std::min(GAIN * (std::fabs(error) - DEADBAND_MM), MAX_MM);
    return std::copysign(magnitude, error);
}

Split split(float delta, float pre_offset, float min_pre) {
    float pre = pre_offset + delta;
    if (pre >= min_pre || pre == 0.f) return {delta, 0.f};
    // pre を 0 にしてもまだ前に出すぎている分は，実測位置を前へずらして位置制御で引き戻す
    // （実測が前に進むので，機体は目標に合わせて遅れる）
    return {-pre_offset, -pre};
}

} // namespace front_correction
