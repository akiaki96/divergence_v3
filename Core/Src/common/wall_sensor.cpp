#include "common/wall_sensor.hpp"
#include "config/mouse_config.hpp"
#include "device/device_instance.hpp"

namespace wall {

namespace {
// 機体の位置 → 実際にそのピンの値が入っている変数（wall_sensor.hpp の説明を参照）
const IrSensor& pick(Position p) {
    switch (p) {
    case left:        return irFL;   // PA3 SENSOR_L
    case front_left:  return irR;    // PA2 SENSOR_FL
    case front_right: return irL;    // PA0 SENSOR_FR
    default:          return irFR;   // PA1 SENSOR_R
    }
}
} // namespace

Snapshot read() {
    Snapshot s{};
    for (uint8_t p = 0; p < POSITION_COUNT; ++p) {
        s.value[p] = pick(static_cast<Position>(p)).filtered_;
    }
    return s;
}

bool hasLeft(const Snapshot& s) {
    return s.value[left] > config::wall::THRESH_LEFT;
}

bool hasFront(const Snapshot& s) {
    return (s.value[front_left] + s.value[front_right]) / 2 > config::wall::THRESH_FRONT;
}

bool hasRight(const Snapshot& s) {
    return s.value[right] > config::wall::THRESH_RIGHT;
}

} // namespace wall
