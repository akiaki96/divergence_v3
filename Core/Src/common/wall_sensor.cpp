#include "common/wall_sensor.hpp"
#include "config/mouse_config.hpp"
#include "device/device_instance.hpp"

namespace wall {

namespace {
// 機体の位置 → その位置のセンサーの変数（wall_sensor.hpp の説明を参照）
const IrSensor& pick(Position p) {
    switch (p) {
    case left:        return irL;
    case front_left:  return irFL;
    case front_right: return irFR;
    default:          return irR;
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

bool hasFrontLeft(const Snapshot& s) {
    return s.value[front_left] > config::wall::THRESH_FRONT_LEFT;
}

bool hasFrontRight(const Snapshot& s) {
    return s.value[front_right] > config::wall::THRESH_FRONT_RIGHT;
}

bool hasFront(const Snapshot& s) {
    return hasFrontLeft(s) || hasFrontRight(s);
}

bool hasRight(const Snapshot& s) {
    return s.value[right] > config::wall::THRESH_RIGHT;
}

} // namespace wall
