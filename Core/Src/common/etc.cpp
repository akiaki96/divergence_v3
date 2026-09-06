#include "common/etc.hpp"
#include "device/device_instance.hpp"
#include "config/mouse_config.hpp"

void haltByAccZ(void) {
    while (imu.accelZ() > config::mode_selector::ACC_THRESH) {
    }
}