#include "common/etc.hpp"
#include "device/device_instance.hpp"
#include "config/mouse_config.hpp"
#include "common/debug.hpp"

void haltByAccZ(void) {
    LOG("haltByAccZ\r\n");
    while (imu.accelZ() + config::imu::G > config::mode_selector::ACC_THRESH) {
    }
    LOG("haltByAccZ end\r\n");
}