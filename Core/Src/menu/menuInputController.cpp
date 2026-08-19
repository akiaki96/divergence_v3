#include "menu/menuInputController.hpp"
#include "common/debug.hpp"
#include "device/device_instance.hpp"
#include "config/mouse_config.hpp"

MenuInputController::MenuInputController(MenuController& menuController)
    : controller_(menuController)
{
}

void MenuInputController::syncUpdate() {
    if (0 < lockUntil_) {
        --lockUntil_;
        return;
    }

    motorRight.setDuty(0.f);
    if (lock_) {
        return;
    }

    encoderDistance_ += encoderRight.velocity() * config::control::DT_S;

    if (encoderDistance_ > config::mode_selector::ENC_THRESH) { // next
        encoderDistance_ = 0.f;
        motorRight.setDuty(-config::mode_selector::KORIKORI);
        controller_.next();
        ledBar16.set(controller_.index(), LedBarDotMode::dot8);
        isOnSelected_ = true;
        lock(10);
    } else if (encoderDistance_ < -config::mode_selector::ENC_THRESH) { // prev
        encoderDistance_ = 0.f;
        motorRight.setDuty(config::mode_selector::KORIKORI);
        controller_.prev();
        ledBar16.set(controller_.index(), LedBarDotMode::dot8);
        isOnSelected_ = true;
        lock(10);
    } else if (adcValue.irFL.filtered_ < config::mode_selector::IR_THRESH && adcValue.irFR.filtered_ > config::mode_selector::IR_THRESH) { // enter
        ledBar16.set(0x000F);
        controller_.enter();
        isOnEnter_ = true;
        lock(300);
    }
}

void MenuInputController::lock(uint32_t delay) {
    lockUntil_ = delay;
}

void MenuInputController::asyncUpdate() {
    while (true) {
        if (isOnSelected_) {
            controller_.currentInfo();

            lock_ = true;
            if (controller_.currentMenuNode()->child(controller_.index())->onSelected() != nullptr) {
                controller_.currentMenuNode()->child(controller_.index())->onSelected()();
            } else {
                LOG("on selected nullptr!!\r\n");
            }
            isOnSelected_ = false;
            lock_ = false;
        }
        if (isOnEnter_) {
            controller_.currentInfo();

            lock_ = true;
            if (controller_.currentMenuNode()->child(controller_.index())->onEnter() != nullptr) {
                controller_.currentMenuNode()->child(controller_.index())->onEnter()();
            } else {
                LOG("on enter nullptr!!\r\n");
            }
            isOnEnter_ = false;
            lock_ = false;
        }

        // LOG("ENC_DIST: %2f, LOCK:%d\r\n", encoderDistance_, lockUntil_);
    }
}
