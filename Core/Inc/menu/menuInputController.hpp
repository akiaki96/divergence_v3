#pragma once

#include "menu/menuController.hpp"

class MenuInputController {
public:
    explicit MenuInputController(MenuController& menuController);

    void syncUpdate();
    void asyncUpdate();

    void enable();
    void disable();

private:
    void lock(uint32_t delay);

    MenuController controller_;

    uint32_t lockUntil_ = 0; // ms
    float encoderDistance_ = 0.0f;

    bool lock_ = false;

    bool onSelected_ = false;
    bool onEnter_ = false;
};