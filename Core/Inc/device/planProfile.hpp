#pragma once

#include "device/motorDriver.hpp"

enum class PlanProfileState {
    off,
    velocityStep,
    velocityRamp,
};

class PlanProfile {
public:
    explicit PlanProfile(MotorDriver& motorDriver);
    void init();
    void update();

    void enable();
    void disable();

private:
    PlanProfileState state = PlanProfileState::off;
    MotorDriver& motorDriver_;
};