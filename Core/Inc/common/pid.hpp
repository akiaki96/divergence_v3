#pragma once

#include "common/etc.hpp"
class PIDController {
public:

    void reset();
    float update(float target, float current);
    float update(float target, float current, float limit, bool& saturated);

    void setGains(float kp, float ki, float kd, float (*ff)(float)) {
        this->kp = kp;
        this->ki = ki;
        this->kd = kd;
        this->ff = ff;
    }

    float kp;
    float ki;
    float kd;
    
    // ff: float -> float
    float (*ff)(float);
    

private:
    float integral_;
    float previous_error_;
};
