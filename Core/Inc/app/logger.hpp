#pragma once

#include <cstdint>

struct LogRecord {
    float time;

    float battery;

    float dutyL;
    float dutyR;

    float encoderL;
    float encoderR;

    float gyroZ;
};

class Logger {
public:
    void start();
    void stop();

    void update();
    void dump();

private:
    static constexpr uint16_t MAX_ROWS = 2500;

    LogRecord buffer_[MAX_ROWS];

    uint16_t writeIndex_ = 0;

    bool recording_ = false;
    bool full_ = false;
    uint16_t timestamp_ms_ = 0;
};