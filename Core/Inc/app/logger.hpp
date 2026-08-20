#pragma once

#include <cstdint>

enum class LoggerState {
    Idle,
    Recording,
    Stopped,
};

class Logger {
public:
    struct Filed {
        const char* name;
        const float* value;
    };
    static constexpr uint32_t MAX_FIELDS = 16;
    static constexpr uint32_t MAX_BUFFER_SIZE = 8 * 3000;

    bool add(const char* name, const float* value);

    void start(void);
    void stop(void);
    void clear(void);

    void sample(void);

    void dump(void);
    uint32_t dataSize(void) const;
    uint32_t sampleCount(void) const;
    uint32_t fieldCount(void) const;

    bool isRecording(void) const;
    bool isFull(void) const;

private:
    Filed fields_[MAX_FIELDS];
    uint32_t fieldCount_ = 0;
    float buffer_[MAX_BUFFER_SIZE];
    uint32_t sampleCount_ = 0;
    uint32_t maxSamples_ = MAX_BUFFER_SIZE;

    bool isRecording_ = false;
    bool isFull_ = false;

    LoggerState state_ = LoggerState::Idle;
};

struct LogRecord {
    float time;

    float battery;

    float dutyL;
    float dutyR;

    float encoderL;
    float encoderR;

    float gyroZ;
};

class OldLogger {
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