#include "app/logger.hpp"
#include "device/device_instance.hpp"
#include "common/debug.hpp"

bool Logger::add(const char* name, const float* value) {
    if (fieldCount_ >= MAX_FIELDS) {
        return false;
    }

    if (name == nullptr || value == nullptr) {
        return false;
    }

    fields_[fieldCount_] = {
        .name = name,
        .value = value,
    };
    ++fieldCount_;
    maxSamples_ = MAX_BUFFER_SIZE / fieldCount_;

    return true;
}

void Logger::start(void) {
    sampleCount_ = 0;
    state_ = LoggerState::Recording;
}

void Logger::stop(void) {
    state_ = LoggerState::Stopped;
}

void Logger::clear(void) {
    state_ = LoggerState::Stopped;
    sampleCount_ = 0;
}

void Logger::sample(void) {
    if (state_ != LoggerState::Recording) {
        return;
    }

    if (sampleCount_ >= maxSamples_) {
        state_ = LoggerState::Stopped;
        return;
    }

    for (uint32_t i = 0; i < fieldCount_; ++i) {
        buffer_[sampleCount_ * fieldCount_ + i] = *fields_[i].value;
    }
    ++sampleCount_;
}

void Logger::dump() {
    printf("BIN_START\r\n");
    printf("SIZE:%u\r\n", static_cast<uint32_t>(dataSize()));
    for (uint32_t i = 0; i < fieldCount_; ++i) {
        if (i > 0) {
            printf(",");
        }
        printf("%s", fields_[i].name);
    }
    printf("\r\n");

    uart_write(
        reinterpret_cast<const uint8_t*>(buffer_),
        dataSize()
    );

    printf("BIN_END\r\n");
}

uint32_t Logger::dataSize(void) const {
    return sampleCount_ * fieldCount_ * sizeof(float);
}

uint32_t Logger::sampleCount(void) const {
    return  sampleCount_;
}

uint32_t Logger::fieldCount(void) const {
    return  fieldCount_;
}

bool Logger::isRecording(void) const {
    return isRecording_;
}

bool Logger::isFull(void) const {
    return isFull_;
}

void OldLogger::start() {
    recording_ = true;
    writeIndex_ = 0;
    timestamp_ms_ = 0;
    full_ = false;
}

void OldLogger::stop() {
    recording_ = false;
}

void OldLogger::update() {
    timestamp_ms_ += 1;
    if (!recording_ || full_) {
        return;
    }

    LogRecord record;
    record.time = static_cast<float>(timestamp_ms_) / 1000.f;
    record.battery = adcValue.batt.filtered();
    record.dutyL = motorLeft.getDuty();
    record.dutyR = motorRight.getDuty();
    record.encoderL = encoderLeft.velocity();
    record.encoderR = encoderRight.velocity();
    record.gyroZ = imu.gyroZ();

    buffer_[writeIndex_] = record;
    writeIndex_++;

    if (writeIndex_ >= MAX_ROWS) {
        full_ = true;
    }
}

void OldLogger::dump() {
    printf("BIN_START\r\n");
    printf("SIZE:%d\r\n", sizeof(LogRecord) * writeIndex_);
    printf("time,battery,dutyL,dutyR,encoderL,encoderR,gyroZ\r\n");

    uart_write(reinterpret_cast<const uint8_t*>(&logger.buffer_), sizeof(LogRecord) * writeIndex_);
    LOG("BIN_END\r\n");
}