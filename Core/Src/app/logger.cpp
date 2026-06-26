#include "app/logger.hpp"
#include "device/device_instance.hpp"
#include "common/debug.hpp"

void Logger::start() {
    recording_ = true;
    writeIndex_ = 0;
    timestamp_ms_ = 0;
    full_ = false;
}

void Logger::stop() {
    recording_ = false;
}

void Logger::update() {
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

void Logger::dump() {
    printf("BIN_START\r\n");
    printf("SIZE:%d\r\n", sizeof(LogRecord) * writeIndex_);
    printf("time,battery,dutyL,dutyR,encoderL,encoderR,gyroZ\r\n");

    uart_write(reinterpret_cast<const uint8_t*>(&logger.buffer_), sizeof(LogRecord) * writeIndex_);
    LOG("BIN_END\r\n");
}