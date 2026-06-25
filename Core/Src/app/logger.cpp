#include "app/logger.hpp"
#include "device/device_instance.hpp"
#include "common/debug.hpp"

void Logger::start() {
    recording_ = true;
    writeIndex_ = 0;
    full_ = false;
}

void Logger::stop() {
    recording_ = false;
}

void Logger::update() {
    if (!recording_ || full_) {
        return;
    }

    LogRecord record;
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
    printf("SIZE:%d\r\n", writeIndex_);
    printf("battery,dutyL,dutyR,encoderL,encoderR,gyroZ\r\n");

    for (uint16_t i = 0; i < writeIndex_; ++i) {
        const LogRecord& record = buffer_[i];
        // Here you would typically send the record to a serial port, file, or other output
        // For example:
        LOG("%f, %f, %f, %f, %f, %f\r\n", record.battery, record.dutyL, record.dutyR, record.encoderL, record.encoderR, record.gyroZ);
    }
}