#include "app/logger.hpp"
#include "device/device_instance.hpp"
#include "common/debug.hpp"

void Logger::initLoggedVal(void) {
    fieldCount_ = 0;
    maxSamples_ = MAX_BUFFER_SIZE;
    isRecording_ = false;
    isFull_ = false;
    add("Global_time", &globalTime);
    state_ = LoggerState::Idle;
    dirName = ".";
}

void Logger::setDirName(const char* name) {
    dirName = name;
}

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
        .getter = {},
    };
    ++fieldCount_;
    maxSamples_ = MAX_BUFFER_SIZE / fieldCount_;
    isRecording_ = false;
    isFull_ = false;
    state_ = LoggerState::Idle;

    return true;
}

bool Logger::add(const char* name, Getter getter) {
    if (fieldCount_ >= MAX_FIELDS) {
        return false;
    }

    if (name == nullptr || !getter.is_valid()) {
        return false;
    }

    fields_[fieldCount_] = {
        .name = name,
        .value = nullptr,
        .getter = getter,
    };
    ++fieldCount_;
    maxSamples_ = MAX_BUFFER_SIZE / fieldCount_;
    isRecording_ = false;
    isFull_ = false;
    state_ = LoggerState::Idle;

    return true;
}

void Logger::start(void) {
    sampleCount_ = 0;
    isRecording_ = true;
    isFull_ = false;
    state_ = LoggerState::Recording;
}

void Logger::stop(void) {
    isRecording_ = false;
    state_ = LoggerState::Stopped;
}

void Logger::clear(void) {
    isRecording_ = false;
    isFull_ = false;
    state_ = LoggerState::Stopped;
    sampleCount_ = 0;
}

void Logger::sample(void) {
    if (state_ != LoggerState::Recording) {
        return;
    }

    if (sampleCount_ >= maxSamples_) {
        isFull_ = true;
        stop();
        return;
    }

    const uint32_t offset = sampleCount_ * fieldCount_;
    for (uint32_t i = 0; i < fieldCount_; ++i) {
        const Filed& field = fields_[i];
        
        if (field.value != nullptr) {
            buffer_[offset + i] = *field.value;
        } else {
            buffer_[offset + i] = field.getter();
        }
    }
    ++sampleCount_;
}

void Logger::dump() {
    // Only a Stopped buffer is safe to transmit: dumping while
    // Recording would race with the sampling ISR still writing into
    // buffer_/sampleCount_ mid-transfer, tearing the sent data.
    if (state_ != LoggerState::Stopped) {
        printf("no data to send!\r\n");
        return;
    }

    printf("BIN_START\r\n");
    printf("%s\r\n", dirName);
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