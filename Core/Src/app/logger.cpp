#include "app/logger.hpp"
#include "device/device_instance.hpp"
#include "common/debug.hpp"
#include "config/mouse_config.hpp"

namespace {
// 1回の sample()（制御の1tick）の長さ [ms]
constexpr uint32_t TICK_MS = static_cast<uint32_t>(config::control::DT_S * 1000.f + 0.5f);
static_assert(TICK_MS >= 1, "the logger assumes a control tick of at least 1 ms");
}

void Logger::initLoggedVal(void) {
    fieldCount_ = 0;
    maxSamples_ = MAX_BUFFER_SIZE;
    decimation_ = 1;
    durationMs_ = 0;
    isRecording_ = false;
    isFull_ = false;
    add("Global_time", &globalTime);
    state_ = LoggerState::Idle;
    dirName = ".";
    fileName = nullptr;
    includeTimestamp = true;
}

void Logger::setDecimation(uint32_t every_n_ticks) {
    decimation_ = (every_n_ticks == 0) ? 1 : every_n_ticks;
    durationMs_ = 0;
}

void Logger::setDuration(uint32_t duration_ms) {
    durationMs_ = duration_ms;
}

uint32_t Logger::recordableMs(void) const {
    return maxSamples_ * decimation_ * TICK_MS;
}

void Logger::setDirName(const char* name) {
    dirName = name;
}

void Logger::setFileName(const char* name) {
    fileName = name;
}

void Logger::setIncludeTimestamp(bool enable) {
    includeTimestamp = enable;
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
    if (durationMs_ > 0) {
        const uint32_t ticks = (durationMs_ + TICK_MS - 1) / TICK_MS;
        decimation_ = (ticks + maxSamples_ - 1) / maxSamples_;   // 切り上げ
        if (decimation_ == 0) decimation_ = 1;
    }
    sampleCount_ = 0;
    tickCount_ = 0;
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

    // 間引き：decimation_ tickに1回だけ記録する（開始直後のtickは記録する）
    if (tickCount_++ % decimation_ != 0) {
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

    const char* names[MAX_FIELDS];
    for (uint32_t i = 0; i < fieldCount_; ++i) {
        names[i] = fields_[i].name;
    }
    bin_table::begin(dirName, fileName, includeTimestamp, dataSize(), names, fieldCount_);
    uart_write(
        reinterpret_cast<const uint8_t*>(buffer_),
        dataSize()
    );
    bin_table::end();
}

namespace bin_table {

void begin(const char* dir, const char* file, bool timestamp, uint32_t size_bytes,
           const char* const* names, uint32_t count) {
    printf("BIN_START\r\n");
    printf("%s\r\n", dir);
    printf("%s\r\n", file != nullptr ? file : "");
    printf("TIMESTAMP:%d\r\n", timestamp ? 1 : 0);
    printf("SIZE:%lu\r\n", static_cast<unsigned long>(size_bytes));
    for (uint32_t i = 0; i < count; ++i) {
        if (i > 0) {
            printf(",");
        }
        printf("%s", names[i]);
    }
    printf("\r\n");
}

void end() {
    printf("BIN_END\r\n");
}

} // namespace bin_table

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