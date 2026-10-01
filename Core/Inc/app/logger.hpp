#pragma once

#include <cstdint>
#include <etl/delegate.h>

enum class LoggerState {
    Idle,
    Recording,
    Stopped,
};

class Logger {
public:
    using Getter = etl::delegate<float()>;

    struct Filed {
        const char* name;
        const float* value;
        Getter getter;
    };
    static constexpr uint32_t MAX_FIELDS = 16;
    static constexpr uint32_t MAX_BUFFER_SIZE = 8 * 3000;

    void initLoggedVal(void);
    void setDirName(const char* name);
    void setFileName(const char* name);
    void setIncludeTimestamp(bool enable);
    bool add(const char* name, const float* value);
    bool add(const char* name, Getter getter);
    // objectのconstメンバ関数Methodを値の取得に使う：logger.add<&Imu::gyroZ>("gyro_z", imu);
    template <auto Method, class T>
    bool add(const char* name, const T& object) {
        return add(name, Getter::create<T, Method>(object));
    }
    // n tickに1回だけ記録する（既定1＝毎tick）。長時間の試験でバッファ（MAX_BUFFER_SIZE）に収めるため。
    // initLoggedVal()で1に戻る
    void setDecimation(uint32_t every_n_ticks);

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

    // dirName/fileName together specify the save path on the PC side
    // (dirName/fileName.csv). fileName defaults to nullptr, in which case
    // the PC tool falls back to a generic name ("log").
    const char* dirName;
    const char* fileName;
    // When true, the PC tool appends a YYYYMMDD_HHMMSS timestamp to the
    // saved filename; when false, the file is saved under the given name
    // with no timestamp (and will overwrite a previous save of that name).
    bool includeTimestamp;

private:
    Filed fields_[MAX_FIELDS];
    uint32_t fieldCount_ = 0;
    float buffer_[MAX_BUFFER_SIZE];
    uint32_t sampleCount_ = 0;
    uint32_t maxSamples_ = MAX_BUFFER_SIZE;
    uint32_t decimation_ = 1;
    uint32_t tickCount_ = 0;

    bool isRecording_ = false;
    bool isFull_ = false;

    LoggerState state_ = LoggerState::Idle;
};