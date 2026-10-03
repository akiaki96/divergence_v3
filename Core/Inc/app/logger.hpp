#pragma once

#include <cstdint>
#include <etl/delegate.h>

// PCへ表（float32の行の並び）を送るときの枠（tools/DATA_FORMAT.md の BIN_START 〜 BIN_END）。
// begin() でヘッダまで送り，呼び出し側が size_bytes ぶんの本体を uart_write() で送ってから end() を呼ぶ。
// Logger::dump() と探索のログ（app/search.cpp）で共有する
namespace bin_table {
void begin(const char* dir, const char* file, bool timestamp, uint32_t size_bytes,
           const char* const* names, uint32_t count);
void end();
}

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
    // 8列なら1500サンプル（1kHzで1.5s）。長い試験は setDuration() で間引いて収める（48KB）
    static constexpr uint32_t MAX_BUFFER_SIZE = 8 * 1500;

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
    // initLoggedVal()で1に戻る。setDuration()を取り消す
    void setDecimation(uint32_t every_n_ticks);
    // 少なくとも duration_ms [ms] 記録できるように，start() のときの列数から間引きを決める（毎tickで収まれば1）。
    // 列を足す前に呼んでよい。initLoggedVal()で取り消される。setDecimation()を取り消す
    void setDuration(uint32_t duration_ms);
    // 今の列数・間引きで記録できる長さ [ms]（start()の後に，setDuration()の結果を確かめるのに使う）
    uint32_t recordableMs(void) const;

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
    uint32_t durationMs_ = 0;   // 0なら decimation_ をそのまま使う
    uint32_t tickCount_ = 0;

    bool isRecording_ = false;
    bool isFull_ = false;

    LoggerState state_ = LoggerState::Idle;
};