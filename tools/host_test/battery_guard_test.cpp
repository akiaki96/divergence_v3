// 回転FF試験の低電圧ガード（Core/Src/test/motor_id.cpp の omega_test_battery_ok）のホスト検証。
// run.sh が実ソースから閾値定数と関数を切り出し（battery_guard.inc），battery / ledBar16 / HAL_Delay / LOG を
// スタブにして次を確認する：
//  - 閾値以上（境界を含む）ではtrueを返し，LEDにもディレイにも触れない
//  - 閾値未満ではfalseを返し，LEDを左右交互（0xFF00 / 0x00FF）に3秒点滅して最後に消灯する
#include <cstdio>
#include <cstdint>
#include <vector>

struct Battery { float v; float voltage() const { return v; } };
static Battery battery{8.3f};

struct LedBar { std::vector<unsigned> log; void set(unsigned m) { log.push_back(m); } };
static LedBar ledBar16;

static uint32_t g_delay_total = 0;
static void HAL_Delay(uint32_t ms) { g_delay_total += ms; }

#define LOG(...) std::printf(__VA_ARGS__)

#include "battery_guard.inc"

static int g_bad = 0;
static void check(const char* name, bool ok) {
    if (!ok) { ++g_bad; std::printf("  NG  %s\n", name); }
}

static void reset() { ledBar16.log.clear(); g_delay_total = 0; }

int main() {
    // 十分な電圧
    for (float v : {8.4f, 8.2f, 8.0f + 1e-3f, kOmegaTestMinBatteryV}) {
        battery.v = v; reset();
        check("ok voltage returns true", omega_test_battery_ok());
        check("ok voltage: no LED activity", ledBar16.log.empty());
        check("ok voltage: no delay", g_delay_total == 0);
    }
    // 不足
    for (float v : {7.99f, 7.7f, 7.0f, 0.0f}) {
        battery.v = v; reset();
        check("low voltage returns false", !omega_test_battery_ok());
        check("low voltage: alternating halves", ledBar16.log.size() == 13 && ledBar16.log[0] == 0xFF00 && ledBar16.log[1] == 0x00FF
              && ledBar16.log[10] == 0xFF00 && ledBar16.log[11] == 0x00FF);
        check("low voltage: LED off at the end", ledBar16.log.back() == 0x0000);
        check("low voltage: about 3 s of blinking", g_delay_total == 3000);
    }
    std::printf("battery guard test: %s（閾値 %.2f V）\n", g_bad ? "FAILED" : "ALL OK", kOmegaTestMinBatteryV);
    return g_bad ? 1 : 0;
}
