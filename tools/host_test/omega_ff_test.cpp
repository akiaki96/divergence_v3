// 回転角速度制御の2自由度FF・Tiスケジュールのホスト検証（実機不要）。
// Core/Src/device/motorDriver.cpp から切り出した補間・FF関数（run.shが ff_funcs.inc を生成）と
// Core/Src/common/pid.cpp を実際にコンパイルして使う。config::pid_omega の値を変えても壊れないよう，
// 期待値は表の値から作る。
//  1. 単体テスト：補間点での一致，中点，符号の連続性，dω_ref/dtのクランプ
//  2. 閉ループ（簡易プラント族，不感帯付き一次遅れ）でFF ON/OFFを比較（MotorDriver::update()のomega分岐と同じ手順）
//     ※ 簡易モデルの構造の見積り。実機の非線形は含まない（data_analysis2/rot_gain_scheduling_plan.md §17〜§18）
#include <cstdio>
#include <cmath>
#include <vector>
#include <algorithm>
#include "common/pid.hpp"
#include "config/mouse_config.hpp"
#include "ff_funcs.inc"

static float zero_ff_fn(float) { return 0.f; }

static int g_bad = 0;
static void check(const char* name, float got, float expect, float tol) {
    bool ok = std::fabs(got - expect) <= tol;
    if (!ok) ++g_bad;
    if (!ok) std::printf("  NG  %-36s got %10.6f expect %10.6f\n", name, got, expect);
}

static void unit_tests() {
    using namespace config::pid_omega;
    char nm[64];
    for (int i = 0; i < TI_TABLE_SIZE; ++i) {
        float w = TI_OMEGA_BP[i];
        std::snprintf(nm, sizeof nm, "static_ff(+%g)", w);  check(nm, omega_static_ff(w), FF_U_POS[i], 1e-6f);
        std::snprintf(nm, sizeof nm, "static_ff(-%g)", w);  check(nm, omega_static_ff(-w), -FF_U_NEG[i], 1e-6f);
        std::snprintf(nm, sizeof nm, "accel_ff(%g,+1)", w); check(nm, omega_accel_ff(w, 1.f), FF_ACC[i], 1e-9f);
        std::snprintf(nm, sizeof nm, "Ti(+%g)", w);         check(nm, omega_ti_schedule(w), TI_S_BP_POS[i], 1e-7f);
        std::snprintf(nm, sizeof nm, "Ti(-%g)", w);         check(nm, omega_ti_schedule(-w), TI_S_BP_NEG[i], 1e-7f);
    }
    for (int i = 1; i < TI_TABLE_SIZE; ++i) {
        float wm = 0.5f * (TI_OMEGA_BP[i - 1] + TI_OMEGA_BP[i]);
        std::snprintf(nm, sizeof nm, "static_ff mid(+%g)", wm);
        check(nm, omega_static_ff(wm), 0.5f * (FF_U_POS[i - 1] + FF_U_POS[i]), 1e-6f);
        std::snprintf(nm, sizeof nm, "static_ff mid(-%g)", wm);
        check(nm, omega_static_ff(-wm), -0.5f * (FF_U_NEG[i - 1] + FF_U_NEG[i]), 1e-6f);
    }
    float wend = TI_OMEGA_BP[TI_TABLE_SIZE - 1];
    check("static_ff saturates above table(+)", omega_static_ff(wend * 2.f), FF_U_POS[TI_TABLE_SIZE - 1], 1e-6f);
    check("static_ff saturates above table(-)", omega_static_ff(-wend * 2.f), -FF_U_NEG[TI_TABLE_SIZE - 1], 1e-6f);
    check("static_ff continuous at 0", omega_static_ff(1e-3f) - omega_static_ff(-1e-3f), 0.f, 1e-4f);
    check("accel_ff zero rate", omega_accel_ff(wend, 0.f), 0.f, 1e-9f);
    check("accel_ff odd in rate", omega_accel_ff(wend, 1000.f) + omega_accel_ff(wend, -1000.f), 0.f, 1e-9f);
    check("accel_ff clamp(+)", omega_accel_ff(wend, 1e9f), FF_ACC[TI_TABLE_SIZE - 1] * OMEGA_ACCEL_FF_MAX, 1e-5f);
    check("accel_ff clamp(-)", omega_accel_ff(wend, -1e9f), -FF_ACC[TI_TABLE_SIZE - 1] * OMEGA_ACCEL_FF_MAX, 1e-5f);
}

struct Plant { float K, T, u0; };
struct Res { float os, r90, lag; };

// accel: 指令のレート制限 [dps/s]，sS/sA: 静的/加速度FFの倍率
static Res run(const Plant& p, float sign, float tgt_abs, float accel, bool ff_on, float sS = 1.f, float sA = 1.f) {
    const float dt = config::control::DT_S;
    const int N = 1200;
    PIDController pid;
    pid.reset();
    float target = sign * tgt_abs;
    float w = 0.f, wprev = 0.f, ref = 0.f;
    std::vector<float> wl(N), rl(N);
    for (int k = 0; k < N; ++k) {
        float max_step = accel * dt;
        float d = target - ref;
        if (d > max_step) d = max_step;
        else if (d < -max_step) d = -max_step;
        ref += d;
        float ti = omega_ti_schedule(ref);
        pid.setGains(config::pid_omega::kp, config::pid_omega::kp / ti, config::pid_omega::kd, zero_ff_fn, ti);
        float ff = 0.f;
        if (ff_on) ff = sS * omega_static_ff(ref) + sA * omega_accel_ff(ref, d / dt);
        pid.setExternalFF(ff);
        bool sat = false;
        float u = pid.update(ref, wprev, config::pid_omega::DUTY_DIFF_LIMIT, sat);
        float drive = (u >= 0.f ? 1.f : -1.f) * p.K * std::max(std::fabs(u) - p.u0, 0.f);
        w += dt * (drive - w) / p.T;
        wprev = w;
        wl[k] = w * sign;
        rl[k] = ref * sign;
    }
    Res r{};
    float mx = *std::max_element(wl.begin(), wl.end());
    r.os = std::max(0.f, (mx - tgt_abs) / tgt_abs) * 100.f;
    r.r90 = -1.f;
    for (int k = 0; k < N; ++k) if (wl[k] >= 0.9f * tgt_abs) { r.r90 = (float)k; break; }
    float lag = 0.f;
    for (int k = 0; k < 240; ++k) lag = std::max(lag, rl[k] - wl[k]);
    r.lag = lag / tgt_abs * 100.f;
    return r;
}

struct Side { const char* name; float sign; float tgt; std::vector<float> Ks, Ts, uss; };

int main() {
    unit_tests();
    std::printf("単体テスト: %s\n\n", g_bad ? "FAILED" : "ALL OK");

    Side sides[] = {
        {"+430 (K 3000-6000, T 100-300ms, u_ss 0.176-0.222)", +1.f, 430.f, {3000, 4500, 6000}, {0.10f, 0.15f, 0.20f, 0.30f}, {0.176f, 0.200f, 0.222f}},
        {"-430 (K 1700-3000, T 75-150ms,  u_ss 0.220-0.272)", -1.f, 430.f, {1700, 2500, 3000}, {0.075f, 0.10f, 0.15f}, {0.220f, 0.245f, 0.272f}},
    };
    for (auto& sd : sides) {
        std::printf("=== %s : ランプ2500dps/s^2 ===\n", sd.name);
        std::printf("%-20s | %-14s | %-18s | %-8s | n\n", "", "OS% 平均/最悪", "90%到達ms 平均/最悪", "遅れ%");
        for (int ff = 0; ff <= 1; ++ff) {
            double os = 0, r90 = 0, lag = 0; float osw = 0, r90w = 0; int n = 0;
            for (float K : sd.Ks) for (float T : sd.Ts) for (float us : sd.uss) {
                Plant p{K, T, us - sd.tgt / K};
                if (p.u0 <= 0.f) continue;
                Res roff = run(p, sd.sign, sd.tgt, 2500.f, false);
                if (roff.r90 < 100.f || roff.r90 > 260.f) continue;   // 実機の立上り(100〜260ms)に合うプラントのみ
                Res r = ff ? run(p, sd.sign, sd.tgt, 2500.f, true) : roff;
                os += r.os; r90 += r.r90; lag += r.lag; osw = std::max(osw, r.os); r90w = std::max(r90w, r.r90); ++n;
            }
            if (n) std::printf("%-20s | %5.1f / %5.1f  | %6.0f / %6.0f     | %6.1f   | %d\n", ff ? "FF ON" : "FF OFF (PIのみ)", os / n, osw, r90 / n, r90w, lag / n, n);
        }
        std::printf("\n");
    }

    std::printf("=== +側：時定数T別のFF効果（OS%% / 90%%到達ms：OFF -> ON）と加速度FF倍率sA ===\n");
    float sAs[] = {0.4f, 0.7f, 1.0f, 1.3f, 1.6f, 2.0f};
    std::printf("%-10s| OFF        ", "T");
    for (float sA : sAs) std::printf("| sA=%.1f     ", sA);
    std::printf("\n");
    for (float T : sides[0].Ts) {
        double o0 = 0, r0 = 0; int n0 = 0;
        std::vector<double> o(6, 0.0), r(6, 0.0); std::vector<int> n(6, 0);
        for (float K : sides[0].Ks) for (float us : sides[0].uss) {
            Plant p{K, T, us - sides[0].tgt / K};
            if (p.u0 <= 0.f) continue;
            Res a0 = run(p, +1.f, 430.f, 2500.f, false);
            if (a0.r90 < 100.f || a0.r90 > 260.f) continue;
            o0 += a0.os; r0 += a0.r90; ++n0;
            for (int i = 0; i < 6; ++i) { Res a1 = run(p, +1.f, 430.f, 2500.f, true, 1.f, sAs[i]); o[i] += a1.os; r[i] += a1.r90; ++n[i]; }
        }
        if (!n0) continue;
        std::printf("T=%3.0fms  | %5.1f/%4.0f  ", T * 1000, o0 / n0, r0 / n0);
        for (int i = 0; i < 6; ++i) std::printf("| %5.1f/%4.0f ", o[i] / n[i], r[i] / n[i]);
        std::printf("\n");
    }
    return g_bad ? 1 : 0;
}
