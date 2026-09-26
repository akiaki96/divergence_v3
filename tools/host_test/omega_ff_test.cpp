// 回転角速度制御の2自由度FF・Tiスケジュールのホスト検証（実機不要）。
// Core/Src/device/motorDriver.cpp から切り出した補間・FF関数（run.shが ff_funcs.inc を生成）と
// Core/Src/common/pid.cpp を実際にコンパイルして使う。config::pid_omega の値を変えても壊れないよう，
// 期待値は表の値から作る。
//  1. 単体テスト：補間点での一致，中点，符号の連続性，dω_ref/dtのクランプ
//     F6：バッテリ換算係数 V_REF/V の値・電圧クランプ・電圧基準の出力上限
//  2. 閉ループ（簡易プラント族，不感帯付き一次遅れ）でFF ON/OFFを比較（MotorDriver::update()のomega分岐と同じ手順）
//  3. F6：プラントの感度が電圧に比例する（実機の u_ss×V≒一定）として，電圧を振ったときの補償ON/OFFを比較
//     ※ 簡易モデルの構造の見積り。実機の非線形は含まない（data_analysis2/04_rot_omega_control/rot_gain_scheduling_plan.md §17〜§18）
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
        std::snprintf(nm, sizeof nm, "accel_ff(+%g,+1)", w); check(nm, omega_accel_ff(w, 1.f), FF_ACC_POS[i], 1e-9f);
        if (w > 0.f) {   // w=0の-w は -0.0f となり符号判定(>=0)で+側の表が選ばれるため，ゼロは除く
            std::snprintf(nm, sizeof nm, "accel_ff(-%g,-1)", w); check(nm, omega_accel_ff(-w, -1.f), -FF_ACC_NEG[i], 1e-9f);
        }
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
    check("accel_ff clamp(+,+)", omega_accel_ff(wend, 1e9f), FF_ACC_POS[TI_TABLE_SIZE - 1] * OMEGA_ACCEL_FF_MAX, 1e-5f);
    check("accel_ff clamp(+,-)", omega_accel_ff(wend, -1e9f), -FF_ACC_POS[TI_TABLE_SIZE - 1] * OMEGA_ACCEL_FF_MAX, 1e-5f);
    // F6：バッテリ換算係数
    check("batt_scale(V_REF)=1", omega_batt_scale(BATT_V_REF), 1.f, 1e-6f);
    check("batt_scale(8.4V)", omega_batt_scale(8.4f), BATT_V_REF / 8.4f, 1e-6f);
    check("batt_scale(7.0V)", omega_batt_scale(7.0f), BATT_V_REF / 7.0f, 1e-6f);
    check("batt_scale monotone", (omega_batt_scale(7.0f) > omega_batt_scale(8.0f)) ? 1.f : 0.f, 1.f, 0.f);
    check("batt_scale clamps low (0V)", omega_batt_scale(0.f), BATT_V_REF / BATT_V_MIN, 1e-6f);
    check("batt_scale clamps high (20V)", omega_batt_scale(20.f), BATT_V_REF / BATT_V_MAX, 1e-6f);
    check("batt_scale range sane", (omega_batt_scale(0.f) < 1.5f && omega_batt_scale(20.f) > 0.8f) ? 1.f : 0.f, 1.f, 0.f);
    // 電圧基準の上限：実dutyの上限 = LIMIT_V / V は，基準電圧duty空間の上限 LIMIT_V/V_REF に換算係数を掛けたもの
    for (float v : {7.0f, 7.4f, 7.9f, 8.4f})
        check("limit(V) = LIMIT_V/V", DUTY_DIFF_LIMIT_V / BATT_V_REF * omega_batt_scale(v), DUTY_DIFF_LIMIT_V / v, 1e-6f);
    check("F6 limit not below legacy at V_REF", (DUTY_DIFF_LIMIT_V / BATT_V_REF >= DUTY_DIFF_LIMIT) ? 1.f : 0.f, 1.f, 0.f);
    // -430のFF合計（静的＋2500dps/s^2の加速度FF）が新しい上限に収まる（従来の0.28では超えていた）
    float ffneg = FF_U_NEG[TI_TABLE_SIZE - 1] + FF_ACC_NEG[TI_TABLE_SIZE - 1] * OMEGA_ACCEL_MAX;
    check("FF sum(-430) under F6 limit", (ffneg < DUTY_DIFF_LIMIT_V / BATT_V_REF) ? 1.f : 0.f, 1.f, 0.f);
    // 速い輪のdutyが MAX_DUTY(0.95) に収まる（基本duty約0.07＋diff/2。V_MINでも）
    check("fast wheel duty under MAX_DUTY", (0.2f + DUTY_DIFF_LIMIT_V / BATT_V_MIN / 2.f < 0.95f) ? 1.f : 0.f, 1.f, 0.f);
    check("accel_ff clamp(-,-)", omega_accel_ff(-wend, -1e9f), -FF_ACC_NEG[TI_TABLE_SIZE - 1] * OMEGA_ACCEL_FF_MAX, 1e-5f);
}

struct Plant { float K, T, u0; };
struct Res { float os, r90, lag; };

// accel: 指令のレート制限 [dps/s]，sS/sA: 静的/加速度FFの倍率
// V/comp: バッテリ電圧[V]とF6（バッテリ補償）。プラント（K, u0）は基準電圧V_REFでのduty空間で定義し，
// 実機の性質（必要duty×V≒一定）に合わせて，電圧Vでは実dutyに V/V_REF を掛けたものがプラントに入る。
// comp=trueなら制御は基準電圧空間で行い出力に V_REF/V を掛ける（＝プラントにはそのままの値が入る）。上限も電圧基準
static Res run(const Plant& p, float sign, float tgt_abs, float accel, bool ff_on, float sS = 1.f, float sA = 1.f,
               float V = config::pid_omega::BATT_V_REF, bool comp = false, float* sat_ms = nullptr) {
    const float dt = config::control::DT_S;
    const int N = 1200;
    PIDController pid;
    pid.reset();
    float target = sign * tgt_abs;
    float w = 0.f, wprev = 0.f, ref = 0.f;
    std::vector<float> wl(N), rl(N);
    int nsat = 0;
    const float scale = comp ? omega_batt_scale(V) : 1.f;
    const float limit = comp ? config::pid_omega::DUTY_DIFF_LIMIT_V / config::pid_omega::BATT_V_REF : config::pid_omega::DUTY_DIFF_LIMIT;
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
        float u = pid.update(ref, wprev, limit, sat);
        if (sat) ++nsat;
        float u_plant = u * scale * V / config::pid_omega::BATT_V_REF;   // 実dutyを基準電圧duty空間へ（プラントの感度は電圧比例）
        float drive = (u_plant >= 0.f ? 1.f : -1.f) * p.K * std::max(std::fabs(u_plant) - p.u0, 0.f);
        w += dt * (drive - w) / p.T;
        wprev = w;
        wl[k] = w * sign;
        rl[k] = ref * sign;
    }
    Res r{};
    if (sat_ms) *sat_ms = (float)nsat;
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

    // ---- F6：電圧を振ったときの補償ON/OFF（プラントの感度は電圧に比例。K, T, u_ssは基準電圧V_REF=7.9Vでの値）----
    std::printf("=== F6：バッテリ電圧を振った比較（ランプ2500dps/s^2, FF ON）。u_ssはV_REFでの値。OS%% / 90%%到達ms / 飽和ms ===\n");
    const float Vs[] = {7.4f, 7.9f, 8.4f};
    struct Case { const char* name; float sign; Plant p; };
    Case cases[] = {
        {"+430  K4500 T0.15 u_ss0.200", +1.f, {4500.f, 0.15f, 0.200f - 430.f / 4500.f}},
        {"-430  K2500 T0.10 u_ss0.245", -1.f, {2500.f, 0.10f, 0.245f - 430.f / 2500.f}},
        {"-430  K1700 T0.075 u_ss0.272", -1.f, {1700.f, 0.075f, 0.272f - 430.f / 1700.f}},
    };
    for (auto& c : cases) {
        std::printf("%-30s|", c.name);
        for (float V : Vs) std::printf(" V=%.1f OFF            ON             |", V);
        std::printf("\n%-30s|", "");
        float r90_off[3], r90_on[3], os_off[3], os_on[3];
        for (int i = 0; i < 3; ++i) {
            float so = 0.f, sn = 0.f;
            Res a = run(c.p, c.sign, 430.f, 2500.f, true, 1.f, 1.f, Vs[i], false, &so);
            Res b = run(c.p, c.sign, 430.f, 2500.f, true, 1.f, 1.f, Vs[i], true, &sn);
            r90_off[i] = a.r90; r90_on[i] = b.r90; os_off[i] = a.os; os_on[i] = b.os;
            std::printf("       %4.1f/%3.0f/%3.0f  %4.1f/%3.0f/%3.0f |", a.os, a.r90, so, b.os, b.r90, sn);
        }
        std::printf("\n");
        // 電圧による変動幅（max-min）：補償ONの方が小さいこと（＝電圧に依らない応答）
        auto spread = [](const float* v) { return *std::max_element(v, v + 3) - *std::min_element(v, v + 3); };
        std::printf("%-30s| 電圧による変動幅  OS: OFF %.1f -> ON %.1f pt,  90%%到達: OFF %.0f -> ON %.0f ms\n\n", "",
                    spread(os_off), spread(os_on), spread(r90_off), spread(r90_on));
        if (spread(r90_on) > spread(r90_off) + 1.f) { ++g_bad; std::printf("  NG  F6 should not widen 90%% rise spread across voltages (%s)\n", c.name); }
        if (spread(os_on) > spread(os_off) + 0.5f)  { ++g_bad; std::printf("  NG  F6 should not widen OS spread across voltages (%s)\n", c.name); }
    }
    return g_bad ? 1 : 0;
}
