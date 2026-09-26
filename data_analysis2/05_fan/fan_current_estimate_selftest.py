#!/usr/bin/env python3
"""fan_current_estimate.py の自己検証。既知のLiPoモデルで合成したログから電流・R_effを復元できるか確認する。

モデル: V = OCV - I*R0 - v_rc,  dv_rc/dt = (I*R1 - v_rc)/tau_rc （オーム＋1次遅れ分極）
ファン電流: I(t) = I_ss*(1 - exp(-t/tau_fan)) + 突入オーバーシュート。I_ss = 3.0*duty^1.5 [A]
ADC: 1LSB量子化 + ノイズ + PWMリプル（位相が巡回する想定でサンプル毎に位相が進む）
実行: python fan_current_estimate_selftest.py
"""
import os
import subprocess
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPT = os.path.join(HERE, "fan_current_estimate.py")
sys.path.insert(0, HERE)
import fan_current_estimate as fce  # noqa: E402

R0, R1, TAU_RC = 0.080, 0.030, 2.0
OCV0 = 8.10
PRE, ON, POST = 1.0, 3.0, 2.0


def i_ss(duty):
    return 3.0 * duty ** 1.5


def synth(duty, path, seed=0):
    rng = np.random.default_rng(seed)
    n = int((PRE + ON + POST) * 1000)
    t = 10.0 + np.arange(n) * 1e-3
    tt = t - t[0]
    on = (tt >= PRE) & (tt < PRE + ON)
    tau_on = np.clip(tt - PRE, 0, None)
    i = np.where(on, i_ss(duty) * (1 - np.exp(-tau_on / 0.3)) + 0.4 * i_ss(duty) * np.exp(-tau_on / 0.08), 0.0)
    v = np.empty(n)
    v_rc = 0.0
    for k in range(n):
        v_rc += (i[k] * R1 - v_rc) / TAU_RC * 1e-3
        v[k] = OCV0 - 2e-5 * tt[k] - i[k] * R0 - v_rc          # 微小な自然放電ドリフト
    ripple = 0.020 * (i / max(i.max(), 1e-9)) * np.sin(2 * np.pi * 0.192 * np.arange(n))
    v = v + ripple + rng.normal(0, 0.002, n)
    v = np.round(v / fce.ADC_LSB_V) * fce.ADC_LSB_V
    d = np.where(on, duty, 0.0)
    with open(path, "w") as f:
        f.write("Global_time,battery,fan_duty\n")
        for a, b, c in zip(t, v, d):
            f.write(f"{a:.6f},{b:.6f},{c:.3f}\n")
    return i_ss(duty), i[-1]


def main():
    tmp = os.path.join(os.environ.get("CLAUDE_JOB_DIR", tempfile.gettempdir()), "tmp", "fan_selftest")
    os.makedirs(tmp, exist_ok=True)
    duties = [0.25, 0.50, 0.75, 1.00]
    paths, truth = [], []
    for k, dty in enumerate(duties):
        p = os.path.join(tmp, f"fan_{int(dty * 100):03d}.csv")
        synth(dty, p, seed=k)
        paths.append(p)
        truth.append(i_ss(dty))

    res = [fce.analyze(p) for p in paths]
    r_ss_true = R0 + R1 * (1 - np.exp(-ON / TAU_RC))    # ON 3s 時点でのDC実効抵抗
    # 校正: 実電流(真値)から R_eff を求め，別duty(校正に使っていない点)の電流を推定する
    r_cal, rms = fce.fit_r([r["dv_ss"] for r in res[1::2]], truth[1::2])   # duty 0.5, 1.0 で校正
    print(f"R_ss 真値 {r_ss_true * 1e3:.1f} mΩ / 校正値 {r_cal * 1e3:.1f} mΩ (残差RMS {rms * 1e3:.1f} mV)")
    ok = True
    for r, i_true in zip(res, truth):
        i_est = r["dv_ss"] / r_cal
        err = (i_est - i_true) / i_true * 100
        print(f"duty {r['duty']:.2f}: I 真値 {i_true:.2f} A, 推定 {i_est:.2f} A ({err:+.1f} %)")
        ok &= abs(err) < 3.0
    r_off = fce.fit_r([r["dv_off"] for r in res], truth)[0]
    print(f"OFFジャンプからの R: {r_off * 1e3:.1f} mΩ（R0 真値 {R0 * 1e3:.1f} mΩ, 分極の一部が含まれ僅かに大きい）")
    ok &= abs(r_off - R0) < 0.006
    ok &= abs(r_cal - r_ss_true) < 0.004

    out = subprocess.run([sys.executable, SCRIPT, *paths, "--r-eff", f"{r_cal:.4f}"],
                         capture_output=True, text=True)
    ok &= out.returncode == 0
    print(out.stdout if out.returncode == 0 else out.stderr)
    print("SELFTEST", "PASS" if ok else "FAIL")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
