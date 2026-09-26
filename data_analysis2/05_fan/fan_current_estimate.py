#!/usr/bin/env python3
"""吸引ファンのON/OFF時のLiPo電圧降下から，ファン電流を推定する。

入力: ファーム(Fan メニュー "fan vsag x.xx")が出力するCSV
      tools/log/fan_vsag_m2/fan_010.csv 等（列: Global_time, battery[V], fan_duty）
原理: ファンOFF→ON で バッテリ電圧が ΔV 下がる。ΔV = I_fan * R_eff なので I_fan = ΔV / R_eff。
      R_eff はLiPo内部抵抗＋配線/コネクタ/スイッチ等，ADC測定点より上流の直列抵抗の合計。

使い方:
  # 推定（R_eff が既知）
  python fan_current_estimate.py tools/log/fan_vsag_m2/fan_*.csv --r-eff 0.12
  # R_eff の校正（同じ条件でファン電流を電流計で実測した値をCSVと同じ順で渡す）
  python fan_current_estimate.py fan_020.csv fan_040.csv --i-meas 0.4 0.9
  # 校正なしで ΔV だけ見る（モーター間の相対比較にはこれで足りる）
  python fan_current_estimate.py fan_020.csv
詳細と注意点は fan_current_estimation.md を参照。
"""
import argparse
import sys

import numpy as np

# 解析窓 [s]。ON/OFF 遷移は fan_duty の変化点で検出する（1kHzサンプル）
STEADY_WIN = 0.5      # 定常値: ON終端/OFF開始直前の この長さ を平均
STEADY_GUARD = 0.01   # 遷移の直前 この長さ は除外
EDGE_SKIP = 0.005     # OFF直後 この長さ はPWM遷移・ログ1tickのずれを避けるため除外
EDGE_WIN = 0.020      # OFFエッジ前後で電圧ジャンプを見る窓幅
INRUSH_WIN = 0.5      # ON直後 この区間の最小電圧を突入(ピーク)電流の指標にする
INRUSH_SMOOTH = 5     # 突入ピークの最小値を取る前の移動平均 [サンプル]
ADC_LSB_V = 3.3 / 4096.0 * (33000.0 + 20000.0) / 20000.0   # battery.filtered_ の1LSB相当 [V]


def load(path):
    d = np.genfromtxt(path, delimiter=",", names=True)
    return d["Global_time"], d["battery"], d["fan_duty"]


def _win(t, v, t0, t1):
    m = (t >= t0) & (t < t1)
    return v[m]


def analyze(path):
    t, v, duty = load(path)
    on = np.flatnonzero(duty > 0)
    if on.size == 0:
        raise ValueError(f"{path}: fan_duty がずっと0（ファンが回っていない）")
    i_on = on[0]
    after = np.flatnonzero(duty[i_on:] == 0)
    if after.size == 0:
        raise ValueError(f"{path}: OFFへの遷移が記録されていない")
    i_off = i_on + after[0]
    t_on, t_off = t[i_on], t[i_off]

    pre = _win(t, v, t_on - STEADY_WIN, t_on - STEADY_GUARD)
    ss = _win(t, v, t_off - STEADY_WIN, t_off - STEADY_GUARD)
    e_pre = _win(t, v, t_off - EDGE_SKIP - EDGE_WIN, t_off - STEADY_GUARD / 10)
    e_post = _win(t, v, t_off + EDGE_SKIP, t_off + EDGE_SKIP + EDGE_WIN)
    tail = _win(t, v, t[-1] - STEADY_WIN, t[-1] + 1)
    if min(pre.size, ss.size, e_pre.size, e_post.size) < 3:
        raise ValueError(f"{path}: 解析窓に十分なサンプルが無い（ON/OFF区間が短い？）")

    on_seg = (t >= t_on) & (t < t_on + INRUSH_WIN)
    k = np.ones(INRUSH_SMOOTH) / INRUSH_SMOOTH
    v_min_on = np.convolve(v[on_seg], k, mode="valid").min()

    def sem(x):
        return x.std(ddof=1) / np.sqrt(x.size)

    r = {
        "path": path,
        "duty": float(duty[i_on]),
        "t_on": t_on, "t_off": t_off,
        "v_pre": pre.mean(),
        "v_on": ss.mean(),
        "dv_ss": pre.mean() - ss.mean(),                  # 定常電圧降下（DC: オーム＋分極）
        "dv_ss_se": np.hypot(sem(pre), sem(ss)),
        "dv_off": e_post.mean() - e_pre.mean(),           # OFF直後の電圧ジャンプ（ほぼオーム分）
        "dv_off_se": np.hypot(sem(e_pre), sem(e_post)),
        "dv_peak": pre.mean() - v_min_on,                 # 突入込みの最大降下（参考値。分極の遅れがあり電流へは換算しない）
        "ripple_on": ss.std(ddof=1),                      # ON中のサンプル間ばらつき（リプル＋ノイズ）
        "ripple_off": pre.std(ddof=1),                    # OFF中のばらつき（ADCノイズのみ）
        "drift": tail.mean() - pre.mean(),                # ON前と終了時の休止電圧の差（放電・回復）
    }
    r["t"], r["v"], r["duty_t"] = t, v, duty
    return r


def fit_r(dv, i_meas):
    """dv = R*i の原点を通る最小二乗。R と 残差RMS を返す"""
    dv, i = np.asarray(dv), np.asarray(i_meas)
    r = float(dv @ i / (i @ i))
    rms = float(np.sqrt(np.mean((dv - r * i) ** 2)))
    return r, rms


def plot(results, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(len(results), 1, figsize=(9, 3 * len(results)), squeeze=False)
    for ax, r in zip(axes[:, 0], results):
        ax.plot(r["t"] - r["t_on"], r["v"], lw=0.6, label="battery")
        ax.axhline(r["v_pre"], color="g", ls="--", lw=0.8, label="V_pre")
        ax.axhline(r["v_on"], color="r", ls="--", lw=0.8, label="V_on (steady)")
        ax.axvline(0, color="k", lw=0.5)
        ax.axvline(r["t_off"] - r["t_on"], color="k", lw=0.5)
        ax.set_title(f"{r['path']}  duty={r['duty']:.2f}  dV_ss={r['dv_ss'] * 1e3:.1f} mV")
        ax.set_xlabel("t - t_on [s]")
        ax.set_ylabel("V")
        ax.grid(True)
        ax.legend(loc="lower right")
    fig.tight_layout()
    fig.savefig(out, dpi=120)
    print(f"plot: {out}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", nargs="+")
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--r-eff", type=float, help="実効内部抵抗 R_eff [Ω]。指定するとファン電流[A]を出力する")
    g.add_argument("--i-meas", type=float, nargs="+",
                   help="電流計で実測したファン定常電流[A]（CSVと同じ順）。R_effを校正する")
    ap.add_argument("--method", choices=["ss", "off"], default="ss",
                    help="電流換算に使う電圧降下。ss=定常(DC)降下 / off=OFF直後のオーム性ジャンプ（既定 ss）")
    ap.add_argument("--plot", metavar="PNG", help="各試行の電圧波形を保存する")
    a = ap.parse_args()

    if a.i_meas is not None and len(a.i_meas) != len(a.csv):
        sys.exit(f"--i-meas の個数({len(a.i_meas)})がCSVの個数({len(a.csv)})と一致しない")

    res = [analyze(p) for p in a.csv]
    key = "dv_ss" if a.method == "ss" else "dv_off"

    print(f"ADC 1LSB = {ADC_LSB_V * 1e3:.2f} mV（ΔV がこれの数倍未満なら分解能不足）")
    print(f"{'file':<28}{'duty':>5}{'V_pre':>8}{'V_on':>8}{'dV_ss[mV]':>11}{'dV_off[mV]':>12}"
          f"{'dV_peak[mV]':>13}{'rip_on/off[mV]':>16}{'drift[mV]':>11}")
    for r in res:
        name = r["path"].split("/")[-1]
        print(f"{name:<28}{r['duty']:>5.2f}{r['v_pre']:>8.3f}{r['v_on']:>8.3f}"
              f"{r['dv_ss'] * 1e3:>7.1f}±{r['dv_ss_se'] * 1e3:<3.1f}"
              f"{r['dv_off'] * 1e3:>8.1f}±{r['dv_off_se'] * 1e3:<3.1f}"
              f"{r['dv_peak'] * 1e3:>13.1f}"
              f"{r['ripple_on'] * 1e3:>9.1f}/{r['ripple_off'] * 1e3:<6.1f}"
              f"{r['drift'] * 1e3:>11.1f}")

    dv = [r[key] for r in res]
    if a.i_meas is not None:
        r_eff, rms = fit_r(dv, a.i_meas)
        print(f"\n校正結果 ({key}): R_eff = {r_eff * 1e3:.1f} mΩ  （残差RMS {rms * 1e3:.1f} mV, N={len(res)}）")
        if len(res) == 1:
            print("  ※ 1点のみの校正。複数duty(電流レベル)で取ると直線性と誤差が確認できる")
        print(f"  推定に使うときは --r-eff {r_eff:.4f} --method {a.method}（校正時と同じ method を使うこと）")
    elif a.r_eff is not None:
        print(f"\nファン電流推定 (R_eff = {a.r_eff * 1e3:.1f} mΩ, method = {key}):")
        for r, d in zip(res, dv):
            se = (r["dv_ss_se"] if key == "dv_ss" else r["dv_off_se"]) / a.r_eff
            print(f"  duty {r['duty']:.2f}: I_fan = {d / a.r_eff:.2f} ± {se:.2f} A")
        print("  ± は統計誤差のみ。R_eff の不確かさ（校正誤差・SOC/温度依存）は別途乗る")
    else:
        print("\n（R_eff 未指定: ΔV のみ。モーター間の比較は同一電池・同SOCなら ΔV 比 = 電流比）")

    if a.plot:
        plot(res, a.plot)


if __name__ == "__main__":
    main()
