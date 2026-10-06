#!/usr/bin/env python3
"""横壁による向きの補正（common/wall_control）のゲインを，探索のトレースから決める。

モデル（直進中。y は中心線からの横ずれ [mm，左が正]，θ は壁に対する向き [rad，左が正]）：
    error = KS·(y + L·θ)       横のセンサーは車軸の約 L 先の壁を見るので，向きも L 倍で効く
    dθ/dt = −KP·error·π/180,  dy/dt = v·θ
走った距離 x で書くと y'' + G·L·y' + G·y = 0，G = KP·KS·(π/180)/v。KP = Kv·v なら G は速度によらない。
減衰 σ = G·L/2 [1/mm]，2% 収束 ≈ 4/σ。

  1. 同定：トレースの直進の区間（目標角度が一定・速度 > 150・wall_omega ≠ 0・上限で飽和していない）で，
     error = −wall_omega/KP を KS·(∫v·θ dt + L·θ) + 区間ごとの定数 + 区間ごとの傾き·x に当てはめて KS を出す。
     θ は current_angle − target_angle（区間の中の一定の向きのずれは傾きの項が吸収する）。L はグリッドで探す
  2. 設計：目標の収束距離（--settle）から G，Kv = G/(KS·π/180) を出す
  3. 確かめ：角度 PI（config::rotation の ANGLE_KP/KI）込みの閉ループを 1 ms で回し，速度ごとの収束距離を出す

トレースの KP：config の KP_PER_VELOCITY で走ったログは --kv，前の固定 KP（0.02）のログは --kp で指定する。

使い方:
    python3 tools/wall_control_design.py tools/log/search/*_trace*.csv --kp 0.02
    python3 tools/wall_control_design.py tools/log/search/700_trace_2.csv --kv 5.5e-4 --settle 270
    python3 tools/wall_control_design.py --ks 31 --L 100          # 同定せずに設計・確かめだけ
"""
import argparse
import csv
import math
import os
import re
import sys

import numpy as np

TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TOOLS_DIR)
CONFIG = os.path.join(ROOT, "Core", "Inc", "config", "mouse_config.hpp")
D2R = math.pi / 180.0


def read_config():
    """mouse_config.hpp から角度 PI と config::wall のゲイン（あれば）を読む"""
    text = open(CONFIG, encoding="utf-8").read()
    out = {}
    tread = float(re.search(r"TREAD_MM\s*=\s*([\d.]+)f", text).group(1))
    wheel_diff = math.pi / 180.0 * tread / 2.0
    out["angle_kp"] = float(re.search(r"ANGLE_KP\s*=\s*([\d.]+)f\s*/\s*WHEEL_DIFF_PER_DPS", text).group(1)) / wheel_diff
    out["angle_ki"] = float(re.search(r"ANGLE_KI\s*=\s*([\d.]+)f\s*/\s*WHEEL_DIFF_PER_DPS", text).group(1)) / wheel_diff
    for key in ["KP_PER_VELOCITY", "MAX_OMEGA_PER_VELOCITY"]:
        m = re.search(key + r"\s*=\s*([\d.eE+-]+)f", text)
        if m:
            out[key] = float(m.group(1))
    return out


def load_segments(path, gain_of_v, max_omega_of_v):
    """直進で補正が働いている連続区間ごとに (t, v, θ[rad], error) を返す"""
    rows = list(csv.DictReader(open(path)))
    if not rows or "wall_omega" not in rows[0]:
        return []
    col = lambda k: np.array([float(r[k]) for r in rows])
    t, v, ta, ca, w = col("Global_time"), col("target_velocity_x"), col("target_angle"), col("current_angle"), col("wall_omega")
    straight = np.abs(np.gradient(ta)) < 1e-3
    unsat = np.abs(w) < 0.98 * max_omega_of_v(np.maximum(v, 1.0))
    ok = (v > 150) & straight & (w != 0) & unsat
    segs, cur = [], []
    for i in np.where(ok)[0]:
        if cur and i != cur[-1] + 1:
            segs.append(cur)
            cur = []
        cur.append(i)
    if cur:
        segs.append(cur)
    out = []
    for s in segs:
        if len(s) < 6:
            continue
        s = np.array(s)
        out.append((t[s], v[s], np.deg2rad(ca[s] - ta[s]), -w[s] / gain_of_v(v[s])))
    return out


def fit_ks(segs, L):
    """error = KS·(∫vθ + Lθ) + a_k + b_k·x を最小二乗で解く。(KS, R², rms) を返す"""
    n_cols = 1 + 2 * len(segs)
    rows, rhs = [], []
    for k, (t, v, th, e) in enumerate(segs):
        dt = np.diff(t)
        x = np.concatenate([[0.0], np.cumsum(0.5 * (v[1:] + v[:-1]) * dt)])
        y = np.concatenate([[0.0], np.cumsum(0.5 * (v[1:] * th[1:] + v[:-1] * th[:-1]) * dt)])
        for i in range(len(t)):
            row = np.zeros(n_cols)
            row[0] = y[i] + L * th[i]
            row[1 + 2 * k] = 1.0
            row[2 + 2 * k] = x[i]
            rows.append(row)
            rhs.append(e[i])
    A, b = np.array(rows), np.array(rhs)
    sol = np.linalg.lstsq(A, b, rcond=None)[0]
    res = b - A @ sol
    return sol[0], 1.0 - np.sum(res ** 2) / np.sum((b - b.mean()) ** 2), math.sqrt(np.mean(res ** 2))


def simulate(kv, max_per_v, v, ks, L, cfg, y0=5.0, th0_deg=0.0, dist=900.0, dt=1e-3, tau_omega=0.01):
    """横ずれ y の閉ループ（補正 → 角度目標に積分 → 角度 PI → 角速度ループは1次遅れ）。
    (2% 収束距離 [mm], 最小 y [mm], ω のピーク [dps]) を返す"""
    y, th, offset, integ, omega = y0, th0_deg * D2R, 0.0, 0.0, 0.0
    n = int(dist / v / dt)
    ys = np.empty(n)
    w_peak = 0.0
    for i in range(n):
        e = ks * (y + L * th)
        w = float(np.clip(-kv * v * e, -max_per_v * v, max_per_v * v))
        w_peak = max(w_peak, abs(w))
        offset += w * dt
        err = offset - th / D2R
        integ += err * dt
        omega += (w + cfg["angle_kp"] * err + cfg["angle_ki"] * integ - omega) * dt / tau_omega
        th += omega * D2R * dt
        y += v * math.sin(th) * dt
        ys[i] = y
    ref = max(abs(y0), 1.0)
    over = np.where(np.abs(ys) > 0.02 * ref)[0]
    settle = (over[-1] + 1) * v * dt if len(over) else 0.0
    return settle, ys.min(), w_peak


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("traces", nargs="*", help="探索のトレース（*_trace*.csv）")
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--kp", type=float, help="トレースを走ったときの固定 KP [dps/count]（前の 0.02）")
    g.add_argument("--kv", type=float, help="トレースを走ったときの KP_PER_VELOCITY（省略で config の値）")
    ap.add_argument("--max-omega", type=float, default=90.0, help="--kp のときの補正の上限 [dps]")
    ap.add_argument("--ks", type=float, help="同定せずにこの KS [count/mm] を使う")
    ap.add_argument("--L", type=float, default=100.0, help="設計に使う L [mm]（センサーが見る位置の車軸からの前方距離）")
    ap.add_argument("--settle", type=float, default=270.0, help="目標の 2%% 収束距離 [mm]")
    args = ap.parse_args()
    cfg = read_config()

    ks = args.ks
    if ks is None:
        if not args.traces:
            sys.exit("トレースか --ks を指定する")
        if args.kp is not None:
            gain_of_v = lambda v: np.full_like(v, args.kp)
            max_of_v = lambda v: np.full_like(v, args.max_omega)
        else:
            kv = args.kv if args.kv is not None else cfg["KP_PER_VELOCITY"]
            gain_of_v = lambda v: kv * v
            max_of_v = lambda v: cfg.get("MAX_OMEGA_PER_VELOCITY", 1e9) * v
        segs = []
        for p in args.traces:
            segs += load_segments(p, gain_of_v, max_of_v)
        if not segs:
            sys.exit("使える直進の区間がない（wall_control が off のログ？）")
        print(f"区間 {len(segs)}，点 {sum(len(s[0]) for s in segs)}")
        for L in [0, 50, 80, 100, 120, 150]:
            k, r2, rms = fit_ks(segs, L)
            print(f"  L={L:4.0f} mm: KS={k:6.1f} count/mm  R²={r2:.3f}  rms={rms:5.0f} count")
        ks = fit_ks(segs, args.L)[0]
        print(f"→ KS = {ks:.1f} count/mm（L = {args.L:.0f} mm で）")

    sigma = 4.0 / args.settle
    G = 2.0 * sigma / args.L
    zeta = sigma / math.sqrt(G)
    kv = G / (ks * D2R)
    print(f"\n設計: 2% 収束 {args.settle:.0f} mm → σ={sigma:.4f}/mm，G={G:.3g}/mm²，ζ={zeta:.2f}")
    print(f"  KP_PER_VELOCITY = {kv:.3g} dps/count/(mm/s)"
          f"（KP: 300 mm/s で {kv*300:.3f}，500 で {kv*500:.3f}，1000 で {kv*1000:.3f}）")
    if "KP_PER_VELOCITY" in cfg:
        print(f"  config: KP_PER_VELOCITY = {cfg['KP_PER_VELOCITY']:.3g}，MAX_OMEGA_PER_VELOCITY = {cfg.get('MAX_OMEGA_PER_VELOCITY')}")

    kv_sim = cfg.get("KP_PER_VELOCITY", kv)
    max_per_v = cfg.get("MAX_OMEGA_PER_VELOCITY", 0.18)
    print(f"\n閉ループ（KP_PER_VELOCITY={kv_sim:.3g}，KS={ks:.1f}，L={args.L:.0f}，角度 PI 込み）")
    for v in [300, 500, 700, 1000, 1500]:
        s, ymin, wp = simulate(kv_sim, max_per_v, v, ks, args.L, cfg, y0=5.0)
        s10, _, wp10 = simulate(kv_sim, max_per_v, v, ks, args.L, cfg, y0=10.0)
        print(f"  v={v:5d}: y0=5 mm → 2% 収束 {s:4.0f} mm（行き過ぎ {min(ymin, 0):5.2f} mm，ω ピーク {wp:4.1f} dps）"
              f"  y0=10 mm → {s10:4.0f} mm（ω ピーク {wp10:4.1f} / 上限 {max_per_v*v:4.0f} dps）")


if __name__ == "__main__":
    main()
