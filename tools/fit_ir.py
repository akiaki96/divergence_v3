#!/usr/bin/env python3
"""前の壁までの距離とIRセンサーの値の関係（Device → IR → Front sweep のログ）から，値 → 距離の換算を当てはめる。

正解の距離はエンコーダから作る：機体の後端を区画の後壁に当てて置いたとき，車軸から cells 区画先の前の壁の面まで
    d0 = 180·cells − 12 − BACK_TO_AXLE_MM
で，各サンプルの距離は d = d0 − current_distance_x（車軸から前の壁の面まで [mm]）。

やること：
  1. 4つの変数（ir_var_L/FL/FR/R）のうち，距離で値が大きく変わるものを前のセンサーと判断する
     （変数名と読み込んでいるピンが食い違っている可能性があるため，データで確かめる）
  2. 1mmごとの中央値で値が最大になる距離（ピーク）を探す。それより近いと発光・受光素子の位置のずれで値が下がり，
     同じ値が2つの距離に対応するので，ピークより遠い側だけを有効範囲として当てはめる
  3. 2つのモデルを当てはめ，残差の小さい方を選ぶ（v はセンサーの値，d は距離 [mm]）
       power    : d = A · v^p                （反射光が距離のべき乗で弱まる）
       inv_sqrt : d = a / √(v − b) + c       （反射光 ∝ 1/(d − c)² に暗い分の b を足したもの）
  4. 近づくときと離れるときの差（平滑化の遅れ・ヒステリシス），静止時のノイズ，電池電圧を確かめる

結果は tools/ir_calibration.json に保存する（換算をファームウェアに入れるときの元データ）。
標準ライブラリだけで動く。

使い方:
    python3 tools/fit_ir.py                              # tools/log/ir_sweep/front_*.csv をすべて使う
    python3 tools/fit_ir.py tools/log/ir_sweep/front_2cell_1.csv --table
"""
import argparse
import csv
import datetime
import glob
import json
import math
import os
import re
import statistics
import sys

TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_DIR = os.path.dirname(TOOLS_DIR)
CELL_MM = 180.0
WALL_HALF_MM = 6.0
SENSORS = ["ir_var_L", "ir_var_FL", "ir_var_FR", "ir_var_R"]
LOG_NAME = re.compile(r"^front_(?P<cells>\d+)cell(?:_\d+)?\.csv$")

BIN_MM = 1.0           # 中央値をとる距離の幅
MIN_BIN_COUNT = 3      # これより少ないビンは使わない
PEAK_MARGIN_MM = 2.0   # ピークからこれだけ遠い所から有効範囲にする
FRONT_SPAN = 20.0      # 値の幅がノイズのこの倍より大きければ距離で変わる（前のセンサー）とみなす
MAX_RES_MM = 3.0       # 1サンプルのノイズを距離に直した分解能がこれより悪い（遠い）所は当てはめに使わない
NEAR_TOL_MM = 1.0      # 近い端の残差（3ビンの平均）がこれより大きい間は近い側を削る（値が下がる効果が残る所を除く）
BANDS = [(60, 80), (80, 100), (100, 130), (130, 170), (170, 230), (230, 320)]   # 精度を示す距離の帯 [mm]


class FitError(Exception):
    pass


def read_back_to_axle():
    path = os.path.join(REPO_DIR, "Core", "Inc", "config", "mouse_config.hpp")
    with open(path, encoding="utf-8") as f:
        m = re.search(r"BACK_TO_AXLE_MM\s*=\s*([0-9.]+)f", f.read())
    if not m:
        raise FitError(f"{path} に BACK_TO_AXLE_MM が見つかりません")
    return float(m.group(1))


def load(path, back_to_axle):
    m = LOG_NAME.match(os.path.basename(path))
    if not m:
        raise FitError(f"{path}: ファイル名から区画数を読めません（例 front_1cell.csv）")
    cells = int(m.group("cells"))
    d0 = CELL_MM * cells - WALL_HALF_MM - (WALL_HALF_MM + back_to_axle)
    with open(path, newline="") as f:
        rows = list(csv.reader(f))
    header, data = rows[0], [r for r in rows[1:] if r]
    need = ["current_distance_x", "target_distance_x", "battery"] + SENSORS
    missing = [n for n in need if n not in header]
    if missing:
        raise FitError(f"{path}: 列がありません: {', '.join(missing)}（Front sweep のログか確認）")
    col = {n: header.index(n) for n in header}
    samples = []
    prev_target = None
    for r in data:
        x = float(r[col["current_distance_x"]])
        target = float(r[col["target_distance_x"]])
        # 近づく(+1)・離れる(−1)・止まっている(0) を目標位置の変化から決める
        direction = 0 if prev_target is None or abs(target - prev_target) < 1e-4 else (1 if target > prev_target else -1)
        prev_target = target
        samples.append({
            "d": d0 - x,
            "dir": direction,
            "battery": float(r[col["battery"]]),
            **{s: float(r[col[s]]) for s in SENSORS},
        })
    return {"path": path, "cells": cells, "d0": d0, "samples": samples}


def bins_of(samples, sensor, direction=None):
    """1mmごとの (距離の中心, 中央値, 個数, ばらつき[MADから求めた標準偏差]) のリスト（距離の昇順）"""
    groups = {}
    for s in samples:
        if direction is not None and s["dir"] != direction:
            continue
        groups.setdefault(math.floor(s["d"] / BIN_MM), []).append(s[sensor])
    out = []
    for k in sorted(groups):
        vals = groups[k]
        if len(vals) < MIN_BIN_COUNT:
            continue
        med = statistics.median(vals)
        mad = statistics.median(abs(v - med) for v in vals)
        out.append(((k + 0.5) * BIN_MM, med, len(vals), 1.4826 * mad))
    return out


def fit_power(points):
    """d = A·v^p を ln d = p·ln v + ln A の最小二乗で"""
    xs = [math.log(v) for _, v in points]
    ys = [math.log(d) for d, _ in points]
    n = len(xs)
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    p = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx
    a = math.exp(my - p * mx)
    return {"A": a, "p": p}, (lambda v, a=a, p=p: a * v ** p)


def fit_inv_sqrt(points):
    """d = a/√(v − b) + c。b を走査し，各 b で a, c を線形の最小二乗で決める"""
    vmin = min(v for _, v in points)
    best = None
    # b は v の最小値より小さい範囲（負もありうる）を対数的に細かく探す
    span = max(vmin, 1.0)
    candidates = [vmin - span * f for f in [10 ** (e / 20.0) for e in range(-60, 21)]]
    for b in candidates:
        xs = [1.0 / math.sqrt(v - b) for _, v in points]
        ys = [d for d, _ in points]
        n = len(xs)
        mx, my = sum(xs) / n, sum(ys) / n
        sxx = sum((x - mx) ** 2 for x in xs)
        if sxx <= 0:
            continue
        a = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx
        c = my - a * mx
        sse = sum((a * x + c - y) ** 2 for x, y in zip(xs, ys))
        if best is None or sse < best[0]:
            best = (sse, a, b, c)
    _, a, b, c = best
    return {"a": a, "b": b, "c": c}, (lambda v, a=a, b=b, c=c: a / math.sqrt(v - b) + c if v > b else float("nan"))


def residuals(points, f):
    errs = [f(v) - d for d, v in points]
    rms = math.sqrt(sum(e * e for e in errs) / len(errs))
    return rms, max(abs(e) for e in errs)


def analyse(sensor, samples):
    allb = bins_of(samples, sensor)
    if len(allb) < 10:
        raise FitError(f"{sensor}: データが少なすぎます")
    meds = [m for _, m, _, _ in allb]
    noise = statistics.median(s for _, _, _, s in allb)
    span = max(meds) - min(meds)
    info = {"span": span, "noise": noise, "front": span > FRONT_SPAN * max(noise, 1.0)}
    if not info["front"]:
        return info

    # ピーク：中央値が最大の距離。それより近い側は値が下がる（同じ値が2つの距離に対応する）
    peak_i = max(range(len(allb)), key=lambda i: allb[i][1])
    d_peak, v_peak = allb[peak_i][0], allb[peak_i][1]
    cand = [(d, m, sd) for d, m, _, sd in allb if d >= d_peak + PEAK_MARGIN_MM and m > 0]
    if len(cand) < 10:
        raise FitError(f"{sensor}: ピーク（{d_peak:.1f}mm）より遠いデータが少なすぎます")

    # 遠い側：値が小さく，1サンプルのノイズが距離に直すと大きい所は外す（分解能 = ノイズ × |dd/dv|）
    _, f0 = fit_power([(d, m) for d, m, _ in cand])
    def resolution(m, sd, f):
        return max(sd, 0.5) * abs(f(m * 1.01) - f(m * 0.99)) / (0.02 * m)
    cand = [(d, m, sd) for d, m, sd in cand if resolution(m, sd, f0) <= MAX_RES_MM]

    # 近い側：ピークのすぐ外もまだ値が下がる効果が残る。近い端の残差が大きい間は削って当てはめ直す
    cand.sort()
    while True:
        pts = [(d, m) for d, m, _ in cand]
        if len(pts) < 10:
            raise FitError(f"{sensor}: 有効なデータが少なすぎます（近い側の残差が {NEAR_TOL_MM}mm に収まらない）")
        fits = {}
        for name, fitter in (("power", fit_power), ("inv_sqrt", fit_inv_sqrt)):
            params, f = fitter(pts)
            rms, mx = residuals(pts, f)
            fits[name] = {"params": params, "rms_mm": rms, "max_mm": mx, "f": f}
        best = min(fits, key=lambda k: fits[k]["rms_mm"])
        f = fits[best]["f"]
        near_err = abs(statistics.mean(f(m) - d for d, m in pts[:3]))
        if near_err <= NEAR_TOL_MM:
            break
        cand = cand[1:]

    # 近づくとき・離れるときの差：同じ距離のビンの中央値を，選んだモデルで距離に直して比べる
    d_lo, d_hi = cand[0][0], cand[-1][0]
    fwd = {round(d, 1): m for d, m, _, _ in bins_of(samples, sensor, 1)}
    bwd = {round(d, 1): m for d, m, _, _ in bins_of(samples, sensor, -1)}
    common = [d for d in fwd if d in bwd and d_lo <= d <= d_hi]
    hyst = statistics.median(f(fwd[d]) - f(bwd[d]) for d in common) if common else float("nan")

    # 距離の帯ごとの精度：残差（モデルの誤差）と分解能（1サンプルのノイズを距離に直したもの）
    bands = []
    for lo, hi in BANDS:
        inb = [(d, m, sd) for d, m, sd in cand if lo <= d < hi]
        if not inb:
            continue
        res = math.sqrt(statistics.mean((f(m) - d) ** 2 for d, m, _ in inb))
        reso = statistics.median(resolution(m, sd, f) for _, m, sd in inb)
        bands.append({"d_lo": lo, "d_hi": hi, "rms_mm": res, "resolution_mm": reso, "bins": len(inb)})

    info.update({
        "peak": {"d": d_peak, "v": v_peak},
        "valid": {"d_min": d_lo, "d_max": d_hi,
                  "v_min": min(m for _, m, _ in cand), "v_max": max(m for _, m, _ in cand)},
        "fits": fits, "best": best, "hysteresis_mm": hyst, "bands": bands, "bins": allb,
    })
    return info


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("logs", nargs="*", help="Front sweep のログ（省略で tools/log/ir_sweep/front_*.csv）")
    ap.add_argument("--back-to-axle", type=float, help="走ったときの BACK_TO_AXLE_MM（省略で mouse_config.hpp の値）")
    ap.add_argument("--out", default=os.path.join(TOOLS_DIR, "ir_calibration.json"))
    ap.add_argument("--table", action="store_true", help="10mmごとの値の表も出す")
    args = ap.parse_args()

    paths = args.logs or sorted(glob.glob(os.path.join(TOOLS_DIR, "log", "ir_sweep", "front_*.csv")))
    try:
        if not paths:
            raise FitError("ログがありません（Device → IR → Front sweep を走らせる）")
        b = args.back_to_axle if args.back_to_axle is not None else read_back_to_axle()
        logs = [load(p, b) for p in paths]
        samples = [s for lg in logs for s in lg["samples"]]
        result = {s: analyse(s, samples) for s in SENSORS}
    except (OSError, FitError) as e:
        print(f"fit_ir: エラー: {e}", file=sys.stderr)
        return 1

    batt = [s["battery"] for s in samples]
    print(f"ログ {len(logs)}本（{', '.join(os.path.basename(lg['path']) for lg in logs)}），{len(samples)}サンプル，"
          f"BACK_TO_AXLE_MM {b:g}，距離 {min(s['d'] for s in samples):.1f}〜{max(s['d'] for s in samples):.1f}mm，"
          f"電池 {min(batt):.2f}〜{max(batt):.2f}V")
    out = {"generated_at": datetime.datetime.now().isoformat(timespec="seconds"),
           "logs": [os.path.relpath(lg["path"], TOOLS_DIR) for lg in logs], "back_to_axle_mm": b,
           "distance": "車軸から前の壁の面まで [mm]", "battery_v": [min(batt), max(batt)], "sensors": {}}
    for s, info in result.items():
        if not info["front"]:
            print(f"  {s:10s}: 距離でほとんど変わらない（幅 {info['span']:.0f}，ノイズ {info['noise']:.1f}）→ 前のセンサーではない")
            continue
        best = info["fits"][info["best"]]
        print(f"  {s:10s}: 前のセンサー。ピーク {info['peak']['d']:.1f}mm（値 {info['peak']['v']:.0f}）より近いと値が下がる")
        print(f"              有効範囲 {info['valid']['d_min']:.1f}〜{info['valid']['d_max']:.1f}mm（値 {info['valid']['v_min']:.0f}〜{info['valid']['v_max']:.0f}）")
        for name, fit in info["fits"].items():
            mark = "←" if name == info["best"] else "  "
            params = ", ".join(f"{k}={v:.6g}" for k, v in fit["params"].items())
            print(f"              {name:8s} {params:45s} 残差 RMS {fit['rms_mm']:.2f}mm, 最大 {fit['max_mm']:.2f}mm {mark}")
        print(f"              近づく−離れるの差 {info['hysteresis_mm']:+.2f}mm")
        print("              距離の帯      残差RMS   分解能（1サンプル）")
        for bd in info["bands"]:
            print(f"              {bd['d_lo']:3d}〜{bd['d_hi']:3d}mm   {bd['rms_mm']:5.2f}mm   {bd['resolution_mm']:5.2f}mm")
        if args.table:
            print("              距離[mm]  値（中央値）")
            for d, m, _, _ in info["bins"]:
                if abs(d - round(d / 10) * 10) < BIN_MM / 2 + 1e-9:
                    print(f"              {d:7.1f}   {m:7.1f}")
        out["sensors"][s] = {
            "model": info["best"], "params": best["params"],
            "rms_mm": best["rms_mm"], "max_mm": best["max_mm"],
            "peak": info["peak"], "valid": info["valid"],
            "hysteresis_mm": info["hysteresis_mm"], "bands": info["bands"],
            "alternatives": {k: {"params": v["params"], "rms_mm": v["rms_mm"]} for k, v in info["fits"].items() if k != info["best"]},
        }
    def no_nan(x):   # JSON に NaN を書かない（測れなかった値は null）
        if isinstance(x, float) and math.isnan(x):
            return None
        if isinstance(x, dict):
            return {k: no_nan(v) for k, v in x.items()}
        if isinstance(x, list):
            return [no_nan(v) for v in x]
        return x

    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(no_nan(out), f, ensure_ascii=False, indent=2)
    print(f"保存: {os.path.relpath(args.out)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
