#!/usr/bin/env python3
"""前の壁までの距離とIRセンサーの値の関係（Device → IR → Front sweep のログ）から，値 → 距離の換算表を作る。

正解の距離はエンコーダから作る：機体の後端を区画の後壁に当てて置いたとき，車軸から cells 区画先の前の壁の面まで
    d0 = 180·cells − 12 − BACK_TO_AXLE_MM
で，各サンプルの距離は d = d0 − current_distance_x（車軸から前の壁の面まで [mm]）。

やること：
  1. 4つの変数（ir_var_L/FL/FR/R）を，壁から離れるほど値が一貫して下がるもの（前のセンサー）と，
     そうでないもの（横のセンサー：横の壁を見ていて，前の壁が近い時だけ値が変わる）に分ける
     （変数名と読み込んでいるピンが食い違っている可能性があるため，データで確かめる）
  2. 前のセンサー：1mmごとの中央値で値が最大になる距離（ピーク）を探す。それより近いと発光・受光素子の位置の
     ずれで値が下がり，同じ値が2つの距離に対応するので，ピークより遠い側だけを使う
  3. ピークより遠い側の中央値に「距離に対して単調に減る」制約をかけて換算表（値 → 距離）を作る（PAVA）。
     近い側は受光が飽和に近く値の変化が小さいので，分解能（1サンプルのノイズ ÷ 傾き）が MAX_RES_MM より
     悪い所は有効範囲から外す
  4. 精度は，近づくときのデータで作った表を離れるときのデータで確かめる（逆も）。自分のデータに合わせすぎていないかを見る
  5. 比較のため，モデル d = A·v^p（power），d = a/√(v − b) + c（inv_sqrt）も当てはめて帯ごとの残差を出す
  6. 横のセンサー：遠い所の値と，前の壁の影響が出始める距離を出す（区画中央で止まると前の壁は84mm）

結果は tools/ir_calibration.json に保存する（換算をファームウェアに入れるときの元データ）。標準ライブラリだけで動く。

使い方:
    python3 tools/fit_ir.py                              # tools/log/ir_sweep/front_*.csv をすべて使う
    python3 tools/fit_ir.py tools/log/ir_sweep/front_2cell_1.csv --table
"""
import argparse
import bisect
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
SLOPE_HALF_MM = 3.0    # 傾きを求める窓の半分の幅
MAX_RES_MM = 3.0       # 分解能（1サンプルのノイズ ÷ 傾き）がこれより悪い所は有効範囲から外す
LUT_STEP_MM = 2.0      # 換算表の点の間隔
FRONT_CORR = -0.9      # 前のセンサー：距離と値の順位相関がこれより小さい（離れるほど一貫して下がる）
FRONT_FAR_RATIO = 0.6  # 前のセンサー：遠い端の値が，近い側から50mmの所の値のこの割合より小さい
SIDE_DEVIATION = 0.05  # 横のセンサー：遠い所の値からこの割合以上ずれたら前の壁の影響とみなす
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
        samples.append({"d": d0 - x, "dir": direction, "battery": float(r[col["battery"]]),
                        **{s: float(r[col[s]]) for s in SENSORS}})
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


def spearman(xs, ys):
    def ranks(a):
        order = sorted(range(len(a)), key=lambda i: a[i])
        r = [0.0] * len(a)
        for rank, i in enumerate(order):
            r[i] = rank
        return r
    rx, ry = ranks(xs), ranks(ys)
    mx, my = statistics.mean(rx), statistics.mean(ry)
    num = sum((a - mx) * (b - my) for a, b in zip(rx, ry))
    den = math.sqrt(sum((a - mx) ** 2 for a in rx) * sum((b - my) ** 2 for b in ry))
    return num / den if den else 0.0


def pava_decreasing(values, weights):
    """距離の昇順に並んだ値へ「単調に減る」制約をかけた最小二乗の解（Pool Adjacent Violators）"""
    blocks = []   # [平均, 重み, 個数]
    for v, w in zip(values, weights):
        blocks.append([v, w, 1])
        while len(blocks) >= 2 and blocks[-2][0] < blocks[-1][0]:   # 増えている所を前のブロックとまとめる
            v2, w2, n2 = blocks.pop()
            v1, w1, n1 = blocks.pop()
            blocks.append([(v1 * w1 + v2 * w2) / (w1 + w2), w1 + w2, n1 + n2])
    out = []
    for v, _, n in blocks:
        out += [v] * n
    return out


def local_slope(bins, i):
    """i番目のビンの付近（±SLOPE_HALF_MM）の中央値の傾き [値/mm]（最小二乗の直線）"""
    d0 = bins[i][0]
    pts = [(d, m) for d, m, _, _ in bins if abs(d - d0) <= SLOPE_HALF_MM]
    if len(pts) < 3:
        return 0.0
    mx = statistics.mean(d for d, _ in pts)
    my = statistics.mean(m for _, m in pts)
    sxx = sum((d - mx) ** 2 for d, _ in pts)
    return sum((d - mx) * (m - my) for d, m in pts) / sxx if sxx else 0.0


class Lut:
    """値 → 距離の換算表（値の降順＝距離の昇順の点を線形補間）"""

    def __init__(self, points):
        self.points = sorted(points)                      # (距離, 値)，距離の昇順
        self.vs = [-v for _, v in self.points]            # bisect 用に値を負にして昇順にする
        self.ds = [d for d, _ in self.points]

    def __call__(self, v):
        i = bisect.bisect_left(self.vs, -v)
        if i <= 0:
            return self.ds[0]
        if i >= len(self.vs):
            return self.ds[-1]
        v0, v1 = -self.vs[i - 1], -self.vs[i]
        t = 0.0 if v0 == v1 else (v0 - v) / (v0 - v1)
        return self.ds[i - 1] + t * (self.ds[i] - self.ds[i - 1])


def build_lut(bins, d_lo, d_hi):
    """d_lo〜d_hi のビンから単調な換算表を作る（LUT_STEP_MM ごとの点）"""
    sel = [(d, m, n) for d, m, n, _ in bins if d_lo <= d <= d_hi]
    if len(sel) < 5:
        return None
    iso = pava_decreasing([m for _, m, _ in sel], [n for _, _, n in sel])
    pts, last_d = [], None
    for (d, _, _), v in zip(sel, iso):
        if last_d is None or d - last_d >= LUT_STEP_MM - 1e-9 or d == sel[-1][0]:
            if not pts or v < pts[-1][1]:   # 値が同じ点は残さない（逆引きできない）
                pts.append((d, v))
                last_d = d
    return Lut(pts) if len(pts) >= 3 else None


def fit_power(points):
    xs = [math.log(v) for _, v in points]
    ys = [math.log(d) for d, _ in points]
    mx, my = statistics.mean(xs), statistics.mean(ys)
    p = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sum((x - mx) ** 2 for x in xs)
    a = math.exp(my - p * mx)
    return {"A": a, "p": p}, (lambda v, a=a, p=p: a * v ** p)


def fit_inv_sqrt(points):
    vmin = min(v for _, v in points)
    span = max(vmin, 1.0)
    best = None
    for b in [vmin - span * 10 ** (e / 20.0) for e in range(-60, 21)]:
        xs = [1.0 / math.sqrt(v - b) for _, v in points]
        ys = [d for d, _ in points]
        mx, my = statistics.mean(xs), statistics.mean(ys)
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


def band_errors(points, f):
    """[(距離, 値)] を f で距離に直したときの帯ごとの RMS 誤差"""
    out = []
    for lo, hi in BANDS:
        errs = [f(v) - d for d, v in points if lo <= d < hi and not math.isnan(f(v))]
        if errs:
            out.append({"d_lo": lo, "d_hi": hi, "rms_mm": math.sqrt(statistics.mean(e * e for e in errs)), "n": len(errs)})
    return out


def analyse_side(sensor, allb):
    far = [m for d, m, _, _ in allb if d >= allb[-1][0] - 100]
    far_med = statistics.median(far)
    # 遠い側から近づいて，遠い所の値から SIDE_DEVIATION 以上ずれ続け始める距離
    influence = None
    for d, m, _, _ in reversed(allb):
        if abs(m - far_med) > SIDE_DEVIATION * far_med:
            if influence is None:
                influence = d
        else:
            influence = None
    at84 = [m for d, m, _, _ in allb if abs(d - 84.0) <= 1.0]
    return {"role": "side", "far_value": far_med, "front_wall_influence_d": influence,
            "value_at_84mm": statistics.median(at84) if at84 else None}


def analyse(sensor, samples):
    allb = bins_of(samples, sensor)
    if len(allb) < 20:
        raise FitError(f"{sensor}: データが少なすぎます")
    d_min = allb[0][0]
    away = [(d, m) for d, m, _, _ in allb if d >= d_min + 40]   # 近すぎる所（ピーク付近）を除いて判定する
    corr = spearman([d for d, _ in away], [m for _, m in away]) if len(away) > 10 else 0.0
    mid = [m for d, m, _, _ in allb if abs(d - (d_min + 50)) <= 2]
    far = [m for d, m, _, _ in allb if d >= allb[-1][0] - 10]
    ratio = statistics.median(far) / statistics.median(mid) if mid and far and statistics.median(mid) > 0 else 1.0
    if not (corr < FRONT_CORR and ratio < FRONT_FAR_RATIO):
        info = analyse_side(sensor, allb)
        info.update({"corr": corr, "far_ratio": ratio})
        return info

    peak_i = max(range(len(allb)), key=lambda i: allb[i][1])
    d_peak, v_peak = allb[peak_i][0], allb[peak_i][1]
    beyond = allb[peak_i:]
    # 分解能：1サンプルのノイズ ÷ 傾き。近い側（飽和に近く平ら）と遠い側（値が小さい）で悪くなる
    reso = []
    for i in range(len(beyond)):
        slope = local_slope(beyond, i)
        reso.append(max(beyond[i][3], 0.5) / -slope if slope < 0 else float("inf"))
    ok = [r <= MAX_RES_MM for r in reso]
    if not any(ok):
        raise FitError(f"{sensor}: 分解能が {MAX_RES_MM}mm より良い所がありません")
    i_lo = ok.index(True)
    i_hi = len(ok) - 1 - ok[::-1].index(True)
    d_lo, d_hi = beyond[i_lo][0], beyond[i_hi][0]

    lut = build_lut(allb, d_lo, d_hi)
    if lut is None:
        raise FitError(f"{sensor}: 換算表を作れません")

    # 確かめ：近づくときのデータで作った表で離れるときの値を距離に直す（逆も）
    fwd, bwd = bins_of(samples, sensor, 1), bins_of(samples, sensor, -1)
    holdout_bands = []
    lut_f, lut_b = build_lut(fwd, d_lo, d_hi), build_lut(bwd, d_lo, d_hi)
    test_pts = [(d, m, lut_f) for d, m, _, _ in bwd if d_lo <= d <= d_hi and lut_f] + \
               [(d, m, lut_b) for d, m, _, _ in fwd if d_lo <= d <= d_hi and lut_b]
    if test_pts:
        for lo, hi in BANDS:
            errs = [t(m) - d for d, m, t in test_pts if lo <= d < hi]
            if errs:
                holdout_bands.append({"d_lo": lo, "d_hi": hi, "rms_mm": math.sqrt(statistics.mean(e * e for e in errs)),
                                      "bias_mm": statistics.mean(errs), "n": len(errs)})
    band_reso = []
    for lo, hi in BANDS:
        r = [x for (d, _, _, _), x in zip(beyond, reso) if lo <= d < hi and d_lo <= d <= d_hi]
        if r:
            band_reso.append({"d_lo": lo, "d_hi": hi, "resolution_mm": statistics.median(r)})

    # 比較：モデルの当てはめ（有効範囲の中央値で）
    pts = [(d, m) for d, m, _, _ in allb if d_lo <= d <= d_hi and m > 0]
    models = {}
    for name, fitter in (("power", fit_power), ("inv_sqrt", fit_inv_sqrt)):
        params, f = fitter(pts)
        models[name] = {"params": params, "bands": band_errors(pts, f)}

    return {"role": "front", "corr": corr, "far_ratio": ratio, "peak": {"d": d_peak, "v": v_peak},
            "valid": {"d_min": d_lo, "d_max": d_hi, "v_max": lut.points[0][1], "v_min": lut.points[-1][1]},
            "lut": [[round(v, 2), round(d, 2)] for d, v in lut.points], "holdout": holdout_bands,
            "resolution": band_reso, "models": models, "bins": allb}


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
        if info["role"] == "side":
            infl = info["front_wall_influence_d"]
            at84 = info["value_at_84mm"]
            print(f"  {s:10s}: 横のセンサー（遠い所の値 {info['far_value']:.0f}，順位相関 {info['corr']:+.2f}）")
            if infl is not None:
                txt = f"，84mm で {at84:.0f}（{(at84 / info['far_value'] - 1) * 100:+.0f}%）" if at84 else ""
                print(f"              前の壁が {infl:.0f}mm より近いと値が{SIDE_DEVIATION * 100:.0f}%以上変わる{txt}")
            out["sensors"][s] = {k: v for k, v in info.items()}
            continue
        print(f"  {s:10s}: 前のセンサー（順位相関 {info['corr']:+.2f}）。ピーク {info['peak']['d']:.1f}mm（値 {info['peak']['v']:.0f}）")
        print(f"              換算表の有効範囲 {info['valid']['d_min']:.1f}〜{info['valid']['d_max']:.1f}mm"
              f"（値 {info['valid']['v_max']:.0f}〜{info['valid']['v_min']:.0f}，{len(info['lut'])}点）")
        print("              距離の帯     換算表（近づく⇔離れるで確かめ） 分解能   モデル残差 power / inv_sqrt")
        models = {k: {bd["d_lo"]: bd["rms_mm"] for bd in m["bands"]} for k, m in info["models"].items()}
        reso = {bd["d_lo"]: bd["resolution_mm"] for bd in info["resolution"]}
        for bd in info["holdout"]:
            lo = bd["d_lo"]
            print(f"              {lo:3d}〜{bd['d_hi']:3d}mm   RMS {bd['rms_mm']:5.2f}mm（偏り {bd['bias_mm']:+.2f}）"
                  f"        {reso.get(lo, float('nan')):5.2f}mm   {models['power'].get(lo, float('nan')):6.2f} / "
                  f"{models['inv_sqrt'].get(lo, float('nan')):6.2f}mm")
        if args.table:
            print("              距離[mm]  値（中央値）")
            for d, m, _, _ in info["bins"]:
                if abs(d - round(d / 10) * 10) < BIN_MM / 2 + 1e-9:
                    print(f"              {d:7.1f}   {m:7.1f}")
        out["sensors"][s] = {k: v for k, v in info.items() if k != "bins"}
        out["sensors"][s]["models"] = {k: {"params": m["params"], "bands": m["bands"]} for k, m in info["models"].items()}

    fronts = [s for s, i in result.items() if i["role"] == "front"]
    print(f"前のセンサーの変数: {', '.join(fronts) if fronts else 'なし'}"
          "（左右どちらかは手でふさいで確かめる。feature/search の wall_sensor.cpp の対応と比べる）")

    def no_nan(x):   # JSON に NaN・inf を書かない（測れなかった値は null）
        if isinstance(x, float) and (math.isnan(x) or math.isinf(x)):
            return None
        if isinstance(x, dict):
            return {k: no_nan(v) for k, v in x.items()}
        if isinstance(x, (list, tuple)):
            return [no_nan(v) for v in x]
        return x

    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(no_nan(out), f, ensure_ascii=False, indent=2)
    print(f"保存: {os.path.relpath(args.out)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
