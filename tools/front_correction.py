#!/usr/bin/env python3
"""探索のログから，前壁の距離による S90 の入口の補正（common/front_correction）を評価する。

各走行の S90 を積んだ読み（前壁あり）について，前後のずれ e と補正 δ を出す。
  - ログに front_err 列があればそれを使う（ファームが記録した e）。なければ ir_fl / ir_fr から
    tools/ir_calibration.json の換算表と REF（--ref，省略で config::front_correction）で計算し直す
  - 直前の動作ごと（直進の後・ターンの後）の e の分布。直進の後は壁切れで位置が合っているので，
    その中央値は REF の候補になる（--suggest-ref）
  - edges のログ（--edges，同じ順に並べる）があれば，その読みの後で最初に対応づいた壁切れのずれ
    （予想位置 − 実測，正なら機体は実際は前）と e の相関。補正が効くなら e と負の相関になる
    （e > 0 は機体が実際は後ろ）

使い方:
    python3 tools/front_correction.py tools/log/search/500_5.csv
    python3 tools/front_correction.py tools/log/search/500_{3,4,5}.csv \\
        --edges tools/log/search/500_edges{,_1,_2}.csv
    python3 tools/front_correction.py tools/log/search/500_*.csv --suggest-ref
"""
import argparse
import csv
import json
import math
import os
import re
import statistics as st
import sys

TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TOOLS_DIR)
CONFIG = os.path.join(ROOT, "Core", "Inc", "config", "mouse_config.hpp")

# solver/core/action.h（シミュレータの action_enum.py）の値
ACT_NAMES = {45: "1cell", 46: "S90L", 47: "S90R", 48: "back"}
TURNS = (46, 47)


def read_config():
    """mouse_config.hpp の config::front_correction / wall_edge の定数"""
    text = open(CONFIG, encoding="utf-8").read()
    out = {}
    for ns in ("front_correction", "wall_edge"):
        m = re.search(r"namespace config::%s \{(.*?)\n\}" % ns, text, re.S)
        if not m:
            continue
        for name, value in re.findall(r"constexpr\s+\w+\s+(\w+)\s*=\s*(-?[0-9.]+)f?;", m.group(1)):
            out[f"{ns}.{name}"] = float(value)
    return out


def load_table(calib, key):
    return [(float(v), float(d)) for v, d in calib["sensors"][key]["lut"]]


def to_mm(table, value):
    if value > table[0][0] or value < table[-1][0]:
        return math.nan
    for (v0, d0), (v1, d1) in zip(table, table[1:]):
        if value >= v1:
            return d0 + (d1 - d0) * ((v0 - value) / (v0 - v1) if v0 != v1 else 0.0)
    return math.nan


def correction(e, c):
    if math.isnan(e) or abs(e) <= c["front_correction.DEADBAND_MM"]:
        return 0.0
    mag = min(c["front_correction.GAIN"] * (abs(e) - c["front_correction.DEADBAND_MM"]), c["front_correction.MAX_MM"])
    return math.copysign(mag, e)


def corr(xs, ys):
    if len(xs) < 3 or st.pstdev(xs) == 0 or st.pstdev(ys) == 0:
        return math.nan
    mx, my = st.mean(xs), st.mean(ys)
    return sum((a - mx) * (b - my) for a, b in zip(xs, ys)) / len(xs) / st.pstdev(xs) / st.pstdev(ys)


def summary(label, values):
    if not values:
        print(f"  {label}: なし")
        return
    q = sorted(values)
    n = len(q)
    print(f"  {label}: n {n}, 平均 {st.mean(q):+.1f}, σ {st.pstdev(q):.1f}, "
          f"10/50/90% {q[n // 10]:+.1f} / {q[n // 2]:+.1f} / {q[(9 * n) // 10]:+.1f} mm")


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("logs", nargs="+", help="探索のログ（tools/log/search/<preset>.csv）")
    ap.add_argument("--edges", nargs="*", default=[], help="壁切れのログ（logs と同じ順）")
    ap.add_argument("--ref", nargs=2, type=float, metavar=("FL", "FR"),
                    help="REF_LEFT_MM / REF_RIGHT_MM を試す（ir_fl / ir_fr から計算し直す）")
    ap.add_argument("--calib", default=os.path.join(TOOLS_DIR, "ir_calibration.json"))
    ap.add_argument("--suggest-ref", action="store_true", help="直進の後の読みの距離の中央値を出す")
    ap.add_argument("--rows", action="store_true", help="読みごとの値を並べる")
    args = ap.parse_args()
    if args.edges and len(args.edges) != len(args.logs):
        print("front_correction: --edges は logs と同じ数だけ並べてください", file=sys.stderr)
        return 1

    c = read_config()
    ref = args.ref or (c["front_correction.REF_LEFT_MM"], c["front_correction.REF_RIGHT_MM"])
    calib = json.load(open(args.calib, encoding="utf-8"))
    fl_table, fr_table = load_table(calib, "ir_var_FL"), load_table(calib, "ir_var_FR")
    lo, hi = c["front_correction.MIN_DISTANCE_MM"], c["front_correction.MAX_DISTANCE_MM"]
    edge_offset = 0.5 * (c["wall_edge.OFFSET_LEFT_MM"] + c["wall_edge.OFFSET_RIGHT_MM"])
    print(f"REF FL {ref[0]:.1f} / FR {ref[1]:.1f} mm，範囲 {lo:g}〜{hi:g} mm，不感帯 "
          f"{c['front_correction.DEADBAND_MM']:g}，ゲイン {c['front_correction.GAIN']:g}，上限 "
          f"{c['front_correction.MAX_MM']:g} mm")

    by_prev = {}
    dist_after_straight = ([], [])
    pairs = []
    for k, path in enumerate(args.logs):
        rows = list(csv.DictReader(open(path, encoding="utf-8")))
        edges = []
        if args.edges:
            for r in csv.DictReader(open(args.edges[k], encoding="utf-8")):
                b = float(r["boundary"])
                if not math.isnan(b):
                    edges.append((float(r["x"]), b + edge_offset - float(r["x"])))
        use_logged = "front_err" in rows[0] and args.ref is None if rows else False
        print(f"{path}: {len(rows)} 読み{'（front_err 列を使う）' if use_logged else ''}")
        for i, r in enumerate(rows):
            action = int(float(r["action"]))
            if i == 0 or action not in TURNS or float(r["front"]) != 1.0:
                continue
            prev = int(float(rows[i - 1]["action"]))
            d_fl = to_mm(fl_table, float(r["ir_fl"]))
            d_fr = to_mm(fr_table, float(r["ir_fr"]))
            if use_logged:
                e = float(r["front_err"])
            elif lo <= d_fl <= hi and lo <= d_fr <= hi:
                # 読む位置（境界 − READ_LEAD）を過ぎた量はログからは分からないので 0 とする（1tick 以内）
                e = 0.5 * ((d_fl - ref[0]) + (d_fr - ref[1]))
            else:
                e = math.nan
            if prev == 45 and lo <= d_fl <= hi and lo <= d_fr <= hi:
                dist_after_straight[0].append(d_fl)
                dist_after_straight[1].append(d_fr)
            if math.isnan(e):
                continue
            by_prev.setdefault(prev, []).append(e)
            nxt = None
            if edges:
                pos = float(r["pos_measured"])
                later = [s for x, s in edges if pos < x < pos + 500.0]
                nxt = later[0] if later else None
                if nxt is not None:
                    pairs.append((e, nxt))
            if args.rows:
                print(f"  step {i:3d} {ACT_NAMES.get(action, action)} (前 {ACT_NAMES.get(prev, prev)}) "
                      f"FL {d_fl:6.1f} FR {d_fr:6.1f} e {e:+6.1f} δ {correction(e, c):+5.1f}"
                      + (f" 次の壁切れ {nxt:+6.1f}" if nxt is not None else ""))

    print("前後のずれ e（正：機体は実際は後ろ）")
    for prev in sorted(by_prev):
        summary(f"{ACT_NAMES.get(prev, prev)} の後", by_prev[prev])
    all_e = [e for v in by_prev.values() for e in v]
    summary("すべて", all_e)
    applied = [correction(e, c) for e in all_e]
    print(f"  補正がかかる読み {sum(1 for d in applied if d != 0)} / {len(applied)}，"
          f"|δ| 平均 {st.mean(abs(d) for d in applied) if applied else 0:.1f} mm")
    if pairs:
        xs, ys = [p[0] for p in pairs], [p[1] for p in pairs]
        print(f"次の壁切れのずれとの相関（効くなら負）: n {len(pairs)}, r {corr(xs, ys):+.2f}, "
              f"σ e {st.pstdev(xs):.1f} / 壁切れ {st.pstdev(ys):.1f} mm")
    if args.suggest_ref:
        fl, fr = dist_after_straight
        if fl:
            print(f"REF の候補（直進の後の読み {len(fl)} 回の中央値）: REF_LEFT_MM {st.median(fl):.1f}, "
                  f"REF_RIGHT_MM {st.median(fr):.1f}")
        else:
            print("REF の候補: 直進の後の読みがありません")
    return 0


if __name__ == "__main__":
    sys.exit(main())
