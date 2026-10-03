#!/usr/bin/env python3
"""壁切れの記録（*_edges.csv）から config::wall_edge の OFFSET_LEFT_MM / OFFSET_RIGHT_MM / LAG_S を求め，
verify / inject / 探索の補正量をまとめる。手順は tools/WALL_EDGE.md。

記録の列（app/wall_edge_log.hpp）: side（0 左，1 右），x，boundary（対応がなければ nan），offset（x − boundary），
shift（実測に足した補正），velocity（目標速度）。

モデル（common/wall_edge.hpp）: 検出したときの車軸の位置 − 区画境界 = OFFSET_side + LAG_S·v
  - 補正しない走行（shift がすべて 0：calib，SEARCH_CORRECTION=false の探索）の offset から最小二乗で求める
  - 速度が1種類なら LAG_S は決まらないので 0 とし，OFFSET は平均
  - 左右で LAG_S は共通（センサーの更新とフィルタは同じ）

使い方:
    python3 tools/wall_edge.py                                # tools/log/wall_edge/calib_*_edges.csv
    python3 tools/wall_edge.py tools/log/wall_edge/calib_500_edges.csv tools/log/search/500_edges.csv
    python3 tools/wall_edge.py --check tools/log/wall_edge/verify_500_edges.csv   # 補正量の確認
"""
import argparse
import csv
import glob
import math
import os
import statistics

TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_GLOB = os.path.join(TOOLS_DIR, "log", "wall_edge", "calib_*_edges.csv")
SIDES = {0: "LEFT", 1: "RIGHT"}


def read_edges(path):
    rows = []
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            rows.append({
                "file": os.path.basename(path),
                "side": int(float(r["side"])),
                "x": float(r["x"]),
                "boundary": float(r["boundary"]),
                "offset": float(r["offset"]),
                "shift": float(r["shift"]),
                "velocity": float(r["velocity"]),
            })
    return rows


def fit(rows):
    """offset = OFFSET_side + LAG·v を左右まとめて最小二乗で解く。(offsets, lag, residuals) を返す"""
    sides = sorted({r["side"] for r in rows})
    speeds = {round(r["velocity"]) for r in rows}
    if len(speeds) < 2:
        offsets = {s: statistics.fmean(r["offset"] for r in rows if r["side"] == s) for s in sides}
        lag = 0.0
    else:
        # 正規方程式（未知数：各 side の切片 + 共通の傾き）
        n = len(sides) + 1
        a = [[0.0] * n for _ in range(n)]
        b = [0.0] * n
        for r in rows:
            row = [1.0 if r["side"] == s else 0.0 for s in sides] + [r["velocity"]]
            for i in range(n):
                b[i] += row[i] * r["offset"]
                for j in range(n):
                    a[i][j] += row[i] * row[j]
        sol = solve(a, b)
        offsets = {s: sol[i] for i, s in enumerate(sides)}
        lag = sol[-1]
    residuals = [r["offset"] - (offsets[r["side"]] + lag * r["velocity"]) for r in rows]
    return offsets, lag, residuals


def solve(a, b):
    n = len(b)
    m = [row[:] + [b[i]] for i, row in enumerate(a)]
    for c in range(n):
        p = max(range(c, n), key=lambda i: abs(m[i][c]))
        m[c], m[p] = m[p], m[c]
        if abs(m[c][c]) < 1e-12:
            raise SystemExit("fit is singular (need edges on both speeds?)")
        for i in range(n):
            if i != c:
                k = m[i][c] / m[c][c]
                m[i] = [x - k * y for x, y in zip(m[i], m[c])]
    return [m[i][n] / m[i][i] for i in range(n)]


def calibrate(paths):
    rows = []
    for p in paths:
        edges = read_edges(p)
        if any(e["shift"] != 0.0 for e in edges):
            print(f"skip {os.path.basename(p)}: corrected run (use --check)")
            continue
        rows += edges
    unmatched = [r for r in rows if math.isnan(r["boundary"])]
    rows = [r for r in rows if not math.isnan(r["boundary"])]
    if unmatched:
        print(f"{len(unmatched)} edges without a boundary (ignored)")
    if not rows:
        raise SystemExit("no matched edges")

    print(f"{'file':28s} side   v[mm/s]  boundary  offset[mm]")
    for r in rows:
        print(f"{r['file']:28s} {SIDES[r['side']]:5s} {r['velocity']:7.0f} {r['boundary']:9.1f} {r['offset']:+10.2f}")

    offsets, lag, residuals = fit(rows)
    rms = math.sqrt(statistics.fmean(e * e for e in residuals))
    worst = max(abs(e) for e in residuals)
    print()
    for s, off in offsets.items():
        n = sum(1 for r in rows if r["side"] == s)
        print(f"{SIDES[s]:5s}: {n} edges, OFFSET {off:+.2f} mm")
    print(f"LAG_S {lag:.5f} s ({lag * 500:+.2f} mm at 500 mm/s), residual rms {rms:.2f} mm, max {worst:.2f} mm")
    if worst > 3.0:
        print("  note: residual > 3 mm: check THRESH_* (edge on a slope?) or a post-only reflection")
    print()
    print("config::wall_edge (Core/Inc/config/mouse_config.hpp):")
    for s, off in offsets.items():
        print(f"inline constexpr float OFFSET_{SIDES[s]}_MM = {off:.1f}f;")
    print(f"inline constexpr float LAG_S = {lag:.5f}f;")


def check(paths):
    for p in paths:
        edges = read_edges(p)
        print(f"{os.path.basename(p)}: {len(edges)} edges, total shift {sum(e['shift'] for e in edges):+.2f} mm")
        for i, e in enumerate(edges):
            b = "     none" if math.isnan(e["boundary"]) else f"{e['boundary']:9.1f}"
            print(f"  {i:2d} {SIDES[e['side']]:5s} x {e['x']:7.1f} boundary {b} shift {e['shift']:+6.2f}")
        corrected = [e["shift"] for e in edges if e["shift"] != 0.0]
        if len(corrected) > 1:
            later = corrected[1:]
            print(f"  after the first correction: mean {statistics.fmean(later):+.2f} mm, "
                  f"max |shift| {max(abs(x) for x in later):.2f} mm (should be ~0 when calibrated)")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("files", nargs="*", help="*_edges.csv（省略すると tools/log/wall_edge/calib_*_edges.csv）")
    ap.add_argument("--check", action="store_true", help="補正量を並べる（verify / inject / 補正ありの探索）")
    args = ap.parse_args()
    paths = args.files or sorted(glob.glob(DEFAULT_GLOB))
    if not paths:
        raise SystemExit(f"no files ({DEFAULT_GLOB})")
    if args.check:
        check(paths)
    else:
        calibrate(paths)


if __name__ == "__main__":
    main()
