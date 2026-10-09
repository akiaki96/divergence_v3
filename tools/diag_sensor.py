#!/usr/bin/env python3
"""斜め走行のセンサーのログ（Device → IR → Diagonal，tools/log/diag/*.csv）から，切れ目からの距離に対する
センサー値の表（理想のリファレンス値）を作る。手順は tools/DIAGONAL.md。

方法（naophis「斜めの姿勢制御をするには」2018-12-08）：横の 45° センサー（ir_l / ir_r）の値が閾値を下回った
ところ（壁・柱の切れ目）からの距離を横軸に，各センサーの値を並べると，同じ側の柱の周期 180√2 ≈ 254.6 mm
ごとに同じ形になる。距離ごとの平均を表にして，走るときはその距離の値を目標にする。
切れ目の直後・直前は値が急に変わって不安定なので「使わない」と印を付ける（--guard で幅を変える）。
同じ側の柱からの切れ目の位相は，左右の平均（前後のずれ）と左右の差の半分（横のずれ，左が正）に分けて出す。

ログの列（test/diag_sensor_test.hpp）：diag_x（入45°の出口の基準点＝区画の辺の中点からの距離），target_diag_x，
angle_error，ir_l / ir_fl / ir_fr / ir_r，since_l / since_r（機体の DiagEdge が数えた切れ目からの距離）。

切れ目はこのスクリプトで検出し直す（DiagEdge と同じ：ヒステリシス，tick 間の補間，短い壁は使わない）。
閾値の既定は Core/Inc/config/mouse_config.hpp の config::diag。--on / --off で変えて表を作り直せる。

使い方:
    python3 tools/diag_sensor.py                              # tools/log/diag/*.csv
    python3 tools/diag_sensor.py tools/log/diag/IN45_500_right_n8.csv --plot
    python3 tools/diag_sensor.py --on 400 380 --off 300 280   # 閾値（左 右）を変えて検出し直す
    python3 tools/diag_sensor.py --selftest                   # 合成データで確かめる（ロボットなし）
"""
import argparse
import csv
import glob
import json
import math
import os
import re
import statistics
import sys

import numpy as np

TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(TOOLS_DIR)
DEFAULT_GLOB = os.path.join(TOOLS_DIR, "log", "diag", "*.csv")
CONFIG_HPP = os.path.join(ROOT_DIR, "Core", "Inc", "config", "mouse_config.hpp")

PITCH = 90.0 * math.sqrt(2.0)     # 辺の中点（柱の並び）の間隔
PERIOD = 2.0 * PITCH              # 同じ側の柱の間隔
SIDES = ("L", "R")
# センサー → どちらの側の切れ目からの距離で表にするか
SENSORS = (("ir_l", "L"), ("ir_fl", "L"), ("ir_fr", "R"), ("ir_r", "R"))
MIN_VELOCITY = 100.0              # [mm/s] これより遅いところ（止まる前後）は使わない
MAX_ANGLE_ERROR = 2.0             # [deg] これより向きがずれた走行は表の質が落ちる（警告）
CONTROL_RUN = re.compile(r"_(ctrl|inj)(_\d+)?\.csv$")   # 斜めの姿勢制御をかけた走行（表には使わない）


# ---------------------------------------------------------------- 設定

def read_config(path=CONFIG_HPP):
    """config::diag の閾値と MIN_WALL_MM を読む"""
    text = open(path, encoding="utf-8").read()
    m = re.search(r"namespace config::diag \{(.*?)\n\}", text, re.S)
    if m is None:
        raise SystemExit(f"config::diag not found in {path}")
    body = m.group(1)

    def value(name):
        v = re.search(rf"\b{name}\s*=\s*([-0-9.]+)f?;", body)
        if v is None:
            raise SystemExit(f"config::diag::{name} not found")
        return float(v.group(1))

    return {
        "on": {"L": value("THRESH_ON_LEFT"), "R": value("THRESH_ON_RIGHT")},
        "off": {"L": value("THRESH_OFF_LEFT"), "R": value("THRESH_OFF_RIGHT")},
        "min_wall": value("MIN_WALL_MM"),
    }


# ---------------------------------------------------------------- 読み込み

def turn_dir(name):
    if "_left" in name:
        return "L"
    if "_right" in name:
        return "R"
    raise SystemExit(f"{name}: cannot tell the turn direction (expected _left / _right in the name)")


def load(path):
    cols = {}
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            for k, v in r.items():
                if k is None or k == "":
                    continue
                cols.setdefault(k, []).append(float(v) if v not in ("", None) else math.nan)
    run = {k: np.asarray(v, dtype=float) for k, v in cols.items()}
    need = ["Global_time", "diag_x", "target_diag_x"] + [s for s, _ in SENSORS]
    missing = [k for k in need if k not in run]
    if missing:
        raise SystemExit(f"{path}: missing columns {missing}")
    run["name"] = os.path.basename(path)
    run["dir"] = turn_dir(run["name"])
    return run


def control_summary(runs):
    """機体の DiagControl の列（diag_lat / diag_offset）があれば，斜めの直線の前半・後半の横のずれの推定と補正を出す"""
    rows = [r for r in runs if "diag_lat" in r]
    if not rows:
        return
    print("\nonboard DiagControl (diag_lat: lateral estimate, left +; diag_offset: heading added) on the diagonal:")
    for r in rows:
        x, lat, off = r["diag_x"], r["diag_lat"], r["diag_offset"]
        end = float(np.nanmax(x))
        def mean(lo, hi, v):
            m = (x >= lo) & (x < hi) & np.isfinite(v)
            return float(np.mean(v[m])) if m.any() else math.nan
        q = end / 4.0
        lats = " ".join(f"{mean(i * q, (i + 1) * q, lat):+5.1f}" for i in range(4))
        print(f"  {r['name']}: diag_lat by quarter {lats} mm, diag_offset end {float(off[np.isfinite(off)][-1]):+.2f} deg"
              f" (max |{float(np.nanmax(np.abs(off))):.2f}|)")


def moving_mask(run):
    """斜めの直線を走っている間（目標速度 ≥ MIN_VELOCITY）"""
    t = run["Global_time"]
    x = run["target_diag_x"]
    if len(t) < 3:
        return np.zeros(len(t), dtype=bool)
    v = np.gradient(x, t)
    return v >= MIN_VELOCITY


def pillar0(side, turn):
    """その側の最初の柱の位置（diag_x）。曲がった側（内側）は −PITCH/2，反対側（外側）は +PITCH/2"""
    return -PITCH / 2.0 if side == turn else PITCH / 2.0


# ---------------------------------------------------------------- 切れ目

def detect_edges(x, value, on, off, min_wall):
    """DiagEdge::update() と同じ。OFF を下回った位置（補間）のリスト"""
    edges = []
    state_on = None
    on_since = 0.0
    prev_v = prev_x = None
    for xi, vi in zip(x, value):
        if math.isnan(vi):
            continue
        if state_on is None:
            state_on = vi >= on
            on_since = xi
        elif not state_on and vi >= on:
            state_on = True
            on_since = xi
        elif state_on and vi < off:
            state_on = False
            t = 1.0 if prev_v == vi else (prev_v - off) / (prev_v - vi)
            xe = prev_x + t * (xi - prev_x)
            if xe - on_since >= min_wall:
                edges.append(xe)
        prev_v, prev_x = vi, xi
    return edges


def since_from_edges(x, edges, complete=False):
    """各サンプルの，それより前の最後の切れ目からの距離（なければ NaN）。
    complete なら最後の切れ目より後も NaN にする（切れ目と切れ目の間の周期だけ残す）"""
    out = np.full(len(x), np.nan)
    e = np.asarray(edges)
    if len(e) == 0:
        return out
    idx = np.searchsorted(e, x, side="right") - 1
    ok = idx >= 0
    out[ok] = x[ok] - e[idx[ok]]
    if complete:
        out[x > e[-1]] = np.nan
    return out


def onboard_edges(x, since):
    """機体の since_* の列から切れ目の位置を戻す（since が小さくなったところ）"""
    edges = []
    prev = math.nan
    for xi, si in zip(x, since):
        if not math.isnan(si) and (math.isnan(prev) or si < prev - 1.0):
            edges.append(xi - si)
        prev = si
    return edges


def wrap(phase):
    """[−PERIOD/2, PERIOD/2) に入れる"""
    return (phase + PERIOD / 2.0) % PERIOD - PERIOD / 2.0


# ---------------------------------------------------------------- 表

def make_table(since, value, bin_mm):
    edges = np.arange(0.0, PERIOD + bin_mm, bin_mm)
    centers = (edges[:-1] + edges[1:]) / 2.0
    mean = np.full(len(centers), np.nan)
    std = np.full(len(centers), np.nan)
    n = np.zeros(len(centers), dtype=int)
    ok = ~np.isnan(since) & ~np.isnan(value) & (since >= 0.0) & (since < edges[-1])
    b = np.floor(since[ok] / bin_mm).astype(int)
    v = value[ok]
    for i in range(len(centers)):
        vi = v[b == i]
        n[i] = len(vi)
        if len(vi) > 0:
            mean[i] = float(np.mean(vi))
            std[i] = float(np.std(vi))
    return centers, mean, std, n


def unstable_bins(centers, n, min_n, guard_after, guard_before):
    """使わない区間：サンプルが少ない，切れ目の直後 guard_after mm と次の切れ目の直前 guard_before mm。
    ばらつき・傾きでは決めない：壁に近づく区間（切れ目の手前）は値が急に増え，走行ごとの横のずれで
    ばらつきも大きくなるが，横のずれへの感度が最も高い区間でもある（2026-10-06 の実機ログで，
    走行ごとのばらつきを横のずれに換算すると 50〜235 mm で一定の約 4 mm）"""
    bad = n < min_n
    bad |= centers < guard_after
    bad |= centers > PERIOD - guard_before
    return bad


def ranges(centers, mask, bin_mm):
    """mask が True の区間を [(start, end)] にまとめる"""
    out = []
    start = None
    for c, m in zip(centers, mask):
        if m and start is None:
            start = c - bin_mm / 2.0
        if not m and start is not None:
            out.append((start, c - bin_mm / 2.0))
            start = None
    if start is not None:
        out.append((start, centers[-1] + bin_mm / 2.0))
    # 2ビン以下の隙間はつなぐ（使う区間が細切れにならないように）
    merged = []
    for a, b in out:
        if merged and a - merged[-1][1] <= 2.0 * bin_mm + 1e-6:
            merged[-1] = (merged[-1][0], b)
        else:
            merged.append((a, b))
    return merged


def suggest_thresholds(values):
    """値の分布の下 5% と上 5% の間で，OFF を 40%，ON を 60% の高さに置く"""
    v = values[~np.isnan(values)]
    if len(v) == 0:
        return None
    lo, hi = np.percentile(v, 5), np.percentile(v, 95)
    return {"low": float(lo), "high": float(hi), "off": float(lo + 0.4 * (hi - lo)), "on": float(lo + 0.6 * (hi - lo))}


# ---------------------------------------------------------------- 解析

def analyze(runs, cfg, bin_mm=2.0, min_n=3, guard_after=10.0, guard_before=15.0, quiet=False):
    def say(*a):
        if not quiet:
            print(*a)

    pooled = {s: ([], []) for s, _ in SENSORS}   # sensor → (since, value)
    phases = {s: [] for s in SIDES}              # side → [(file, phase)]
    decomp = []                                  # 走行ごと (file, turn, 前後, 横)
    raw_side = {s: [] for s in SIDES}
    for run in runs:
        m = moving_mask(run)
        x = run["diag_x"][m]
        if len(x) == 0:
            say(f"{run['name']}: no samples while moving")
            continue
        ae = run.get("angle_error")
        ae_max = float(np.nanmax(np.abs(ae[m]))) if ae is not None else math.nan
        say(f"{run['name']}: turn {run['dir']}, diag_x {x[0]:.1f} .. {x[-1]:.1f} mm, {len(x)} samples "
            f"(every {np.median(np.diff(x)):.2f} mm), |angle error| max {ae_max:.2f} deg"
            + ("  << WARNING: heading drifted" if ae_max > MAX_ANGLE_ERROR else ""))
        since = {}
        for side, col in (("L", "ir_l"), ("R", "ir_r")):
            val = run[col][m]
            raw_side[side].append(val)
            edges = detect_edges(x, val, cfg["on"][side], cfg["off"][side], cfg["min_wall"])
            # 表には切れ目と切れ目の間の周期だけを使う。最後の切れ目より後は，並べた壁の先を見ている
            # （ビームの先に壁がなく値が落ちたまま）か，止まる前の減速で，周期の形にならない
            since[side] = since_from_edges(x, edges, complete=True)
            p0 = pillar0(side, run["dir"])
            ph = [wrap(e - p0) for e in edges]
            phases[side].extend((run["name"], p) for p in ph)
            inner = "inner" if side == run["dir"] else "outer"
            say(f"  {side} ({inner}): {len(edges)} edges at " + ", ".join(f"{e:.1f}" for e in edges))
            if ph:
                say(f"     phase from its pillar: " + ", ".join(f"{p:+.1f}" for p in ph) + " mm")
            if len(edges) >= 2:
                d = np.diff(edges)
                say(f"     spacing {np.mean(d):.1f} mm (period {PERIOD:.1f}), min {np.min(d):.1f} max {np.max(d):.1f}")
            col_since = "since_l" if side == "L" else "since_r"
            if col_since in run:
                ob = onboard_edges(x, run[col_since][m])
                if ob and edges:
                    diff = [min(abs(o - e) for e in edges) for o in ob]
                    say(f"     onboard DiagEdge: {len(ob)} edges, max |diff| to this detection {max(diff):.1f} mm")
                elif ob or edges:
                    say(f"     onboard DiagEdge: {len(ob)} edges (thresholds differ from config::diag?)")
        run_ph = {s: [p for f, p in phases[s] if f == run["name"]] for s in SIDES}
        if run_ph["L"] and run_ph["R"]:
            pl, pr = statistics.fmean(run_ph["L"]), statistics.fmean(run_ph["R"])
            along, lat = (pl + pr) / 2.0, (pl - pr) / 2.0
            decomp.append((run["name"], run["dir"], along, lat))
            say(f"  along {along:+.1f} mm (mean of L and R phases), lateral {lat:+.1f} mm (left +, (L - R) / 2)")
        for col, side in SENSORS:
            pooled[col][0].append(since[side])
            pooled[col][1].append(run[col][m])

    say("")
    say("edge phase from the pillar on the same side (all runs; consistent phase = usable reference):")
    for side in SIDES:
        ps = [p for _, p in phases[side]]
        if len(ps) >= 2:
            say(f"  {side}: mean {statistics.fmean(ps):+.1f} mm, std {statistics.pstdev(ps):.1f} mm, "
                f"range {min(ps):+.1f} .. {max(ps):+.1f} ({len(ps)} edges)")
        elif ps:
            say(f"  {side}: {ps[0]:+.1f} mm (1 edge)")
        else:
            say(f"  {side}: no edges (check the thresholds with --on/--off or the walls)")
    # 横に d（左が正）ずれると，北（右へ入ったとき）を向く左のビームは柱を d 遅れて，右は d 早く過ぎる。
    # 前後のずれ（距離の誤差）は左右とも同じ向きに動かす。左右の位相の差の半分が横，平均が前後
    if decomp:
        say("decomposed per run (lateral: left +; the inner side is the turn direction):")
        alongs = [a for _, _, a, _ in decomp]
        say(f"  along: mean {statistics.fmean(alongs):+.1f} mm, std {statistics.pstdev(alongs):.1f} mm ({len(alongs)} runs)"
            "  <- the edge position itself; small std = usable reference")
        for d in SIDES:
            lats = [l for _, t, _, l in decomp if t == d]
            if lats:
                inward = statistics.fmean(lats) * (1.0 if d == "L" else -1.0)
                say(f"  turn {d}: lateral mean {statistics.fmean(lats):+.1f} mm (std {statistics.pstdev(lats):.1f}),"
                    f" {inward:+.1f} mm toward the inner side ({len(lats)} runs)")
        # 置き方の前後のずれ（後壁との隙間など）を除く：機体が北へ n ずれると，左右どちらへ入っても前後は +n/√2，
        # 内側へは −n/√2。前後は（センサーの取り付けで決まる）全走行の平均に戻るはずなので，その差を内側に足し戻す。
        # 置き方の東西のずれは前後と内側を同じ向きに動かすので除けない
        ref = statistics.fmean(alongs)
        for d in SIDES:
            corr = [(l if d == "L" else -l) + (a - ref) for _, t, a, l in decomp if t == d]
            if corr:
                say(f"  turn {d}: {statistics.fmean(corr):+.1f} mm toward the inner side with the placement shift along "
                    f"the start column removed (std {statistics.pstdev(corr):.1f})  <- the IN45 exit")

    say("")
    say("suggested thresholds (5%..95% of the side sensor while moving; OFF at 40%, ON at 60%):")
    suggest = {}
    for side in SIDES:
        if raw_side[side]:
            s = suggest_thresholds(np.concatenate(raw_side[side]))
            suggest[side] = s
            if s:
                say(f"  {side}: low {s['low']:.0f}, high {s['high']:.0f} -> THRESH_OFF {s['off']:.0f}, THRESH_ON {s['on']:.0f}"
                    f"   (now OFF {cfg['off'][side]:.0f} / ON {cfg['on'][side]:.0f})")

    tables = {}
    for col, side in SENSORS:
        if not pooled[col][0]:
            continue
        since_all = np.concatenate(pooled[col][0])
        value_all = np.concatenate(pooled[col][1])
        c, mean, std, n = make_table(since_all, value_all, bin_mm)
        bad = unstable_bins(c, n, min_n, guard_after, guard_before)
        tables[col] = {"edge_side": side, "since": c, "mean": mean, "std": std, "n": n, "unstable": bad,
                       "samples": (since_all, value_all)}
        say("")
        good = ~bad & (n >= min_n)
        say(f"table {col} vs distance since the {side} edge: {int(np.sum(n > 0))} bins with data, "
            f"median std {np.median(std[n >= min_n]) if np.any(n >= min_n) else math.nan:.1f}")
        un = ranges(c, bad, bin_mm)
        say("  do not use: " + (", ".join(f"{a:.0f}-{b:.0f} mm" for a, b in un) if un else "none"))
        step = max(1, int(round(10.0 / bin_mm)))
        line = []
        for i in range(0, len(c), step):
            if n[i] > 0:
                line.append(f"{c[i]:.0f}:{mean[i]:.0f}{'' if good[i] else '*'}")
        say("  " + " ".join(line) + "   (* = do not use)")
    return {"phases": phases, "decomp": decomp, "suggest": suggest, "tables": tables}


def write_json(result, cfg, bin_mm, path):
    out = {
        "period_mm": PERIOD,
        "bin_mm": bin_mm,
        "thresholds": cfg,
        "suggested_thresholds": result["suggest"],
        "edge_phase": {s: [p for _, p in result["phases"][s]] for s in SIDES},
        "runs": [{"file": f, "turn": t, "along_mm": round(a, 2), "lateral_mm": round(l, 2)}
                 for f, t, a, l in result["decomp"]],
        "tables": {},
    }
    for col, t in result["tables"].items():
        out["tables"][col] = {
            "edge_side": t["edge_side"],
            "since": [round(float(v), 2) for v in t["since"]],
            "mean": [None if math.isnan(v) else round(float(v), 1) for v in t["mean"]],
            "std": [None if math.isnan(v) else round(float(v), 1) for v in t["std"]],
            "n": [int(v) for v in t["n"]],
            "use": [bool(not u) for u in t["unstable"]],
        }
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w") as f:
        json.dump(out, f, indent=1)
    print(f"\nwrote {path}")


def plot(result, runs):
    import matplotlib.pyplot as plt
    tables = result["tables"]
    fig, axes = plt.subplots(len(tables) + 1, 1, figsize=(10, 3 * (len(tables) + 1)))
    ax = axes[0]
    for run in runs:
        m = moving_mask(run)
        for col, _ in SENSORS:
            ax.plot(run["diag_x"][m], run[col][m], lw=0.8, label=f"{run['name']} {col}")
    for k in range(-1, 9):
        ax.axvline(-PITCH / 2.0 + k * PITCH, color="0.8", lw=0.6)
    ax.set_xlabel("diag_x [mm] (grey: pillars, alternating inner / outer)")
    ax.set_ylabel("value")
    ax.legend(fontsize=6, ncol=2)
    for ax, (col, t) in zip(axes[1:], tables.items()):
        s, v = t["samples"]
        ax.plot(s, v, ".", ms=1.5, color="0.6")
        ax.plot(t["since"], t["mean"], "-", color="C0", label="mean")
        ax.fill_between(t["since"], t["mean"] - t["std"], t["mean"] + t["std"], color="C0", alpha=0.2)
        for a, b in ranges(t["since"], t["unstable"], t["since"][1] - t["since"][0]):
            ax.axvspan(a, b, color="C3", alpha=0.12)
        ax.set_xlim(0, PERIOD)
        ax.set_ylabel(col)
        ax.set_xlabel(f"distance since the {t['edge_side']} edge [mm] (red: do not use)")
    fig.tight_layout()
    plt.show()


# ---------------------------------------------------------------- 合成データ（--selftest）

def _segments_sawtooth(turn, n_cells):
    """入45°の後の斜めの直線の両側に，のこぎり状に壁を並べる（tools/DIAGONAL.md の並べ方 A）。
    座標は入口の区画中央を原点に，y が北，x が曲がる向き（右なら東）。柱は (90+180i, 90+180j)。
    区画 (i, j)（中心 (180i, 180j)）を (0,1) → (1,1) → (1,2) → (2,2) … の順に通り，通らない2辺を壁にする"""
    walls = []
    cells = [(0, 1)]
    i, j = 0, 1
    for k in range(n_cells):
        if k % 2 == 0:
            i += 1
        else:
            j += 1
        cells.append((i, j))
    for idx, (ci, cj) in enumerate(cells):
        cx, cy = 180 * ci, 180 * cj
        x0, x1, y0, y1 = cx - 90, cx + 90, cy - 90, cy + 90
        if idx == 0:   # (0,1)：下から入り右へ出る → 左と上が壁
            walls += [((x0, y0), (x0, y1)), ((x0, y1), (x1, y1))]
        elif idx % 2 == 1:   # 左から入り上へ出る → 下と右が壁
            walls += [((x0, y0), (x1, y0)), ((x1, y0), (x1, y1))]
        else:   # 下から入り右へ出る → 左と上が壁
            walls += [((x0, y0), (x0, y1)), ((x0, y1), (x1, y1))]
    # 壁の厚さ（12 mm）：中心線の両側の面に分ける
    faces = []
    for (ax_, ay), (bx, by) in walls:
        if ax_ == bx:
            faces += [((ax_ - 6, ay - 6), (ax_ - 6, by + 6)), ((ax_ + 6, ay - 6), (ax_ + 6, by + 6))]
        else:
            faces += [((ax_ - 6, ay - 6), (bx + 6, ay - 6)), ((ax_ - 6, ay + 6), (bx + 6, ay + 6))]
    if turn == "L":   # 左へ曲がるときは x を反転
        faces = [((-a[0], a[1]), (-b[0], b[1])) for a, b in faces]
    return faces


def _ray(px, py, dx, dy, faces, max_d=400.0):
    best = max_d
    for (ax_, ay), (bx, by) in faces:
        ex, ey = bx - ax_, by - ay
        den = dx * ey - dy * ex
        if abs(den) < 1e-9:
            continue
        t = ((ax_ - px) * ey - (ay - py) * ex) / den
        u = ((ax_ - px) * dy - (ay - py) * dx) / den
        if 0.0 < t < best and 0.0 <= u <= 1.0:
            best = t
    return best


# 仮のセンサーの取り付け（車軸から前 f，左 l [mm]，向き [deg]，左が正）。値 = 30 + 1500·(50/d)²（上限 3000）
_MOUNTS = {"ir_l": (30.0, 20.0, 45.0), "ir_fl": (45.0, 12.0, 5.0), "ir_fr": (45.0, -12.0, -5.0), "ir_r": (30.0, -20.0, -45.0)}


def synth_run(turn, n_half, x_err=0.0, lat=0.0, noise=0.0, rng=None, step=1.0):
    """理想の走行（向きのずれなし）のログを作る。x_err は実測の距離のずれ，lat は横のずれ（左が正）"""
    faces = _segments_sawtooth(turn, n_half + 2)
    sgn = 1.0 if turn == "R" else -1.0
    hx, hy = sgn / math.sqrt(2.0), 1.0 / math.sqrt(2.0)     # 進む向き
    lx, ly = -hy, hx                                         # 左（進む向きを左へ90°）
    ax0 = 90.0 * sgn
    cols = {k: [] for k in ["Global_time", "diag_x", "target_diag_x", "angle_error", "ir_l", "ir_fl", "ir_fr", "ir_r"]}
    u = -100.0
    t = 0.0
    while u <= n_half * PITCH:
        px = ax0 + u * hx + lat * lx
        py = 180.0 + u * hy + lat * ly
        for col, (f, l, a) in _MOUNTS.items():
            sx, sy = px + f * hx + l * lx, py + f * hy + l * ly
            ar = math.radians(a)
            dx, dy = math.cos(ar) * hx + math.sin(ar) * lx, math.cos(ar) * hy + math.sin(ar) * ly
            d = _ray(sx, sy, dx, dy, faces)
            v = min(3000.0, 30.0 + 1500.0 * (50.0 / d) ** 2)
            if noise > 0.0:
                v += rng.normal(0.0, noise)
            cols[col].append(v)
        cols["Global_time"].append(t)
        cols["diag_x"].append(u + x_err)
        cols["target_diag_x"].append(u)
        cols["angle_error"].append(0.0)
        u += step
        t += step / 500.0
    run = {k: np.asarray(v) for k, v in cols.items()}
    run["name"] = f"synth_{'left' if turn == 'L' else 'right'}"
    run["dir"] = turn
    return run


def selftest():
    rng = np.random.default_rng(1)
    failures = 0

    def check(ok, what):
        nonlocal failures
        print(f"  [{' ok ' if ok else 'FAIL'}] {what}")
        if not ok:
            failures += 1

    print("selftest: ray-cast model of a sawtooth diagonal (tools/DIAGONAL.md layout A)")
    clean = [synth_run("R", 8), synth_run("L", 8)]
    cfg0 = {"on": {"L": 0, "R": 0}, "off": {"L": 0, "R": 0}, "min_wall": 10.0}
    # 閾値は合成データの提案値を使う
    sug = analyze(clean, {**cfg0, "on": {"L": 1e9, "R": 1e9}, "off": {"L": -1, "R": -1}}, quiet=True)["suggest"]
    cfg = {"on": {s: sug[s]["on"] for s in SIDES}, "off": {s: sug[s]["off"] for s in SIDES}, "min_wall": 10.0}
    res = analyze(clean, cfg, quiet=True)
    for side in SIDES:
        ps = [p for _, p in res["phases"][side]]
        check(len(ps) >= 6, f"{side}: edges found on both turn directions ({len(ps)})")
        check(len(ps) > 0 and statistics.pstdev(ps) < 0.5,
              f"{side}: edge phase from its pillar is the same for left and right turns (std {statistics.pstdev(ps) if ps else math.nan:.2f} mm)")
    for run in clean:
        m = moving_mask(run)
        e = detect_edges(run["diag_x"][m], run["ir_l"][m], cfg["on"]["L"], cfg["off"]["L"], 10.0)
        check(len(e) >= 2 and abs(float(np.mean(np.diff(e))) - PERIOD) < 0.5,
              f"{run['name']}: left edges every {np.mean(np.diff(e)) if len(e) >= 2 else math.nan:.2f} mm (period {PERIOD:.2f})")

    # 距離のずれ（+15 mm）があっても，切れ目からの距離の表は同じ（位相だけ動く）
    shifted = [synth_run("R", 8, x_err=15.0)]
    res_s = analyze(shifted, cfg, quiet=True)
    ps = [p for _, p in res_s["phases"]["R"]]
    p_clean = statistics.fmean(p for _, p in res["phases"]["R"])
    check(abs(statistics.fmean(ps) - p_clean - 15.0) < 0.5, f"odometry +15 mm shifts the edge phase by {statistics.fmean(ps) - p_clean:+.2f} mm")
    t0, t1 = res["tables"]["ir_r"], res_s["tables"]["ir_r"]
    both = (t0["n"] > 0) & (t1["n"] > 0) & ~t0["unstable"]
    check(np.nanmax(np.abs(t0["mean"][both] - t1["mean"][both])) < 15.0,
          "the table vs distance since the edge does not depend on the odometry shift")

    # 横のずれ（左 +5 mm）は，安定な区間の値を変える（表で姿勢を読める）
    lat = [synth_run("R", 8, lat=5.0)]
    res_l = analyze(lat, cfg, quiet=True)
    tl = res_l["tables"]["ir_l"]
    tc = res["tables"]["ir_l"]
    both = (tl["n"] > 0) & (tc["n"] > 0) & ~tc["unstable"]
    diff = float(np.nanmedian(tl["mean"][both] - tc["mean"][both]))
    check(diff > 0.0, f"5 mm to the left raises ir_l in the stable bins (median {diff:+.0f})")
    lat0 = res["decomp"][0][3]
    lat5 = res_l["decomp"][0][3]
    check(abs(lat5 - lat0 - 5.0) < 1.0, f"5 mm to the left is decomposed as lateral {lat5 - lat0:+.2f} mm")
    along_s = res_s["decomp"][0][2] - res["decomp"][0][2]
    check(abs(along_s - 15.0) < 1.0, f"odometry +15 mm is decomposed as along {along_s:+.2f} mm")

    # ノイズ ±20（1σ）でも，ヒステリシスで切れ目は増えず，表のばらつきはノイズ程度
    noisy = [synth_run("R", 8, noise=20.0, rng=rng), synth_run("L", 8, noise=20.0, rng=rng)]
    res_n = analyze(noisy, cfg, quiet=True)
    for side in SIDES:
        n_clean = len(res["phases"][side])
        n_noisy = len(res_n["phases"][side])
        check(n_noisy == n_clean, f"{side}: noise does not add or drop edges ({n_noisy} vs {n_clean})")
    t = res_n["tables"]["ir_r"]
    good = ~t["unstable"] & (t["n"] >= 3)
    check(np.any(good) and float(np.median(t["std"][good])) < 30.0,
          f"stable bins have std close to the noise (median {float(np.median(t['std'][good])) if np.any(good) else math.nan:.1f})")
    check(np.any(t["unstable"]), "bins right after the edge are marked unstable")

    print("diag_sensor selftest: " + ("PASS" if failures == 0 else f"{failures} FAILED"))
    return failures == 0


# ---------------------------------------------------------------- main

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("files", nargs="*", help=f"logs (default {DEFAULT_GLOB})")
    ap.add_argument("--on", nargs=2, type=float, metavar=("LEFT", "RIGHT"), help="THRESH_ON (default config::diag)")
    ap.add_argument("--off", nargs=2, type=float, metavar=("LEFT", "RIGHT"), help="THRESH_OFF (default config::diag)")
    ap.add_argument("--bin", type=float, default=2.0, help="table bin [mm] (default 2)")
    ap.add_argument("--guard", nargs=2, type=float, default=(10.0, 15.0), metavar=("AFTER", "BEFORE"),
                    help="do not use the bins this close after / before an edge [mm] (default 10 15)")
    ap.add_argument("--out", default=os.path.join(TOOLS_DIR, "log", "diag", "reference.json"), help="table JSON")
    ap.add_argument("--plot", action="store_true", help="show the waveforms and the tables")
    ap.add_argument("--selftest", action="store_true", help="check the analysis on synthetic data (no robot)")
    args = ap.parse_args()

    if args.selftest:
        sys.exit(0 if selftest() else 1)

    # 既定では表のデータだけ（補正をかけた走行 *_ctrl / *_inj は，ファイルを指定したときだけ）
    files = args.files or sorted(f for f in glob.glob(DEFAULT_GLOB)
                                 if not f.endswith("reference.json") and not CONTROL_RUN.search(os.path.basename(f)))
    if not files:
        raise SystemExit(f"no logs ({DEFAULT_GLOB})")
    cfg = read_config()
    if args.on:
        cfg["on"] = {"L": args.on[0], "R": args.on[1]}
    if args.off:
        cfg["off"] = {"L": args.off[0], "R": args.off[1]}
    print(f"thresholds: ON L {cfg['on']['L']:.0f} R {cfg['on']['R']:.0f}, OFF L {cfg['off']['L']:.0f} R {cfg['off']['R']:.0f}, "
          f"min wall {cfg['min_wall']:.0f} mm")
    runs = [load(f) for f in files]
    result = analyze(runs, cfg, bin_mm=args.bin, guard_after=args.guard[0], guard_before=args.guard[1])
    control_summary(runs)
    if any(CONTROL_RUN.search(r["name"]) for r in runs):
        print("(control runs given: reference.json not written; the tables above include the corrections)")
    else:
        write_json(result, cfg, args.bin, args.out)
    if args.plot:
        plot(result, runs)


if __name__ == "__main__":
    main()
