#!/usr/bin/env python3
"""スラロームの横滑りの係数（c, K）を，実機の走行ログと定規で測った停止位置のずれから同定する。

スリップ角（速度の向きが機体の向きより外側へ遅れる角）のモデルはデザイナーと同じ：
    β = K·v·ω + c·ω/v        （v [m/s], ω [rad/s], c [m]）
横すべりの速さは v·β = (K·v² + c)·ω なので，1つの速度の走行からは c_eff = c + K·v² しか決まらない。
2つ以上の速度の走行があれば c（切片）と K（v² に対する傾き）に分けられる。

やり方：ログの実際の動き（ジャイロの角度・角速度，エンコーダの距離・速度）を積分して止まった位置を求め，
そこへモデルの横すべりを足したものが測ったずれに合うように最小二乗で c, K を決める。
ずれは c, K にほぼ比例するので初期値はいらない（設計に使った K, c にもよらない）。

測定の記録（--measurements，既定 tools/slip_measurements.json）:

    {
      "runs": [
        {"log": "slalom/L90_500_right.csv", "outward": 10, "longitudinal": -8,
         "note": "BACK_TO_AXLE_MM=42"},
        ...
      ]
    }

  - log … tools/log からのパス。ファイル名（<種類>_<速度>[_FAN]_<left|right>[_n].csv）からターン・速度・ファン・向きを読む
  - outward … 区画中央（目標の停止位置）からの，出口方向に直交するずれ [mm]。ターンの外側（入口と反対側）が正
  - longitudinal … 出口方向のずれ [mm]。目標より先が正（足りなければ負）
  - back_to_axle_mm … 走ったときの config::mouse::BACK_TO_AXLE_MM（省略すると今の mouse_config.hpp の値）
  - note … 任意

ファンON/OFFは別々に同定する（滑りがファンの有無で変わるため）。結果の c, K をデザイナーの
「滑り係数 c(mm)」「滑り係数 K」に入れ，pre/post_offset を設計し直す（目安も表示する）。

使い方:
    python3 tools/identify_slip.py
    python3 tools/identify_slip.py --measurements my.json --log-dir path/to/tools/log
"""
import argparse
import csv
import json
import math
import os
import re
import sys

from slalom_presets import PRESET_LIST, parse_speed_key

TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_DIR = os.path.dirname(TOOLS_DIR)

CELL_MM = 180.0
WALL_HALF_MM = 6.0
V_FLOOR = 50.0   # [mm/s] これより遅いときは横すべりを足さない（c·ω/v が発散しないように）

# 試験（Core/Src/test/slalom_test.cpp）の幾何。右旋回，x右・y前，区画の後ろの境界が y=0，機体の中心線が x=0。
# 入口から出口の基準点までの変位（action の折れ線）は slalom_presets.py の exit_offset
EXIT_DISPLACEMENT = {p.cpp_name: p.exit_offset for p in PRESET_LIST if p.exit_offset is not None}

LOG_NAME = re.compile(r"^(?P<turn>[A-Z0-9]+)_(?P<speed>\d+(?:p\d+)?)(?P<fan>_FAN)?_(?P<dir>left|right)(?:_\d+)?\.csv$")


class IdentError(Exception):
    pass


def read_back_to_axle():
    path = os.path.join(REPO_DIR, "Core", "Inc", "config", "mouse_config.hpp")
    with open(path, encoding="utf-8") as f:
        m = re.search(r"BACK_TO_AXLE_MM\s*=\s*([0-9.]+)f", f.read())
    if not m:
        raise IdentError(f"{path} に BACK_TO_AXLE_MM が見つかりません")
    return float(m.group(1))


def load_log(path):
    with open(path, newline="") as f:
        rows = list(csv.reader(f))
    header, data = rows[0], [r for r in rows[1:] if r]
    cols = {name: [float(r[i]) for r in data] for i, name in enumerate(header)}
    need = ["Global_time", "current_angle", "gyro_z", "encoder_velocity_x", "current_distance_x", "target_omega"]
    missing = [n for n in need if n not in cols]
    if missing:
        raise IdentError(f"{path}: 列がありません: {', '.join(missing)}（スラロームの試験のログか確認）")
    return cols


class Run:
    """1回の走行。ログを右旋回の座標にそろえて持ち，c, K を与えたときの止まった位置のずれを計算する"""

    def __init__(self, record, log_dir, default_b):
        self.record = record
        name = os.path.basename(record["log"])
        m = LOG_NAME.match(name)
        if not m:
            raise IdentError(f"{record['log']}: ファイル名から種類・速度・向きを読めません（例 L90_500_right.csv）")
        self.turn = m.group("turn")
        if self.turn not in EXIT_DISPLACEMENT:
            raise IdentError(f"{record['log']}: {self.turn} は未対応（S90 / L90 / T180）")
        self.preset = next(p for p in PRESET_LIST if p.cpp_name == self.turn)
        self.speed = float(m.group("speed").replace("p", "."))
        self.fan = m.group("fan") is not None
        self.dir = m.group("dir")
        self.name = name[:-4]
        self.measured = (float(record["outward"]), float(record["longitudinal"]))
        self.back_to_axle = float(record.get("back_to_axle_mm", default_b))

        d = load_log(os.path.join(log_dir, record["log"]))
        s = -1.0 if self.dir == "right" else 1.0   # 右旋回で時計回りが正になるようにそろえる（左旋回は鏡写し）
        self.t = d["Global_time"]
        self.heading = [s * a for a in d["current_angle"]]          # [deg] 時計回りが正
        self.omega = [s * w for w in d["gyro_z"]]                   # [dps] 時計回りが正
        self.v = d["encoder_velocity_x"]                            # [mm/s]
        self.dist = d["current_distance_x"]                         # [mm]
        # ログの最後でまだ動いていたら，ログが一杯になって止まる前に記録が終わっている（止まった位置が無い）
        tail = self.v[-10:]
        self.truncated = sum(abs(v) for v in tail) / len(tail) > 5.0

        entry_y = CELL_MM if self.preset.entry == "edge" else 1.5 * CELL_MM
        dx, dy = EXIT_DISPLACEMENT[self.turn]
        h = math.radians(self.preset.angle)
        self.u = (math.sin(h), math.cos(h))     # 出口の向き
        self.n = (-self.u[1], self.u[0])        # 出口の外側（入口と反対側）
        stop = CELL_MM / 2 if self.preset.exit == "edge" else CELL_MM
        self.stop_point = (dx + stop * self.u[0], entry_y + dy + stop * self.u[1])

    def offsets(self, c_mm=0.0, k=0.0):
        """c [mm], K のモデルの横すべりを足して積分した，目標の停止位置からのずれ（外側, 前後）[mm]"""
        x, y = 0.0, WALL_HALF_MM + self.back_to_axle
        for i in range(1, len(self.t)):
            ds = self.dist[i] - self.dist[i - 1]
            h = math.radians(0.5 * (self.heading[i] + self.heading[i - 1]))
            v = self.v[i]
            if v > V_FLOOR:
                w = math.radians(self.omega[i])
                v_m = v / 1000.0
                h -= k * v_m * w + (c_mm / 1000.0) * w / v_m   # 速度の向きは機体の向きより β だけ外側へ遅れる
            x += ds * math.sin(h)
            y += ds * math.cos(h)
        ex, ey = x - self.stop_point[0], y - self.stop_point[1]
        return ex * self.n[0] + ey * self.n[1], ex * self.u[0] + ey * self.u[1]


def solve_least_squares(rows, rhs):
    """rows: 各式の係数のリスト，rhs: 右辺。正規方程式で解く（未知数は1〜2個）"""
    n = len(rows[0])
    a = [[sum(r[i] * r[j] for r in rows) for j in range(n)] for i in range(n)]
    b = [sum(r[i] * y for r, y in zip(rows, rhs)) for i in range(n)]
    if n == 1:
        return [b[0] / a[0][0]]
    det = a[0][0] * a[1][1] - a[0][1] * a[1][0]
    if abs(det) < 1e-12 * max(1.0, abs(a[0][0] * a[1][1])):
        return None
    return [(b[0] * a[1][1] - b[1] * a[0][1]) / det, (a[0][0] * b[1] - a[1][0] * b[0]) / det]


def fit(runs, use_k):
    """c [mm]（と K）を最小二乗で決める。ずれは c, K にほぼ比例するので，1次の近似で解いてから1回だけ線形化し直す"""
    c, k = 0.0, 0.0
    for _ in range(2):
        rows, rhs = [], []
        for r in runs:
            base = r.offsets(c, k)
            dc = [(a - b) for a, b in zip(r.offsets(c + 1.0, k), base)]                     # 1mmあたり
            dk = [(a - b) / 0.01 for a, b in zip(r.offsets(c, k + 0.01), base)] if use_k else None
            for j in range(2):   # 外側・前後の2式
                rows.append([dc[j]] + ([dk[j]] if use_k else []))
                rhs.append(r.measured[j] - base[j])
        sol = solve_least_squares(rows, rhs)
        if sol is None:
            return None
        c += sol[0]
        k += sol[1] if use_k else 0.0
    res = [(m - o) for r in runs for m, o in zip(r.measured, r.offsets(c, k))]
    rms = math.sqrt(sum(e * e for e in res) / len(res))
    return c, k, rms


def simulate_exit_shift(entry, angle, c_mm, k):
    """デザイナーと同じ計算（1ms刻み，設計値の角速度の台形）で，滑りによる出口の基準点のずれ（外側, 前後）[mm]"""
    v = entry["Set_Speed"]
    w, a = entry["Set_low_AngVel"], entry["Set_Low_AngAcl"]
    ramp = w * w / (2 * a)
    tr, tc = w / a, (angle - 2 * ramp) / w
    dt = 0.001

    def end(c_val, k_val):
        x = y = th = t = 0.0
        while t < 2 * tr + tc:
            om = a * t if t < tr else (w if t < tr + tc else max(0.0, w - a * (t - tr - tc)))
            th += om * dt
            om_r, v_m = math.radians(om), v / 1000.0
            beta = math.degrees(k_val * v_m * om_r + (c_val / 1000.0) * om_r / v_m)
            x += v * dt * math.sin(math.radians(th - beta))
            y += v * dt * math.cos(math.radians(th - beta))
            t += dt
        return x, y

    x0, y0 = end(0.0, 0.0)
    x1, y1 = end(c_mm, k)
    h = math.radians(angle)
    u, n = (math.sin(h), math.cos(h)), (-math.cos(h), math.sin(h))
    ex, ey = x1 - x0, y1 - y0
    return ex * n[0] + ey * n[1], ex * u[0] + ey * u[1]


def suggest(params, fan, c_mm, k):
    print(f"  設計し直すときの目安（slalom_params.json のファン{'ON' if fan else 'OFF'}のパラメータ，c={c_mm:.1f}mm, K={k:.4f}）")
    print("    滑りで出口の基準点がずれる量と，それを打ち消す pre/post_offset の1次の目安。最後はデザイナーで軌跡を確認する")
    for p in PRESET_LIST:
        for key, e in sorted(params.get(p.label, {}).items()):
            if parse_speed_key(key)[1] != fan or p.cpp_name not in EXIT_DISPLACEMENT:
                continue
            o, l = simulate_exit_shift(e, p.angle, c_mm, k)
            pre, post = e["Set_pri_offset"], e["Set_post_offset"]
            if p.angle == 180:
                # 出口が入口と逆向き：pre を増やすと出口が手前（前後が負），post を増やすと先へ。横ずれはオフセットでは消せない
                hint = f"post_offset {post:g} → {post - l:.1f}（外側 {o:+.1f}mm はオフセットでは直らない）"
            else:
                # 出口が入口と直交：pre は出口に対する横ずれ，post は前後だけを動かす
                hint = f"pre_offset {pre:g} → {pre - o:.1f}, post_offset {post:g} → {post - l:.1f}"
            print(f"    {p.cpp_name}_{key:<9} 滑りによるずれ 外側 {o:+5.1f} / 前後 {l:+5.1f} mm → {hint}")


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--measurements", default=os.path.join(TOOLS_DIR, "slip_measurements.json"))
    ap.add_argument("--log-dir", default=os.path.join(TOOLS_DIR, "log"))
    ap.add_argument("--params", default=os.path.join(TOOLS_DIR, "slalom_params.json"))
    args = ap.parse_args()

    try:
        with open(args.measurements, encoding="utf-8") as f:
            records = json.load(f)["runs"]
        default_b = read_back_to_axle()
        runs = [Run(r, args.log_dir, default_b) for r in records]
    except (OSError, KeyError, json.JSONDecodeError, IdentError) as e:
        print(f"identify_slip: エラー: {e}", file=sys.stderr)
        return 1
    params = {}
    if os.path.exists(args.params):
        with open(args.params, encoding="utf-8") as f:
            params = json.load(f)

    for fan in (False, True):
        group = [r for r in runs if r.fan == fan]
        if not group:
            continue
        print(f"===== ファン{'ON' if fan else 'OFF'}：{len(group)}本 =====")
        print("  走行                 実測 外側/前後   ログだけ(滑りなし)   この走行だけで合わせた c_eff")
        for r in group:
            if r.truncated:
                print(f"  ⚠ {r.name}: ログの最後でまだ動いている（{r.v[-1]:.0f}mm/s）。ログが途中で切れていて，"
                      "止まった位置を計算できないので使わない")
        group = [r for r in group if not r.truncated]
        if not group:
            continue
        for r in group:
            o0, l0 = r.offsets()
            single = fit([r], use_k=False)
            print(f"  {r.name:<20} {r.measured[0]:+5.1f}/{r.measured[1]:+5.1f}     {o0:+5.1f}/{l0:+5.1f}"
                  f"           {single[0]:5.1f} mm（残差RMS {single[2]:.1f}mm，B={r.back_to_axle:g}）")

        c_only = fit(group, use_k=False)
        print(f"  c だけ（K=0）  : c = {c_only[0]:.1f} mm，残差RMS {c_only[2]:.1f} mm")
        speeds = sorted({r.speed for r in group})
        best = c_only
        if len(speeds) >= 2:
            ck = fit(group, use_k=True)
            if ck is None:
                print("  c と K        : 分けられません（速度の違いが小さすぎる）")
            else:
                print(f"  c と K        : c = {ck[0]:.1f} mm, K = {ck[1]:.4f}，残差RMS {ck[2]:.1f} mm"
                      f"（速度 {', '.join(f'{s:g}' for s in speeds)} mm/s）")
                print("                  c_eff = c + K·v²：" + "，".join(
                    f"{s:g}mm/s で {ck[0] + ck[1] * (s / 1000) ** 2 * 1000:.1f}mm" for s in speeds))
                if ck[2] < c_only[2] - 0.5:
                    best = ck
                else:
                    print("                  K を足しても残差がほとんど減らないので，c だけ（K=0）を使う")
        else:
            print(f"  c と K        : 速度が1つ（{speeds[0]:g} mm/s）なので分けられない。K=0, c=c_eff として使う"
                  "（K を求めるには速度を変えた走行を足す）")
        print("  走行ごとの残差（実測 − モデル，外側/前後）：" + "，".join(
            f"{r.name} {r.measured[0] - o:+.1f}/{r.measured[1] - l:+.1f}" for r in group for o, l in [r.offsets(best[0], best[1])]))
        suggest(params, fan, best[0], best[1])
        print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
