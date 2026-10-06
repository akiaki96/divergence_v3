#!/usr/bin/env python3
"""スラロームの設計（slalom_params.json）を，デザイナーと同じ軌跡の計算（slalom_sim.py）で自動で合わせ直す。

  refit    … ω・α の形を保ったまま，出口のずれが0になるように合わせ直す（c・K を同定し直した後に使う）
               T180 以外 : pre/post_offset を解き直す（出口の位置はオフセットに対して1次なので厳密に解ける）
               T180      : 外側のずれはオフセットでは直らないので，加減速の角度の割合を保ったまま ω を変えて0にし，
                           pre は今の値のまま post で前後を直す
  optimize … (ω, 加減速の角度の割合 r) を探して，制約を満たす中で最短時間に近い（--slack 以内）もののうち α が最小のものを選ぶ
               制約: α ≤ --alpha-max（既定は mouse_config.hpp の MAX_ALPHA），ω ≤ --omega-max，
                     pre/post ≥ --min-offset，柱・壁までの余裕 ≥ --margin（slalom_sim.clearance）

既定は「今 → 新」の表を出すだけ。--write を付けたときだけ slalom_params.json に書く（新しい saved_at になるので，
slalom_tuning.json の差分の base_saved_at が合わなくなり，ビルドが止まる。差分を見直してから書き換える）。

使い方:
    python3 tools/slalom_autotune.py refit                          # 保存されている c・K のまま合わせ直す
    python3 tools/slalom_autotune.py refit --fan off --c 10.6 --k 0 # ファンOFFのエントリを新しい c・K で
    python3 tools/slalom_autotune.py optimize --turn S90 --speed 700 --fan off --margin 8
    python3 tools/slalom_autotune.py refit --fan off --c 10.6 --write
"""
import argparse
import datetime
import json
import math
import os
import re
import sys

import numpy as np

import slalom_sim as S
from slalom_presets import PRESET_LIST, parse_speed_key

TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))
PARAMS_PATH = os.path.join(TOOLS_DIR, "slalom_params.json")
TUNING_PATH = os.path.join(TOOLS_DIR, "slalom_tuning.json")
MOUSE_CONFIG = os.path.join(TOOLS_DIR, "..", "Core", "Inc", "config", "mouse_config.hpp")

TOL_MM = 0.1   # 書く前に確かめる出口のずれの上限


class AutotuneError(Exception):
    pass


def read_config(name, default):
    """mouse_config.hpp の `inline constexpr float <name> = <値>f;`"""
    try:
        with open(MOUSE_CONFIG, encoding="utf-8") as f:
            m = re.search(rf"\b{name}\s*=\s*([0-9.]+)f?\s*;", f.read())
        return float(m.group(1)) if m else default
    except OSError:
        return default


# ファームウェアの slalom::validate() が使う上限（config::control::DT_S, config::profile_limit::MAX_ALPHA*）
DT_S = read_config("DT_S", 0.001)
MAX_ALPHA = read_config("MAX_ALPHA", 20000.0)
MAX_ALPHA_DECEL = read_config("MAX_ALPHA_DECEL", 20000.0)


# ---- 1つの設計 ----

class Design:
    """ω・α・pre・post と，その評価（ずれ・余裕・時間）"""

    def __init__(self, preset, speed, omega, alpha, pre, post, c_mm, k, width):
        self.preset, self.speed = preset, speed
        self.omega, self.alpha, self.pre, self.post = omega, alpha, pre, post
        self.c_mm, self.k, self.width = c_mm, k, width
        self.traj = S.simulate(preset, speed, omega, alpha, pre, post, c_mm, k)
        self.err = S.exit_error(preset, self.traj.end_x, self.traj.end_y)
        self._clearance = None

    @property
    def clearance(self):
        if self._clearance is None:
            self._clearance = S.clearance(self.preset, self.traj, self.width)
        return self._clearance

    def firmware_problem(self):
        """ファームウェアの slalom::validate()（validateSegment）が弾く理由。通るなら None。
        0 でない区間は1制御周期（DT_S）以上かかること，等角速度の角度が負でないこと，α が上限以下であること"""
        if self.pre < 0 or self.post < 0:
            return "オフセットが負"
        for name, dist in (("pre_offset", self.pre), ("post_offset", self.post)):
            if dist != 0.0 and dist / self.speed < DT_S * 1.01:
                return f"{name} {dist:g}mm が短すぎる（1制御周期 {self.speed * DT_S:.2f}mm 未満）"
        if self.alpha > min(MAX_ALPHA, MAX_ALPHA_DECEL):
            return f"α が MAX_ALPHA {min(MAX_ALPHA, MAX_ALPHA_DECEL):g} を超える"
        ramp = self.omega ** 2 / (2.0 * self.alpha)
        cruise = self.preset.angle - 2.0 * ramp
        if cruise < 0.0:
            return "ω まで加速しきれない（加減速の角度 > 旋回角）"
        if self.omega / self.alpha < DT_S * 1.01:
            return "加減速が短すぎる（1制御周期未満）"
        # ちょうど0（区間を積まない）も float32 の誤差で短い区間になりうるので，常に1制御周期以上を求める
        if cruise / self.omega < DT_S * 1.01:
            return "等角速度の区間が短すぎる（1制御周期未満）"
        return None

    @property
    def err_max(self):
        return max(abs(self.err[0]), abs(self.err[1]))

    def row(self):
        return (f"ω {self.omega:6.0f}  α {self.alpha:6.0f}  pre {self.pre:6.1f}  post {self.post:6.1f}  "
                f"ずれ 外{self.err[0]:+5.2f} 前後{self.err[1]:+5.2f}  余裕 {self.clearance:5.1f}  {self.traj.time_ms:6.1f}ms")


def with_offsets(preset, speed, omega, alpha, c_mm, k, width, pre=None, digits=1):
    """出口のずれが0になるオフセットを付けた設計（オフセットは digits 桁に丸める）。T180 は pre を与える"""
    p, q, _ = S.solve_offsets(preset, speed, omega, alpha, c_mm, k, pre)
    # + 0.0 で -0.0 を 0.0 にする
    return Design(preset, speed, omega, alpha, round(p, digits) + 0.0, round(q, digits) + 0.0, c_mm, k, width)


def lateral_error(preset, speed, omega, ramp_frac, c_mm, k):
    """T180：ω と加減速の角度の割合から，オフセットでは直らない外側のずれ [mm]"""
    alpha = alpha_of(preset, omega, ramp_frac)
    return S.solve_offsets(preset, speed, omega, alpha, c_mm, k)[2]


def alpha_of(preset, omega, ramp_frac):
    """加減速の角度の合計が旋回角の ramp_frac になる α（ω²/(2α) = ramp_frac·angle/2）"""
    return omega * omega / (ramp_frac * preset.angle)


def ramp_frac_of(preset, omega, alpha):
    return omega * omega / (alpha * preset.angle)


def solve_t180_omega(preset, speed, ramp_frac, c_mm, k, lo=50.0, hi=3000.0):
    """T180 の外側のずれを0にする ω（二分法）。見つからなければ None"""
    grid = np.linspace(lo, hi, 60)
    vals = [lateral_error(preset, speed, w, ramp_frac, c_mm, k) for w in grid]
    for (w0, f0), (w1, f1) in zip(zip(grid, vals), zip(grid[1:], vals[1:])):
        if f0 == 0.0:
            return float(w0)
        if (f0 > 0) != (f1 > 0):
            a, b, fa = w0, w1, f0
            for _ in range(60):
                m = (a + b) / 2
                fm = lateral_error(preset, speed, m, ramp_frac, c_mm, k)
                if (fm > 0) == (fa > 0):
                    a, fa = m, fm
                else:
                    b = m
            return float((a + b) / 2)
    return None


# ---- refit ----

def refit(preset, entry, c_mm, k):
    speed, width = entry["Set_Speed"], entry.get("Set_Width", 86.0)
    omega, alpha = entry["Set_low_AngVel"], entry["Set_Low_AngAcl"]
    if not S.parallel(preset):
        return with_offsets(preset, speed, omega, alpha, c_mm, k, width)
    frac = ramp_frac_of(preset, omega, alpha)
    w = solve_t180_omega(preset, speed, frac, c_mm, k)
    if w is None:
        raise AutotuneError("外側のずれを0にする ω が見つかりません")
    w = round(w, 1)
    a = math.ceil(alpha_of(preset, w, frac))   # 切り上げ：加減速の角度が旋回角を超えないように
    return with_offsets(preset, speed, w, a, c_mm, k, width, pre=entry["Set_pri_offset"])


# ---- optimize ----

def candidates(preset, speed, c_mm, k, width, omegas, fracs, args):
    """(ω, r) の格子の各点で，出口のずれを0にした設計"""
    out = []
    for r in fracs:
        if S.parallel(preset):
            w = solve_t180_omega(preset, speed, r, c_mm, k)
            if w is None:
                continue
            # pre と post の配分は自由：時間は pre とともに増えるので，pre を小さい方から試す
            base = with_offsets(preset, speed, w, alpha_of(preset, w, r), c_mm, k, width, pre=0.0, digits=3)
            pre0 = max(args.min_offset, args.min_offset - base.post)
            for extra in np.arange(0.0, 80.0, 1.0):
                out.append(with_offsets(preset, speed, w, alpha_of(preset, w, r), c_mm, k, width, pre=pre0 + extra, digits=3))
        else:
            for w in omegas:
                out.append(with_offsets(preset, speed, w, alpha_of(preset, w, r), c_mm, k, width, digits=3))
    return out


def acceptable(d, args, alpha_max):
    """optimize の制約（余裕は重いので最後に見る）"""
    return (d.firmware_problem() is None and d.alpha <= alpha_max
            and (args.omega_max is None or d.omega <= args.omega_max)
            and d.pre >= args.min_offset and d.post >= args.min_offset and d.err_max <= TOL_MM
            and d.clearance >= args.margin)


def ranked(cands, args, alpha_max):
    """制約を満たす候補を選ぶ順に並べる：最短時間のものを見つけ，そこから --slack 以内に遅いだけの候補を α の小さい順に
    （最短時間だけだと α がいつも上限に張り付く）"""
    cheap = [d for d in cands if d.firmware_problem() is None and d.alpha <= alpha_max
             and (args.omega_max is None or d.omega <= args.omega_max)
             and d.pre >= args.min_offset and d.post >= args.min_offset]
    cheap.sort(key=lambda d: d.traj.total_dist)
    fastest = next((d for d in cheap if d.clearance >= args.margin), None)
    if fastest is None:
        return []
    limit = fastest.traj.total_dist * (1.0 + args.slack)
    pool = [d for d in cheap if d.traj.total_dist <= limit and d.clearance >= args.margin]
    return sorted(pool, key=lambda d: (d.alpha, d.traj.total_dist))


def rounded(preset, fine, c_mm, k):
    """保存する値に丸めた設計（ω は 1dps，α は 10dps/s で切り上げ＝加減速の角度が旋回角を超えない側，オフセットは 0.01mm）。
    丸めた ω・α でオフセットを解き直す。T180 は加減速の割合を保って ω を解き直す"""
    speed, width = fine.speed, fine.width
    w = round(fine.omega)
    a = math.ceil(fine.alpha / 10.0) * 10.0
    if not S.parallel(preset):
        return with_offsets(preset, speed, w, a, c_mm, k, width, digits=2)
    frac = ramp_frac_of(preset, w, a)
    w = solve_t180_omega(preset, speed, frac, c_mm, k)
    if w is None:
        return None
    w = round(w, 1)
    a = math.ceil(alpha_of(preset, w, frac))
    return with_offsets(preset, speed, w, a, c_mm, k, width, pre=round(fine.pre, 2), digits=2)


def optimize(preset, entry, c_mm, k, args, alpha_max):
    speed, width = entry["Set_Speed"], entry.get("Set_Width", 86.0)
    w_hi = args.omega_max or 2000.0
    # 加減速の割合 r は 1 未満（1 だと等角速度の区間が0になり，float の誤差で1制御周期未満の区間ができる）
    coarse = ranked(candidates(preset, speed, c_mm, k, width, np.arange(100.0, w_hi + 1, 20.0),
                               np.arange(0.1, 0.951, 0.05), args), args, alpha_max)
    if not coarse:
        raise AutotuneError("制約を満たす設計がありません（--margin / --min-offset / --alpha-max を見直す）")
    r0 = ramp_frac_of(preset, coarse[0].omega, coarse[0].alpha)
    fracs = np.clip(np.arange(r0 - 0.05, r0 + 0.0501, 0.01), 0.02, 0.97)
    omegas = np.arange(max(50.0, coarse[0].omega - 20), min(w_hi, coarse[0].omega + 20) + 0.5, 1.0)
    fine = ranked(candidates(preset, speed, c_mm, k, width, omegas, fracs, args), args, alpha_max)
    for d in fine + coarse:
        r = rounded(preset, d, c_mm, k)
        if r is not None and acceptable(r, args, alpha_max):
            return r
    raise AutotuneError("丸めた値で制約を満たす設計がありません（--margin / --min-offset を少し緩める）")


# ---- 入出力 ----

def load_json(path):
    if not os.path.exists(path):
        return {}
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def write_params(data):
    tmp = PARAMS_PATH + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)
    os.replace(tmp, PARAMS_PATH)


def selected(args, params):
    for p in PRESET_LIST:
        if args.turn and p.cpp_name not in args.turn:
            continue
        for key, e in sorted(params.get(p.label, {}).items(), key=lambda kv: parse_speed_key(kv[0])):
            speed, fan = parse_speed_key(key)
            if args.speed and speed not in args.speed:
                continue
            if args.fan != "all" and fan != (args.fan == "on"):
                continue
            yield p, key, e


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    ap.add_argument("mode", choices=["refit", "optimize"])
    ap.add_argument("--fan", choices=["off", "on", "all"], default="all")
    ap.add_argument("--c", type=float, help="滑り係数 c [mm]（省略で各エントリの保存値）。--fan off/on と一緒に使う")
    ap.add_argument("--k", type=float, help="滑り係数 K（省略で各エントリの保存値）")
    ap.add_argument("--turn", nargs="+", help="S90 L90 T180 IN45 OUT45 V90 IN135 OUT135 から（省略で全部）")
    ap.add_argument("--speed", nargs="+", type=float)
    ap.add_argument("--margin", type=float, default=8.0, help="optimize: 柱・壁までの余裕の下限 [mm]")
    ap.add_argument("--min-offset", type=float, default=0.0, help="optimize: pre/post_offset の下限 [mm]")
    ap.add_argument("--alpha-max", type=float, help="optimize: α の上限 [dps/s]（既定は MAX_ALPHA）")
    ap.add_argument("--slack", type=float, default=0.01,
                    help="optimize: 最短時間からこの割合だけ遅い候補まで含めて，α が最小のものを選ぶ（既定 0.01 = 1%%）")
    ap.add_argument("--omega-max", type=float, help="optimize: ω の上限 [dps]（既定 2000 まで探す）")
    ap.add_argument("--write", action="store_true", help="slalom_params.json に書く")
    args = ap.parse_args()

    if (args.c is not None or args.k is not None) and args.fan == "all":
        ap.error("--c / --k はファンの有無で違うので，--fan off か --fan on と一緒に指定する")
    alpha_max = min(args.alpha_max or MAX_ALPHA, MAX_ALPHA, MAX_ALPHA_DECEL)

    params = load_json(PARAMS_PATH)
    results = []
    for p, key, e in selected(args, params):
        c = e.get("Set_C_SP", 0.0) if args.c is None else args.c
        k = e.get("Set_K_SP", 0.0) if args.k is None else args.k
        width = e.get("Set_Width", 86.0)
        now = Design(p, e["Set_Speed"], e["Set_low_AngVel"], e["Set_Low_AngAcl"], e["Set_pri_offset"],
                     e["Set_post_offset"], c, k, width)
        print(f"{p.cpp_name}_{key}  c={c:g}mm K={k:g}")
        print(f"  今 {now.row()}")
        try:
            new = refit(p, e, c, k) if args.mode == "refit" else optimize(p, e, c, k, args, alpha_max)
        except AutotuneError as ex:
            print(f"  ✗ {ex}")
            continue
        problems = []
        if new.err_max > TOL_MM:
            problems.append(f"ずれが {TOL_MM}mm を超える")
        fw = new.firmware_problem()
        if fw:
            problems.append(f"ファームウェアの検査を通らない：{fw}（ω・α を変えないと合わないなら optimize を使う）")
        if new.clearance < 0:
            problems.append("柱・壁に当たる")
        if new.alpha > alpha_max:
            problems.append(f"α が上限 {alpha_max:g} を超える")
        print(f"  新 {new.row()}" + ("" if not problems else "  ✗ " + "，".join(problems)))
        if not problems:
            results.append((p, key, e, new))

    if not args.write:
        print(f"\n{len(results)} 件を書けます（--write で slalom_params.json に書く）")
        return 0
    if not results:
        print("書くものがありません")
        return 1

    data = load_json(PARAMS_PATH)   # 他で編集された分を消さないよう，書く直前に読み直す
    tuning = load_json(TUNING_PATH)
    now_s = datetime.datetime.now().isoformat(timespec="seconds")
    stale = []
    for p, key, e, d in results:
        entry = dict(data[p.label][key])
        entry.update({
            "Set_low_AngVel": float(d.omega), "Set_Low_AngAcl": float(d.alpha),
            "Set_pri_offset": float(d.pre), "Set_post_offset": float(d.post),
            "Set_K_SP": float(d.k), "Set_C_SP": float(d.c_mm),
            "result": {"acc_dist": d.traj.acc_dist, "const_dist": d.traj.const_dist,
                       "total_dist": d.traj.total_dist, "time_ms": d.traj.time_ms},
            "saved_at": now_s,
            "autotune": args.mode,
        })
        data[p.label][key] = entry
        if key in tuning.get(p.label, {}):
            stale.append(f"{p.label} / {key}")
    write_params(data)
    print(f"\n{len(results)} 件を slalom_params.json に書きました（saved_at {now_s}）")
    if stale:
        print("slalom_tuning.json の次の差分は古い設計に対するものです。見直して base_saved_at を書き換えるか消してください"
              "（そのままではビルドが止まります）:")
        for s in stale:
            print(f"  {s}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
