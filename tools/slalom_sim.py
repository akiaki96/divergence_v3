"""スラロームの軌跡の計算（slalom_profile_designer.py・slalom_autotune.py・identify_slip.py で共有する）。

座標と角度は slalom_presets.py の盤面と同じ（右旋回，角度はy軸から時計回り）。1回のスラロームは
  入口オフセット（直進）→ 角速度 0→ω（等角加速度）→ ωで等角速度 → ω→0 → 出口オフセット（直進）
で，並進速度は一定。1ms刻みで角度を進め，速度の向きは機体の向きより滑り角 β だけ外側へ遅れる：
  β = K·v·ω + c·ω/v   （v [m/s], ω [rad/s], c [m]）
  K … 横加速度 v·ω に比例する滑り（タイヤの横方向の弾性）
  c … 速度によらない横滑り（車軸の横すべり速度 c·ω。車軸より c 前の点を中心に回るのと同じ）

pygame に依存しない（numpy だけ）。
"""
import math
from dataclasses import dataclass

import numpy as np

DT = 0.001   # [s] 時間の刻み

# 盤面の壁と柱（クラシック迷路，1区画180mm）。壁は x = -90, 90, 270 / y = 0, 180, 360 の線上で，厚さ12mm
WALL_X = (-90.0, 90.0, 270.0)
WALL_Y = (0.0, 180.0, 360.0)
WALL_T = 12.0

# 軌跡のフェーズ（描画の色分けに使う）
PRE, ACCEL, CRUISE, DECEL, POST = range(5)


@dataclass
class Trajectory:
    xs: np.ndarray        # 重心の点列 [mm]（入口の基準点から出口オフセットの終点まで）
    ys: np.ndarray
    headings: np.ndarray  # 各点での機体の横幅の向きを決める角度 [deg]（旋回中は滑りを含む速度の向き）
    phases: np.ndarray    # 各点が属するフェーズ（PRE〜POST）
    end_x: float
    end_y: float
    end_angle: float      # [deg] 旋回を終えたときの機体の向き
    acc_dist: float       # [mm] 加速フェーズの距離
    const_dist: float     # [mm] 等角速度フェーズの距離
    total_dist: float     # [mm] 経路長（入口オフセット〜出口オフセット）
    time_ms: float


def slip_deg(omega_deg, speed, c_mm, k):
    """滑り角 β [deg]。omega_deg は [deg/s]（スカラーでも配列でもよい）"""
    v = speed / 1000.0
    if v <= 0.0:
        return 0.0 * omega_deg
    om = np.deg2rad(omega_deg)
    return np.rad2deg(k * v * om + (c_mm / 1000.0) * om / v)


def angle_profile(ini_angle, fin_angle, omega, alpha):
    """1msごとの (角速度 [deg/s], 機体の角度 [deg], フェーズ) の列と，加速・等角速度のステップ数。
    角速度を先に更新してから角度を進める（前進オイラー）。等角速度は「残りの角度が減速の角度になるまで」，
    減速は角速度が0になるまで。三角形（等角速度なし）も扱える"""
    oms, angs, phs = [], [], []
    w = 0.0
    a = ini_angle
    while w < omega:
        w = min(w + alpha * DT, omega)
        a += w * DT
        oms.append(w); angs.append(a); phs.append(ACCEL)
    n_acc = len(oms)
    while a < fin_angle - omega ** 2 / (2 * alpha):
        a += w * DT
        oms.append(w); angs.append(a); phs.append(CRUISE)
    n_const = len(oms) - n_acc
    while w > 0.0:
        w = max(w - alpha * DT, 0.0)
        a += w * DT
        oms.append(w); angs.append(a); phs.append(DECEL)
    return np.array(oms), np.array(angs), np.array(phs), n_acc, n_const


def _straight(x0, y0, heading, length, phase, step=1.0):
    """直進の点列（step [mm] ごと，始点を除き終点を含む）"""
    n = max(1, int(math.ceil(abs(length) / step))) if length != 0.0 else 0
    s = np.linspace(length / n, length, n) if n else np.zeros(0)
    h = math.radians(heading)
    return x0 + s * math.sin(h), y0 + s * math.cos(h), np.full(n, heading), np.full(n, phase)


def simulate(preset, speed, omega, alpha, pre, post, c_mm=0.0, k=0.0):
    """プリセット（slalom_presets.Preset）の入口の基準点から出口オフセットの終点までの軌跡"""
    oms, angs, phs, n_acc, n_const = angle_profile(preset.ini_angle, preset.fin_angle, omega, alpha)
    step = speed * DT
    dirs = np.deg2rad(angs - slip_deg(oms, speed, c_mm, k))   # 速度の向き

    h_in = math.radians(preset.ini_angle)
    sx = preset.ini_x + pre * math.sin(h_in)
    sy = preset.ini_y + pre * math.cos(h_in)
    tx = sx + np.cumsum(step * np.sin(dirs))
    ty = sy + np.cumsum(step * np.cos(dirs))
    turn_end_x = float(tx[-1]) if len(tx) else sx
    turn_end_y = float(ty[-1]) if len(ty) else sy

    px, py, ph, pp = _straight(preset.ini_x, preset.ini_y, preset.ini_angle, pre, PRE)
    qx, qy, qh, qp = _straight(turn_end_x, turn_end_y, preset.fin_angle, post, POST)
    xs = np.concatenate([[preset.ini_x], px, tx, qx])
    ys = np.concatenate([[preset.ini_y], py, ty, qy])
    headings = np.concatenate([[preset.ini_angle], ph, np.rad2deg(dirs), qh])
    phases = np.concatenate([[PRE], pp, phs, qp]).astype(int)

    h_out = math.radians(preset.fin_angle)
    end_x = turn_end_x + post * math.sin(h_out)
    end_y = turn_end_y + post * math.cos(h_out)
    total = abs(pre) + step * len(oms) + abs(post)
    return Trajectory(xs, ys, headings, phases, end_x, end_y, float(angs[-1]) if len(angs) else preset.ini_angle,
                      round(step * n_acc, 3), round(step * n_const, 3), round(total, 3),
                      round(total / speed * 1000.0, 1) if speed > 0 else 0.0)


# ---- 出口のずれとオフセットの解 ----

def _unit(deg):
    h = math.radians(deg)
    return np.array([math.sin(h), math.cos(h)])


def exit_frame(preset):
    """出口の向き u と外側 n（右旋回の左手＝入口と反対側）"""
    u = _unit(preset.fin_angle)
    return u, np.array([-u[1], u[0]])


def exit_error(preset, end_x, end_y):
    """出口オフセットの終点と出口の基準点の差を，(外側, 前後) [mm] に分けたもの。基準点がなければ None"""
    if preset.exit_offset is None:
        return None
    ex, ey = preset.ini_x + preset.exit_offset[0], preset.ini_y + preset.exit_offset[1]
    u, n = exit_frame(preset)
    d = np.array([end_x - ex, end_y - ey])
    return float(d @ n), float(d @ u)


def parallel(preset):
    """入口と出口の向きが平行（T180）：オフセットでは前後しか直せない"""
    return abs(math.sin(math.radians(preset.fin_angle - preset.ini_angle))) < 1e-9


def solve_offsets(preset, speed, omega, alpha, c_mm=0.0, k=0.0, pre=None):
    """出口のずれを0にする (pre, post, 残る外側のずれ)。
    出口の位置はオフセットに対して正確に1次（入口の向き e_in に pre，出口の向き e_out に post だけ動く）なので，
    旋回部分を1回計算すれば 2x2 の1次方程式で解ける。T180 のように平行なら pre を与え（省略すると0），
    post だけを解く。外側のずれは残る（ω・α で直す）"""
    t = simulate(preset, speed, omega, alpha, 0.0, 0.0, c_mm, k)
    target = np.array(preset.exit_offset, dtype=float)
    d = target - np.array([t.end_x - preset.ini_x, t.end_y - preset.ini_y])
    e_in, e_out = _unit(preset.ini_angle), _unit(preset.fin_angle)
    if not parallel(preset):
        m = np.column_stack([e_in, e_out])
        p_, q_ = np.linalg.solve(m, d)
        return float(p_), float(q_), 0.0
    pre = 0.0 if pre is None else pre
    # pre·e_in + post·e_out = d の e_out 成分（e_out = ±e_in）
    post = float(d @ e_out - pre * (e_in @ e_out))
    _, n = exit_frame(preset)
    return pre, post, float(-(d @ n))


# ---- 壁・柱までの余裕 ----

def _segments_cross(a, b, c, d, eps=1e-9):
    """線分 ab と cd が交わるか（端で触れるのも含む）"""
    def orient(o, p, q):
        v = (p[0] - o[0]) * (q[1] - o[1]) - (p[1] - o[1]) * (q[0] - o[0])
        return 0 if abs(v) < eps else (1 if v > 0 else -1)

    def on_segment(o, p, q):   # q が線分 op 上（同一直線上である前提）
        return (min(o[0], p[0]) - eps <= q[0] <= max(o[0], p[0]) + eps and
                min(o[1], p[1]) - eps <= q[1] <= max(o[1], p[1]) + eps)

    o1, o2, o3, o4 = orient(a, b, c), orient(a, b, d), orient(c, d, a), orient(c, d, b)
    if o1 != o2 and o3 != o4 and 0 not in (o1, o2, o3, o4):
        return True
    return ((o1 == 0 and on_segment(a, b, c)) or (o2 == 0 and on_segment(a, b, d)) or
            (o3 == 0 and on_segment(c, d, a)) or (o4 == 0 and on_segment(c, d, b)) or
            (o1 != o2 and o3 != o4))


def obstacles(preset):
    """ターンの近くにありうる柱と壁を (中心x, 中心y, 半幅x, 半幅y) の矩形で返す。
    壁は，基準の折れ線（preset.path）が通り抜ける区画境界にはなく，ほかの境界には（探索中は）ありうるとみなす"""
    h = WALL_T / 2.0
    rects = [(x, y, h, h) for x in WALL_X for y in WALL_Y]
    path = [(preset.ini_x + dx, preset.ini_y + dy) for dx, dy in preset.path]
    walls = []
    for x in WALL_X:
        for y0, y1 in zip(WALL_Y, WALL_Y[1:]):
            walls.append(((x, y0), (x, y1)))
    for y in WALL_Y:
        for x0, x1 in zip(WALL_X, WALL_X[1:]):
            walls.append(((x0, y), (x1, y)))
    for a, b in walls:
        if any(_segments_cross(p, q, a, b) for p, q in zip(path, path[1:])):
            continue   # 通り抜ける境界
        cx, cy = (a[0] + b[0]) / 2, (a[1] + b[1]) / 2
        half_len = math.hypot(b[0] - a[0], b[1] - a[1]) / 2 - h
        rects.append((cx, cy, half_len, h) if a[1] == b[1] else (cx, cy, h, half_len))
    return rects


def clearance(preset, traj, width):
    """機体の横幅（重心を通り進行方向に直交する長さ width の線分。デザイナーの灰色の線の間）と，
    柱・壁との最小距離 [mm]。負なら当たっている"""
    s = np.linspace(-0.5, 0.5, 9) * width
    h = np.deg2rad(traj.headings)
    # 横幅の向き（進行方向の右手）
    nx, ny = np.cos(h), -np.sin(h)
    px = traj.xs[:, None] + s[None, :] * nx[:, None]
    py = traj.ys[:, None] + s[None, :] * ny[:, None]
    best = math.inf
    for cx, cy, hx, hy in obstacles(preset):
        dx = np.maximum(np.abs(px - cx) - hx, 0.0)
        dy = np.maximum(np.abs(py - cy) - hy, 0.0)
        inside = (np.abs(px - cx) <= hx) & (np.abs(py - cy) <= hy)
        dist = np.where(inside, -1.0, np.hypot(dx, dy))
        best = min(best, float(dist.min()))
    return best
