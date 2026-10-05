#!/usr/bin/env python3
"""前のセンサーの値 → 前壁までの距離の換算表のヘッダ（config/front_distance_table.hpp）を生成する。

  ir_calibration.json … tools/fit_ir.py（feature/wall-distance）が前壁スイープから作った換算表

JSON の sensors.ir_var_FL / ir_var_FR の lut（[値, 距離] の組，値の大きい順・距離の近い順）を，
有効範囲（valid.d_min〜d_max）に絞ってそのまま並べる。距離は「車軸から前の壁の面まで [mm]」。
ir_var_FL / ir_var_FR は device_instance.hpp の irFL / irFR（wall::front_left / front_right）。

CMake がビルドのたびに，JSON かこのスクリプトが変わっていれば再生成する。標準ライブラリだけで動く。

使い方:
    python3 tools/gen_front_distance.py                 # 標準出力へ
    python3 tools/gen_front_distance.py --out path.hpp  # ファイルへ
"""
import argparse
import os
import sys

from gen_slalom_params import GenError, fmt, load_json

TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))

# JSON のセンサー名 → ヘッダの配列名（wall::Position の前左・前右）
SENSORS = [("ir_var_FL", "FRONT_LEFT"), ("ir_var_FR", "FRONT_RIGHT")]


def build_table(calib, key):
    sensor = calib.get("sensors", {}).get(key)
    if sensor is None or "lut" not in sensor:
        raise GenError(f"ir_calibration.json に {key} の lut がありません")
    valid = sensor.get("valid", {})
    d_min = float(valid.get("d_min", float("-inf")))
    d_max = float(valid.get("d_max", float("inf")))
    points = [(float(v), float(d)) for v, d in sensor["lut"] if d_min <= float(d) <= d_max]
    if len(points) < 2:
        raise GenError(f"{key}: 有効範囲の点が2つ未満です")
    for (v0, d0), (v1, d1) in zip(points, points[1:]):
        if not (v0 >= v1 and d0 < d1):
            raise GenError(f"{key}: lut は値が減る（距離が増える）順のはずです（{v0},{d0} → {v1},{d1}）")
    return points


def render(calib, tables):
    out = [
        "// 自動生成ファイル：tools/gen_front_distance.py が tools/ir_calibration.json から生成する。",
        "// 編集しないこと（前壁スイープをやり直して JSON を作り直す）",
        f"// 換算表の作成: {calib.get('generated_at', '?')}，ログ {', '.join(calib.get('logs', []))}",
        "#pragma once",
        "",
        "namespace config::front_distance {",
        "",
        "// 値 → 車軸から前の壁の面までの距離 [mm]。値の大きい順（距離の近い順）",
        "struct Point {",
        "    float value;",
        "    float mm;",
        "};",
        "",
    ]
    for (key, name), points in zip(SENSORS, tables):
        out.append(f"// {key}: {points[0][1]:g}〜{points[-1][1]:g} mm，{len(points)} 点")
        out.append(f"inline constexpr Point {name}[] = {{")
        out += [f"    {{{fmt(v)}, {fmt(d)}}}," for v, d in points]
        out += ["};", ""]
    out += ["} // namespace config::front_distance", ""]
    return "\n".join(out)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--calib", default=os.path.join(TOOLS_DIR, "ir_calibration.json"))
    ap.add_argument("--out", help="出力先（省略で標準出力）")
    args = ap.parse_args()

    try:
        calib = load_json(args.calib)
        tables = [build_table(calib, key) for key, _ in SENSORS]
    except GenError as e:
        print(f"gen_front_distance: エラー: {e}", file=sys.stderr)
        return 1

    text = render(calib, tables)
    if args.out is None:
        sys.stdout.write(text)
        return 0
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    tmp = args.out + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        f.write(text)
    os.replace(tmp, args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
