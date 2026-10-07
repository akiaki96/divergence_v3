#!/usr/bin/env python3
"""最短走行のプリセットのヘッダ（config/run_presets.hpp）を生成する。

  run_presets.json … プリセット名 → ターンの速度・直線の最高速度・加減速度・斜めを使うか など（手で書く）

最短走行の経路（time_based_dijkstra）は、直線が続くところは区画中央から大回り90°（L90）・180°（T180）で、
1区画ずつ曲がるジグザグは区画の辺から小回り90°（S90）で曲がるので、この3つは必ず使う。"diagonal": true なら斜めのターン（IN45, OUT45, V90, IN135, OUT135）も使う。
ターンはどれも "turn_speed" のもの（config::slalom::<turn>_<speed>、"fan": true なら _FAN）を使い、
slalom_params.json にその速度・ファンの条件の設計がなければ生成をエラーで止める（ビルドが止まる）。

run_presets.json の形:

    {
      "500": {
        "turn_speed": 500,        … ターンの速度 [mm/s]（直線の始点・終点の速度）
        "max_speed": 1500,        … 縦横の直線の最高速度 [mm/s]
        "max_speed_dia": 1000,    … 斜めの直線の最高速度 [mm/s]（diagonal が false でも書く）
        "accel": 3000,            … 直線の加速度 [mm/s^2]
        "decel": 3000,            … 直線の減速度 [mm/s^2]
        "diagonal": false,        … 斜めの経路を使うか
        "fan": false,             … 任意（省略で false）。ファンを回して走るか
        "wall_edge": true,        … 任意（省略で false）。区画中央から入るターン（L90・T180・IN45・IN135）の前の直線で壁切れの補正をかけるか
        "note": ""                … 任意。ヘッダのコメントに出す
      }
    }

使い方:
    python3 tools/gen_run_presets.py                 # 標準出力へ
    python3 tools/gen_run_presets.py --out path.hpp  # ファイルへ
"""
import argparse
import os
import re
import sys

from gen_slalom_params import GenError, cpp_ident, fmt, load_json
from slalom_presets import PRESET_LIST, make_speed_key, slalom_key_order

TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))

REQUIRED_KEYS = ["turn_speed", "max_speed", "max_speed_dia", "accel", "decel", "diagonal"]
OPTIONAL_KEYS = ["fan", "wall_edge", "note"]

# RunPreset のターンの集合と、その並び（app/run_preset.hpp の RunTurns、app/search_preset.hpp の DiagonalTurns と同じ順）
ORTHO_TURNS = ["S90", "L90", "T180"]
DIAGONAL_TURNS = ["IN45", "OUT45", "V90", "IN135", "OUT135"]


def build_entries(presets, slalom_params):
    by_cpp_name = {p.cpp_name: p for p in PRESET_LIST}
    entries = []
    for name, preset in presets.items():
        where = f"run_presets.json の「{name}」"
        if not re.fullmatch(r"[0-9A-Za-z_]+", name):
            raise GenError(f"{where}: 名前は英数字と _ だけにしてください（定数名とログのファイル名に使う）")
        missing = [k for k in REQUIRED_KEYS if k not in preset]
        unknown = [k for k in preset if k not in REQUIRED_KEYS + OPTIONAL_KEYS]
        if missing or unknown:
            raise GenError(f"{where}: 足りないキー {missing}，知らないキー {unknown}")

        for key in ["turn_speed", "max_speed", "max_speed_dia", "accel", "decel"]:
            if float(preset[key]) <= 0.0:
                raise GenError(f"{where}: {key} は正の値にしてください")
        speed = float(preset["turn_speed"])
        for key in ["max_speed", "max_speed_dia"]:
            if float(preset[key]) < speed:
                raise GenError(f"{where}: {key} は turn_speed 以上にしてください（直線はターンの速度で入って出る）")

        fan = preset.get("fan", False)
        diagonal = preset["diagonal"]
        wall_edge = preset.get("wall_edge", False)
        for key, value in [("fan", fan), ("diagonal", diagonal), ("wall_edge", wall_edge)]:
            if not isinstance(value, bool):
                raise GenError(f"{where}: {key} は true / false です")
        condition = f"{speed:g}mm/s ファン{'ON' if fan else 'OFF'}"

        kinds = ORTHO_TURNS + (DIAGONAL_TURNS if diagonal else [])
        resolved = {}
        for cpp_name in kinds:
            turn = by_cpp_name[cpp_name]
            designed = slalom_params.get(turn.label, {})
            # 最短走行は基本の組（"500" / "500_fan"）を使う
            if make_speed_key(speed, fan) not in designed:
                raise GenError(f"{where}: {turn.label} の {condition} は slalom_params.json に設計されていません"
                               f"（設計済み: {', '.join(sorted(designed, key=slalom_key_order)) or 'なし'}）"
                               + ("。斜めを使わないなら \"diagonal\": false" if cpp_name in DIAGONAL_TURNS else ""))
            resolved[cpp_name] = (turn.label, cpp_ident(turn.cpp_name, speed, fan))

        entries.append({
            "name": name,
            "ident": f"P_{name}",
            "diag_ident": f"D_{name}" if diagonal else None,
            "turns": resolved,
            "fan": fan,
            "wall_edge": wall_edge,
            "values": preset,
        })
    if not entries:
        raise GenError("run_presets.json にプリセットがありません")
    # メニューは Fast → ファン（OFF / ON）→ プリセット なので，ファンOFFを先に並べる（同じファンの中は書いた順）
    entries.sort(key=lambda e: e["fan"])
    return entries


def turn_list(e, kinds):
    return "{" + ", ".join(f"&config::slalom::{e['turns'][k][1]}" for k in kinds) + "}"


def render(entries):
    out = [
        "// 自動生成ファイル：tools/gen_run_presets.py が tools/run_presets.json から生成する。",
        "// 編集しないこと（JSONを編集する）",
        "#pragma once",
        "",
        "#include <array>",
        '#include "app/run_preset.hpp"',
        '#include "config/slalom_params.hpp"',
        "",
        "namespace config::run {",
        "",
    ]
    for e in entries:
        v = e["values"]
        note = f"  メモ: {v['note']}" if v.get("note") else ""
        extra = (("，斜めあり" if e["diag_ident"] else "，斜めなし") + ("，ファンON" if e["fan"] else "")
                 + ("，壁切れ補正" if e["wall_edge"] else ""))
        out.append(f"// {e['name']}: ターン {v['turn_speed']:g}mm/s，直線 {v['max_speed']:g} / 斜め {v['max_speed_dia']:g}mm/s"
                   f"{extra}{note}")
        if e["diag_ident"]:
            out.append(f"inline constexpr DiagonalTurns {e['diag_ident']} = {turn_list(e, DIAGONAL_TURNS)};")
        out.append(
            f"inline constexpr RunPreset {e['ident']} = {{\"{e['name']}\", {fmt(v['turn_speed'])}, {fmt(v['max_speed'])}, "
            f"{fmt(v['max_speed_dia'])}, {fmt(v['accel'])}, {fmt(v['decel'])}, {turn_list(e, ORTHO_TURNS)}, "
            f"{'&' + e['diag_ident'] if e['diag_ident'] else 'nullptr'}, {'true' if e['fan'] else 'false'}, "
            f"{'true' if e['wall_edge'] else 'false'}}};")
        out.append("")
    out.append("// メニューに並べる順（ファンOFFが先，同じファンの中は run_presets.json に書いた順）")
    out.append(f"inline constexpr std::array<RunPreset, {len(entries)}> PRESETS = {{")
    out += [f"    {e['ident']}," for e in entries]
    out += ["};", ""]
    # メニューのファンの段：PRESETS[first]からcount個（並べ替え済みなので同じファンは連続している）
    fans = []
    for i, e in enumerate(entries):
        if fans and fans[-1][0] == e["fan"]:
            fans[-1][2] += 1
        else:
            fans.append([e["fan"], i, 1])
    out.append("// メニューのファンの段：PRESETS[first]からcount個（Run → Fast → ファン → プリセット）")
    out.append(f"inline constexpr std::array<::slalom::MenuGroup, {len(fans)}> FANS = {{{{")
    out += [f"    {{\"{'fan on' if fan else 'fan off'}\", {first}, {count}}}," for fan, first, count in fans]
    out += ["}};", "", "} // namespace config::run", ""]
    return "\n".join(out)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--presets", default=os.path.join(TOOLS_DIR, "run_presets.json"))
    ap.add_argument("--slalom-params", default=os.path.join(TOOLS_DIR, "slalom_params.json"))
    ap.add_argument("--out", help="出力先（省略で標準出力）")
    args = ap.parse_args()

    try:
        entries = build_entries(load_json(args.presets), load_json(args.slalom_params))
    except GenError as e:
        print(f"gen_run_presets: エラー: {e}", file=sys.stderr)
        return 1

    text = render(entries)
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
