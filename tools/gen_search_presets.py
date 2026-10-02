#!/usr/bin/env python3
"""探索のプリセットのヘッダ（config/search_presets.hpp）を生成する。

  search_presets.json … プリセット名 → 探索速度・使うターンの集合・加速度など（手で書く）

"turns" には使うターンの種類（slalom_presets.py の cpp_name）を並べる。探索中でも既知区間では
大回りなどを使うので，複数のターンを持てる。種類ごとに SearchPreset の集合へ振り分ける：
  区画に沿ったターン（OrthoTurns）  … S90, L90, T180
  斜めのターン（DiagonalTurns）     … IN45, OUT45, V90, IN135, OUT135（1つでもあれば集合を作る）
ターンは並進速度を保ったまま曲がるので，どれも探索速度 "speed" のもの（config::slalom::<turn>_<speed>）を
使う。slalom_params.json にその速度の設計がなければ，生成をエラーで止める（ビルドが止まる）。

S90（小回り90°）は必須：未知区間は区画境界で壁を読んで1歩ずつ進むので，入口・出口とも区画境界の
ターンで曲がる。並べていない種類は nullptr になり，使わない。

壁を読む位置（区画境界の手前）はプリセットによらないので config::search::READ_LEAD_MM にある。

search_presets.json の形:

    {
      "500": {
        "speed": 500,                     … 探索速度 [mm/s]（= ターンの速度）
        "accel": 3000,                    … 直線の加速度・減速度 [mm/s^2]
        "turns": ["S90", "L90", "T180"],  … 使うターンの種類
        "pivot": {"omega": 360,           … 超信地旋回（行き止まりの180°）の最大角速度 [dps]
                  "alpha": 2500},         … 超信地旋回の角加速度 [dps/s]
        "wall_control": true,             … 任意（省略で false）。[実験中] 直進中に横壁で向きを補正するか
        "note": ""                        … 任意。ヘッダのコメントに出す
      }
    }

使い方:
    python3 tools/gen_search_presets.py                 # 標準出力へ
    python3 tools/gen_search_presets.py --out path.hpp  # ファイルへ
"""
import argparse
import json
import os
import re
import sys

from gen_slalom_params import GenError, cpp_ident, fmt, load_json
from slalom_presets import PRESET_LIST

TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))

REQUIRED_KEYS = ["speed", "accel", "turns", "pivot"]
OPTIONAL_KEYS = ["wall_control", "note"]
PIVOT_KEYS = ["omega", "alpha"]

# SearchPreset のターンの集合と，その並び（app/search_preset.hpp の OrthoTurns / DiagonalTurns と同じ順）
ORTHO_TURNS = ["S90", "L90", "T180"]
DIAGONAL_TURNS = ["IN45", "OUT45", "V90", "IN135", "OUT135"]
REQUIRED_TURN = "S90"


def build_entries(presets, slalom_params):
    by_cpp_name = {p.cpp_name: p for p in PRESET_LIST}
    entries = []
    for name, preset in presets.items():
        where = f"search_presets.json の「{name}」"
        if not re.fullmatch(r"[0-9A-Za-z_]+", name):
            raise GenError(f"{where}: 名前は英数字と _ だけにしてください（定数名とログのファイル名に使う）")
        missing = [k for k in REQUIRED_KEYS if k not in preset]
        unknown = [k for k in preset if k not in REQUIRED_KEYS + OPTIONAL_KEYS]
        if missing or unknown:
            raise GenError(f"{where}: 足りないキー {missing}，知らないキー {unknown}")

        for key in ["speed", "accel"]:
            if float(preset[key]) <= 0.0:
                raise GenError(f"{where}: {key} は正の値にしてください")
        speed = float(preset["speed"])

        pivot = preset["pivot"]
        if not isinstance(pivot, dict) or sorted(pivot) != sorted(PIVOT_KEYS):
            raise GenError(f"{where}: pivot は {{\"omega\": …, \"alpha\": …}} です")
        for key in PIVOT_KEYS:
            if float(pivot[key]) <= 0.0:
                raise GenError(f"{where}: pivot.{key} は正の値にしてください")

        wall_control = preset.get("wall_control", False)
        if not isinstance(wall_control, bool):
            raise GenError(f"{where}: wall_control は true / false です")

        turns = preset["turns"]
        if not isinstance(turns, list) or len(set(turns)) != len(turns):
            raise GenError(f"{where}: turns は重複のないターンの種類の配列です（例 [\"S90\", \"L90\"]）")
        if REQUIRED_TURN not in turns:
            raise GenError(f"{where}: turns に {REQUIRED_TURN} がありません（未知区間の1歩は小回り90°で曲がる）")
        resolved = {}
        for cpp_name in turns:
            turn = by_cpp_name.get(cpp_name)
            if turn is None:
                raise GenError(f"{where}: turns の「{cpp_name}」は slalom_presets.py にありません"
                               f"（使えるのは {', '.join(by_cpp_name)}）")
            designed = slalom_params.get(turn.label, {})
            speed_key = next((k for k in designed if float(k) == speed), None)
            if speed_key is None:
                raise GenError(f"{where}: {turn.label} の {speed:g}mm/s は slalom_params.json に設計されていません"
                               f"（設計済み: {', '.join(sorted(designed, key=float)) or 'なし'}）")
            resolved[cpp_name] = (turn.label, cpp_ident(turn.cpp_name, speed_key))
        s90 = by_cpp_name[REQUIRED_TURN]
        if s90.entry != "edge" or s90.exit != "edge":
            raise GenError(f"{REQUIRED_TURN} は入口・出口とも区画境界のはずです（slalom_presets.py を確認）")

        entries.append({
            "name": name,
            "ident": f"P_{name}",
            "diag_ident": f"D_{name}" if any(t in resolved for t in DIAGONAL_TURNS) else None,
            "turns": resolved,
            "pivot": pivot,
            "wall_control": wall_control,
            "values": preset,
        })
    if not entries:
        raise GenError("search_presets.json にプリセットがありません")
    return entries


def turn_list(e, kinds):
    refs = [f"&config::slalom::{e['turns'][k][1]}" if k in e["turns"] else "nullptr" for k in kinds]
    return "{" + ", ".join(refs) + "}"


def render(entries):
    out = [
        "// 自動生成ファイル：tools/gen_search_presets.py が tools/search_presets.json から生成する。",
        "// 編集しないこと（JSONを編集する）",
        "#pragma once",
        "",
        "#include <array>",
        '#include "app/search_preset.hpp"',
        '#include "config/slalom_params.hpp"',
        "",
        "namespace config::search {",
        "",
    ]
    for e in entries:
        v = e["values"]
        note = f"  メモ: {v['note']}" if v.get("note") else ""
        labels = "・".join(label for label, _ in e["turns"].values())
        wall = "，横壁の補正あり" if e["wall_control"] else ""
        out.append(f"// {e['name']}: {v['speed']:g}mm/s，ターンは{labels}{wall}{note}")
        if e["diag_ident"]:
            out.append(f"inline constexpr DiagonalTurns {e['diag_ident']} = {turn_list(e, DIAGONAL_TURNS)};")
        out.append(
            f"inline constexpr SearchPreset {e['ident']} = {{\"{e['name']}\", {fmt(v['speed'])}, {fmt(v['accel'])}, "
            f"{turn_list(e, ORTHO_TURNS)}, {'&' + e['diag_ident'] if e['diag_ident'] else 'nullptr'}, "
            f"{{{fmt(e['pivot']['omega'])}, {fmt(e['pivot']['alpha'])}}}, "
            f"{'true' if e['wall_control'] else 'false'}}};")
        out.append("")
    out.append("// メニューに並べる順（search_presets.json に書いた順）")
    out.append(f"inline constexpr std::array<SearchPreset, {len(entries)}> PRESETS = {{")
    out += [f"    {e['ident']}," for e in entries]
    out += ["};", "", "} // namespace config::search", ""]
    return "\n".join(out)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--presets", default=os.path.join(TOOLS_DIR, "search_presets.json"))
    ap.add_argument("--slalom-params", default=os.path.join(TOOLS_DIR, "slalom_params.json"))
    ap.add_argument("--out", help="出力先（省略で標準出力）")
    args = ap.parse_args()

    try:
        entries = build_entries(load_json(args.presets), load_json(args.slalom_params))
    except GenError as e:
        print(f"gen_search_presets: エラー: {e}", file=sys.stderr)
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
