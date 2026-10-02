#!/usr/bin/env python3
"""探索のプリセットのヘッダ（config/search_presets.hpp）を生成する。

  search_presets.json … プリセット名 → 探索速度・使うスラローム・加速度など（手で書く）

使うスラロームは "turn"（slalom_presets.py の cpp_name。例 "S90"）と "speed" から
config::slalom::<turn>_<speed>（例 S90_500）を選ぶ。そのスラロームが slalom_params.json に
設計されていなければ，生成をエラーで止める（ビルドが止まる）。

探索は区画境界で壁を読んで1歩ずつ進むので，ターンは入口・出口とも区画境界（"edge"）のもの
（小回り90°）でなければならない。スラロームは並進速度を保ったまま旋回するので，探索速度が
そのままスラロームの速度になる。

search_presets.json の形:

    {
      "500": {
        "turn": "S90",          … スラロームの種類（slalom_presets.py の cpp_name）
        "speed": 500,           … 探索速度 [mm/s]（= スラロームの速度）
        "accel": 3000,          … 直線の加速度・減速度 [mm/s^2]
        "read_lead_mm": 10,     … 区画境界の何mm手前で壁を読むか
        "pivot_omega": 360,     … 超信地旋回（行き止まりの180°）の最大角速度 [dps]
        "pivot_alpha": 2500,    … 超信地旋回の角加速度 [dps/s]
        "wall_control": true,   … 直進中に横壁で向きを補正するか
        "note": ""              … 任意。ヘッダのコメントに出す
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

REQUIRED_KEYS = ["turn", "speed", "accel", "read_lead_mm", "pivot_omega", "pivot_alpha", "wall_control"]
OPTIONAL_KEYS = ["note"]


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

        turn = by_cpp_name.get(preset["turn"])
        if turn is None:
            raise GenError(f"{where}: turn「{preset['turn']}」は slalom_presets.py にありません"
                           f"（使えるのは {', '.join(by_cpp_name)}）")
        if turn.entry != "edge" or turn.exit != "edge":
            raise GenError(f"{where}: 探索は区画境界で1歩ずつ進むので，入口・出口とも区画境界のターン"
                           f"（小回り90°）しか使えません（{turn.label} は {turn.entry} → {turn.exit}）")

        speed = float(preset["speed"])
        designed = slalom_params.get(turn.label, {})
        speed_key = next((k for k in designed if float(k) == speed), None)
        if speed_key is None:
            raise GenError(f"{where}: {turn.label} の {speed:g}mm/s は slalom_params.json に設計されていません"
                           f"（設計済み: {', '.join(sorted(designed, key=float)) or 'なし'}）")

        for key in ["speed", "accel", "read_lead_mm", "pivot_omega", "pivot_alpha"]:
            if float(preset[key]) <= 0.0:
                raise GenError(f"{where}: {key} は正の値にしてください")
        if not isinstance(preset["wall_control"], bool):
            raise GenError(f"{where}: wall_control は true / false です")

        entries.append({
            "name": name,
            "ident": f"P_{name}",
            "turn_ident": cpp_ident(turn.cpp_name, speed_key),
            "turn_label": turn.label,
            "values": preset,
        })
    if not entries:
        raise GenError("search_presets.json にプリセットがありません")
    return entries


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
        out.append(f"// {e['name']}: {v['speed']:g}mm/s，ターンは{e['turn_label']}（{e['turn_ident']}）{note}")
        out.append(
            f"inline constexpr SearchPreset {e['ident']} = {{\"{e['name']}\", {fmt(v['speed'])}, "
            f"{fmt(v['accel'])}, {fmt(v['read_lead_mm'])}, {fmt(v['pivot_omega'])}, {fmt(v['pivot_alpha'])}, "
            f"{'true' if v['wall_control'] else 'false'}, &config::slalom::{e['turn_ident']}}};")
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
