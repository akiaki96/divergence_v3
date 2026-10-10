#!/usr/bin/env python3
"""探索のプリセットのヘッダ（config/search_presets.hpp）を生成する。

  search_presets.json … プリセット名 → 探索速度・使うターンの集合・加速度など（手で書く）
  run_presets.json    … "confirm" で名前を書いた最短走行のプリセットがあるかを確かめる

"turns" には使うターンの種類（slalom_presets.py の cpp_name）を並べる。探索中でも既知区間では
大回りなどを使うので，複数のターンを持てる。種類ごとに SearchPreset の集合へ振り分ける：
  区画に沿ったターン（OrthoTurns）  … S90, L90, T180
  斜めのターン（DiagonalTurns）     … IN45, OUT45, V90, IN135, OUT135（1つでもあれば集合を作る）
ターンは並進速度を保ったまま曲がるので，どれも探索速度 "speed" のものを使う。どの設計を使うかは
"slalom"（slalom_params.json の速度のキー＝スラロームの組）で選ぶ。省略すると "speed" と "fan" から決まる
（"500"，"fan": true なら "500_fan"）。同じ速度で別に保存した組（"500_b" など）を選べば，ターンと既知の直進の
速度・加速度の組み合わせを名前つきのプリセットとして並べて試せる。組の速度は "speed" と同じ，ファンは "fan"
（省略すると組のキーから決まる）と同じでなければならない。slalom_params.json にその組の設計がなければ，
生成をエラーで止める（ビルドが止まる）。

S90（小回り90°）は必須：未知区間は区画境界で壁を読んで1歩ずつ進むので，入口・出口とも区画境界の
ターンで曲がる。並べていない種類は nullptr になり，使わない。

壁を読む位置（区画境界の手前）はプリセットによらないので config::search::READ_LEAD_MM にある。

search_presets.json の形:

    {
      "500": {
        "speed": 500,                     … 探索速度 [mm/s]（= ターンの速度）
        "accel": 3000,                    … 直線の加速度・減速度 [mm/s^2]
        "straight_speed": 1000,           … 任意（省略で speed）。既知の区画が続く直進で加速する最高速度 [mm/s]
                                            （区画の左・前・右の壁がすべて分かっていて，ソルバーの答えが決まっている区間）
        "turns": ["S90", "L90", "T180"],  … 使うターンの種類
        "slalom": "500_b",                … 任意（省略で speed と fan から）。使うスラロームの組（slalom_params.json のキー）
        "pivot": {"omega": 360,           … 超信地旋回（行き止まりの180°）の最大角速度 [dps]
                  "alpha": 2500},         … 超信地旋回の角加速度 [dps/s]
        "fan": false,                     … 任意（省略で false）。ファンを回して走るか
        "wall_control": true,             … 任意（省略で false）。[実験中] 直進中に横壁で向きを補正するか
        "front_correction": true,         … 任意（省略で false）。[実験中] S90 の入口を前壁の距離で補正するか
                                            （false でも推定したずれはログに残す。config::front_correction）
        "goal": [7, 7],                   … 任意（省略で config::search::GOAL_X/Y）。ゴール区画 [x, y]。
                                            試験用に近いゴール（例 [1, 0]）で往復させるときに書く
        "one_way": false,                 … 任意（省略で false＝往復）。true ならゴールに着いたらそこで止まる（片道）
        "reset_walls": true,              … 任意（省略で true）。false なら壁を消さず，保存した最新の迷路を引き継いで探索する
        "confirm": "1200_dia",            … 任意（省略で使わない）。最短走行の経路で確かめる探索にする。値は最短走行の
                                            プリセット（run_presets.json のキー）で，経路計算のコスト（速度・ターン・斜め）に使う。
                                            ゴールに着いたら止まって，未知の壁を通れるとみなした最短走行の経路を求め，その経路の
                                            未知の壁を確かめに行く（崩れるかすべて分かったら，また止まって求め直す）。経路に未知の
                                            壁がなくなったらスタートへ戻る。one_way とは一緒に使えない
        "menu": "search",                 … 任意（省略で "search"）。並べるメニュー。
                                            "search" … Run → Search（config::search::PRESETS）
                                            "test"   … Test → Search（config::search::TEST_PRESETS）
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
from slalom_presets import PRESET_LIST, make_speed_key, parse_slalom_key, slalom_key_order

TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))

REQUIRED_KEYS = ["speed", "accel", "turns", "pivot"]
OPTIONAL_KEYS = ["straight_speed", "slalom", "fan", "wall_control", "front_correction", "goal", "one_way",
                 "reset_walls", "confirm", "menu", "note"]
# "menu" の値 → 生成する配列の名前（menu/menu.hpp がそれぞれのメニューに並べる）
MENU_ARRAYS = {"search": "PRESETS", "test": "TEST_PRESETS"}
MAZE_SIZE = 16
PIVOT_KEYS = ["omega", "alpha"]

# SearchPreset のターンの集合と，その並び（app/search_preset.hpp の OrthoTurns / DiagonalTurns と同じ順）
ORTHO_TURNS = ["S90", "L90", "T180"]
DIAGONAL_TURNS = ["IN45", "OUT45", "V90", "IN135", "OUT135"]
REQUIRED_TURN = "S90"


def build_entries(presets, slalom_params, run_presets):
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
        straight_speed = float(preset.get("straight_speed", speed))
        if straight_speed < speed:
            raise GenError(f"{where}: straight_speed は speed 以上にしてください（省略で speed＝加速しない）")

        pivot = preset["pivot"]
        if not isinstance(pivot, dict) or sorted(pivot) != sorted(PIVOT_KEYS):
            raise GenError(f"{where}: pivot は {{\"omega\": …, \"alpha\": …}} です")
        for key in PIVOT_KEYS:
            if float(pivot[key]) <= 0.0:
                raise GenError(f"{where}: pivot.{key} は正の値にしてください")

        slalom = preset.get("slalom")
        if slalom is not None:
            if not isinstance(slalom, str):
                raise GenError(f"{where}: slalom はスラロームの組の名前（slalom_params.json のキー，例 \"500_b\"）です")
            try:
                key = parse_slalom_key(slalom)
            except ValueError as e:
                raise GenError(f"{where}: {e}") from None
            if key.speed != speed:
                raise GenError(f"{where}: slalom「{slalom}」の速度 {key.speed:g} が speed {speed:g} と違います"
                               "（ターンは探索速度で曲がる）")
            if "fan" in preset and preset["fan"] != key.fan:
                raise GenError(f"{where}: slalom「{slalom}」のファン（{'ON' if key.fan else 'OFF'}）が fan と違います")
        fan = preset.get("fan", key.fan if slalom is not None else False)
        wall_control = preset.get("wall_control", False)
        front_correction = preset.get("front_correction", False)
        one_way = preset.get("one_way", False)
        reset_walls = preset.get("reset_walls", True)
        for key, value in [("fan", fan), ("wall_control", wall_control), ("front_correction", front_correction),
                           ("one_way", one_way), ("reset_walls", reset_walls)]:
            if not isinstance(value, bool):
                raise GenError(f"{where}: {key} は true / false です")
        if slalom is None:
            slalom = make_speed_key(speed, fan) if isinstance(fan, bool) else ""
        variant = parse_slalom_key(slalom).variant if slalom else ""
        condition = f"組「{slalom}」（{speed:g}mm/s ファン{'ON' if fan else 'OFF'}）"

        goal = preset.get("goal")  # None なら config::search::GOAL_X/Y
        if goal is not None:
            if (not isinstance(goal, list) or len(goal) != 2
                    or not all(isinstance(g, int) and not isinstance(g, bool) for g in goal)):
                raise GenError(f"{where}: goal は区画の [x, y] です（例 [1, 0]）")
            if not all(0 <= g < MAZE_SIZE for g in goal):
                raise GenError(f"{where}: goal {goal} は迷路（{MAZE_SIZE}x{MAZE_SIZE}）の外です")
            if goal == [0, 0]:
                raise GenError(f"{where}: goal をスタート区画 (0, 0) にはできません")

        confirm = preset.get("confirm")  # None なら最短走行の経路で確かめない
        if confirm is not None:
            if not isinstance(confirm, str) or confirm not in run_presets:
                raise GenError(f"{where}: confirm は最短走行のプリセットの名前（run_presets.json のキー）です"
                               f"（「{confirm}」はありません）")
            if one_way:
                raise GenError(f"{where}: confirm（ゴールの後も確かめてからスタートへ戻る）と one_way は一緒に使えません")

        menu = preset.get("menu", "search")
        if menu not in MENU_ARRAYS:
            raise GenError(f"{where}: menu は {' / '.join(MENU_ARRAYS)} のどれかです")

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
            if slalom not in designed:
                raise GenError(f"{where}: {turn.label} の {condition} は slalom_params.json に設計されていません"
                               f"（設計済み: {', '.join(sorted(designed, key=slalom_key_order)) or 'なし'}）")
            resolved[cpp_name] = (turn.label, cpp_ident(turn.cpp_name, speed, fan, variant))
        s90 = by_cpp_name[REQUIRED_TURN]
        if s90.entry != "edge" or s90.exit != "edge":
            raise GenError(f"{REQUIRED_TURN} は入口・出口とも区画境界のはずです（slalom_presets.py を確認）")

        entries.append({
            "name": name,
            "ident": f"P_{name}",
            "diag_ident": f"D_{name}" if any(t in resolved for t in DIAGONAL_TURNS) else None,
            "turns": resolved,
            "pivot": pivot,
            "fan": fan,
            "wall_control": wall_control,
            "front_correction": front_correction,
            "one_way": one_way,
            "reset_walls": reset_walls,
            "confirm": confirm,
            "goal": goal,
            "menu": menu,
            "values": preset,
            "slalom": slalom,
            "straight_speed": straight_speed,
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
        '#include "config/mouse_config.hpp"',
        '#include "config/run_presets.hpp"',
        '#include "config/slalom_params.hpp"',
        "",
        "namespace config::search {",
        "",
    ]
    for e in entries:
        v = e["values"]
        note = f"  メモ: {v['note']}" if v.get("note") else ""
        labels = "・".join(label for label, _ in e["turns"].values())
        wall = (("，ファンON" if e["fan"] else "") + ("，横壁の補正あり" if e["wall_control"] else "")
                + ("，前壁でS90の入口を補正" if e["front_correction"] else ""))
        goal_note = f"，ゴール ({e['goal'][0]}, {e['goal'][1]})" if e["goal"] else ""
        mode_note = (("，片道" if e["one_way"] else "") + ("，壁を引き継ぐ" if not e["reset_walls"] else "")
                     + (f"，最短走行（{e['confirm']}）の経路で確かめる" if e["confirm"] else ""))
        goal = f"{e['goal'][0]}, {e['goal'][1]}" if e["goal"] else "GOAL_X, GOAL_Y"
        straight = f"（既知の直進 {e['straight_speed']:g}mm/s）" if e["straight_speed"] > float(v["speed"]) else ""
        out.append(f"// {e['name']}: {v['speed']:g}mm/s{straight}，ターンは組「{e['slalom']}」の{labels}"
                   f"{wall}{goal_note}{mode_note}{note}")
        if e["diag_ident"]:
            out.append(f"inline constexpr DiagonalTurns {e['diag_ident']} = {turn_list(e, DIAGONAL_TURNS)};")
        out.append(
            f"inline constexpr SearchPreset {e['ident']} = {{\"{e['name']}\", {fmt(v['speed'])}, {fmt(e['straight_speed'])}, {fmt(v['accel'])}, "
            f"{turn_list(e, ORTHO_TURNS)}, {'&' + e['diag_ident'] if e['diag_ident'] else 'nullptr'}, "
            f"{{{fmt(e['pivot']['omega'])}, {fmt(e['pivot']['alpha'])}}}, "
            f"{'true' if e['fan'] else 'false'}, {'true' if e['wall_control'] else 'false'}, "
            f"{'true' if e['front_correction'] else 'false'}, {goal}, "
            f"{'true' if e['one_way'] else 'false'}, {'true' if e['reset_walls'] else 'false'}, "
            f"{'&config::run::P_' + e['confirm'] if e['confirm'] else 'nullptr'}}};")
        out.append("")
    for menu, array in MENU_ARRAYS.items():
        listed = [e for e in entries if e["menu"] == menu]
        out.append(f"// メニュー（\"menu\": \"{menu}\"）に並べる順（search_presets.json に書いた順）")
        out.append(f"inline constexpr std::array<SearchPreset, {len(listed)}> {array} = {{")
        out += [f"    {e['ident']}," for e in listed]
        out += ["};", ""]
    out += ["} // namespace config::search", ""]
    return "\n".join(out)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--presets", default=os.path.join(TOOLS_DIR, "search_presets.json"))
    ap.add_argument("--slalom-params", default=os.path.join(TOOLS_DIR, "slalom_params.json"))
    ap.add_argument("--run-presets", default=os.path.join(TOOLS_DIR, "run_presets.json"))
    ap.add_argument("--out", help="出力先（省略で標準出力）")
    args = ap.parse_args()

    try:
        entries = build_entries(load_json(args.presets), load_json(args.slalom_params), load_json(args.run_presets))
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
