#!/usr/bin/env python3
"""最短走行のプリセットのヘッダ（config/run_presets.hpp）を生成する。

  run_presets.json … プリセット名 → ターンの速度・直線の最高速度・加減速度・斜めを使うか など（手で書く）

最短走行の経路（time_based_dijkstra）は、直線が続くところは区画中央から大回り90°（L90）・180°（T180）で、
1区画ずつ曲がるジグザグは区画の辺から小回り90°（S90）で曲がる。"diagonal": true なら斜めのターン
（IN45, OUT45, V90, IN135, OUT135）も使い、ジグザグは斜めで走れるので S90 は任意（設計があれば使う。
"s90": false なら設計があっても使わない。使わないときはソルバーの S90 のコストを大きくして選ばせない）。
V90 も "v90": false で使わないようにできる（1500mm/s 以上は α の上限で設計できないため。コストの扱いは S90 と同じ）。
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
        "s90": false,             … 任意（斜めありのときだけ。省略で true＝設計があれば使う）。小回り90°を使うか
        "v90": false,             … 任意（斜めありのときだけ。省略で true＝必ず使う）。V90 を使うか。使わないと
                                    斜めのジグザグは出45°・入45°（間に斜めの直線）か縦横で走る
        "start_speed": 900,       … 任意（省略で置き換えない）。スタート直後，最初の区画中央までで最初のターンの速度まで
                                    加速しきれないときだけ，そのターンと直線なしで続くターンをこの速度の同じ種類に置き換える。
                                    turn_speed が約 1100mm/s を超えるときに要る（区画中央から入るターンにスタートから
                                    42mm で入るため）。大回り90°・180°（斜めありなら斜めの5種類も）をこの速度で設計しておく
        "s90_speed": 900,         … 任意（省略で turn_speed）。小回り90°の速度 [mm/s]。turn_speed はほかのターン（大回り90°・
                                    180°，斜めありなら斜めの5種類）の速度になる。斜めありでも書けば S90 を必ず使う（設計が要る）。
                                    S90 と他のターンの間の直線は最短で半区画なので（ほかのターンは S90 側の口が区画中央），
                                    |turn_speed² − s90_speed²| ≤ 2·min(accel, decel)·90mm でないとエラー
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
OPTIONAL_KEYS = ["fan", "wall_edge", "s90", "v90", "s90_speed", "start_speed", "note"]

HALF_CELL_MM = 90.0   # S90 と他のターンの間の直線の最短（ソルバーが S90 の前後に足す半区画）

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
        use_s90 = preset.get("s90", True)
        use_v90 = preset.get("v90", True)
        for key, value in [("fan", fan), ("diagonal", diagonal), ("wall_edge", wall_edge), ("s90", use_s90),
                           ("v90", use_v90)]:
            if not isinstance(value, bool):
                raise GenError(f"{where}: {key} は true / false です")
        if not use_s90 and not diagonal:
            raise GenError(f"{where}: 斜めなしでは小回り90°（S90）が要るので \"s90\": false にできません")
        if not use_v90 and not diagonal:
            raise GenError(f"{where}: \"v90\": false は斜めありのときだけ書けます")
        s90_speed = float(preset.get("s90_speed", speed))
        if s90_speed <= 0.0:
            raise GenError(f"{where}: s90_speed は正の値にしてください")
        if s90_speed != speed:
            if not use_s90:
                raise GenError(f"{where}: \"s90\": false なら s90_speed は書けません")
            if float(preset["max_speed"]) < s90_speed:
                raise GenError(f"{where}: max_speed は s90_speed 以上にしてください")
            need = abs(speed ** 2 - s90_speed ** 2) / (2.0 * HALF_CELL_MM)
            have = min(float(preset["accel"]), float(preset["decel"]))
            if need > have:
                raise GenError(f"{where}: S90（{s90_speed:g}mm/s）と他のターン（{speed:g}mm/s）の間の半区画の直線で"
                               f"速度を変えきれません。accel と decel を {need:.0f} mm/s^2 以上にするか，速度の差を小さくしてください")

        kinds = ORTHO_TURNS + ([k for k in DIAGONAL_TURNS if use_v90 or k != "V90"] if diagonal else [])
        resolved = {}
        for cpp_name in kinds:
            turn = by_cpp_name[cpp_name]
            designed = slalom_params.get(turn.label, {})
            turn_v = s90_speed if cpp_name == "S90" else speed
            if cpp_name == "S90" and diagonal and (not use_s90 or (turn_v == speed and make_speed_key(turn_v, fan) not in designed)):
                continue   # 斜めありなら小回り90°は任意（無ければジグザグを斜めで走る）
            # 最短走行は基本の組（"500" / "500_fan"）を使う
            if make_speed_key(turn_v, fan) not in designed:
                raise GenError(f"{where}: {turn.label} の {turn_v:g}mm/s ファン{'ON' if fan else 'OFF'} は slalom_params.json に設計されていません"
                               f"（設計済み: {', '.join(sorted(designed, key=slalom_key_order)) or 'なし'}）"
                               + ("。斜めを使わないなら \"diagonal\": false" if cpp_name in DIAGONAL_TURNS else ""))
            resolved[cpp_name] = (turn.label, cpp_ident(turn.cpp_name, turn_v, fan))

        # スタート用のターン（小回り90°は区画中央のターンの直後に来ないので要らない）
        start_speed = float(preset.get("start_speed", 0.0))
        start = {}
        if start_speed:
            if not 0.0 < start_speed < speed:
                raise GenError(f"{where}: start_speed は 0 より大きく turn_speed より小さくしてください")
            for cpp_name in [k for k in kinds if k != "S90"]:
                turn = by_cpp_name[cpp_name]
                if make_speed_key(start_speed, fan) not in slalom_params.get(turn.label, {}):
                    raise GenError(f"{where}: start_speed の {turn.label} {start_speed:g}mm/s ファン{'ON' if fan else 'OFF'} "
                                   f"は slalom_params.json に設計されていません")
                start[cpp_name] = (turn.label, cpp_ident(turn.cpp_name, start_speed, fan))

        entries.append({
            "name": name,
            "ident": f"P_{name}",
            "diag_ident": f"D_{name}" if diagonal else None,
            "turns": resolved,
            "fan": fan,
            "wall_edge": wall_edge,
            "s90_speed": s90_speed,
            "start_speed": start_speed,
            "start": start,
            "start_diag_ident": f"DS_{name}" if (diagonal and start_speed) else None,
            "values": preset,
        })
    if not entries:
        raise GenError("run_presets.json にプリセットがありません")
    # メニューは Fast → ファン（OFF / ON）→ 縦横 / 斜め → プリセット なので，ファンOFFを先に，同じファンの中は
    # 斜めなしを先に並べる（同じ段の中は書いた順）
    entries.sort(key=lambda e: (e["fan"], e["diag_ident"] is not None))
    return entries


def turn_list(e, kinds, key="turns"):
    return "{" + ", ".join(f"&config::slalom::{e[key][k][1]}" if k in e[key] else "nullptr" for k in kinds) + "}"


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
        missing = [label for k, label in [("S90", "小回り90°"), ("V90", "V90")]
                   if k not in e["turns"] and (k == "S90" or e["diag_ident"])]
        extra = (("，斜めあり" if e["diag_ident"] else "，斜めなし") + (f"（{'・'.join(missing)}なし）" if missing else "")
                 + ("，ファンON" if e["fan"] else "")
                 + ("，壁切れ補正" if e["wall_edge"] else ""))
        s90 = f"（小回り90° {e['s90_speed']:g}mm/s）" if e["s90_speed"] != float(v["turn_speed"]) else ""
        out.append(f"// {e['name']}: ターン {v['turn_speed']:g}mm/s{s90}，直線 {v['max_speed']:g} / 斜め {v['max_speed_dia']:g}mm/s"
                   f"{extra}{note}")
        if e["diag_ident"]:
            out.append(f"inline constexpr DiagonalTurns {e['diag_ident']} = {turn_list(e, DIAGONAL_TURNS)};")
        if e["start_diag_ident"]:
            out.append(f"inline constexpr DiagonalTurns {e['start_diag_ident']} = {turn_list(e, DIAGONAL_TURNS, 'start')};")
        out.append(
            f"inline constexpr RunPreset {e['ident']} = {{\"{e['name']}\", {fmt(v['turn_speed'])}, {fmt(e['s90_speed'])}, {fmt(v['max_speed'])}, "
            f"{fmt(v['max_speed_dia'])}, {fmt(v['accel'])}, {fmt(v['decel'])}, {turn_list(e, ORTHO_TURNS)}, "
            f"{'&' + e['diag_ident'] if e['diag_ident'] else 'nullptr'}, {'true' if e['fan'] else 'false'}, "
            f"{'true' if e['wall_edge'] else 'false'}, {fmt(e['start_speed'])}, {turn_list(e, ORTHO_TURNS, 'start')}, "
            f"{'&' + e['start_diag_ident'] if e['start_diag_ident'] else 'nullptr'}}};")
        out.append("")
    out.append("// メニューに並べる順（ファンOFFが先，同じファンの中は斜めなしが先，同じ段の中は run_presets.json に書いた順）")
    out.append(f"inline constexpr std::array<RunPreset, {len(entries)}> PRESETS = {{")
    out += [f"    {e['ident']}," for e in entries]
    out += ["};", ""]
    # メニューの段（並べ替え済みなので同じ段は連続している）：
    #   FANS[f]  … KINDS[first] から count 個（fan off / fan on）
    #   KINDS[k] … PRESETS[first] から count 個（縦横 / 斜め）
    kinds = []   # [fan, diag, first, count]
    for i, e in enumerate(entries):
        diag = e["diag_ident"] is not None
        if kinds and kinds[-1][0] == e["fan"] and kinds[-1][1] == diag:
            kinds[-1][3] += 1
        else:
            kinds.append([e["fan"], diag, i, 1])
    fans = []    # [fan, first, count]
    for j, (fan, _, _, _) in enumerate(kinds):
        if fans and fans[-1][0] == fan:
            fans[-1][2] += 1
        else:
            fans.append([fan, j, 1])
    out.append("// メニューのファンの段：KINDS[first]からcount個（Run → Fast → ファン → 縦横 / 斜め → プリセット）")
    out.append(f"inline constexpr std::array<::slalom::MenuGroup, {len(fans)}> FANS = {{{{")
    out += [f"    {{\"{'fan on' if fan else 'fan off'}\", {first}, {count}}}," for fan, first, count in fans]
    out += ["}};", ""]
    out.append("// メニューの縦横 / 斜めの段：PRESETS[first]からcount個")
    out.append(f"inline constexpr std::array<::slalom::MenuGroup, {len(kinds)}> KINDS = {{{{")
    out += [f"    {{\"{'diagonal' if diag else 'ortho'}\", {first}, {count}}},   // {'fan on' if fan else 'fan off'}"
            for fan, diag, first, count in kinds]
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
