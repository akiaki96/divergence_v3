#!/usr/bin/env python3
"""最短走行のプリセットのヘッダ（config/run_presets.hpp）を生成する。

  run_presets.json … プリセット名 → ターンの速度（種類ごとに変えてよい）・直線の最高速度・加減速度・斜めを使うか など（手で書く）

最短走行の経路（time_based_dijkstra）は、直線が続くところは区画中央から大回り90°（L90）・180°（T180）で、
1区画ずつ曲がるジグザグは区画の辺から小回り90°（S90）で曲がる。"diagonal": true なら斜めのターン
（IN45, OUT45, V90, IN135, OUT135）も使い、ジグザグは斜めで走れるので S90 は任意（設計があれば使う。
"s90": false なら設計があっても使わない）。V90 も "v90": false で使わないようにできる。使わないターンはソルバーの
コストを大きくして選ばせない。

ターンの速度は種類ごとに決める（"turn_speed" が既定，"speeds" で種類ごとに上書き）。その速度の設計
（config::slalom::<turn>_<speed>、"fan": true なら _FAN）がいちばん速い候補で，slalom_params.json にある同じ種類・
同じファンの条件のもっと遅い設計（基本の組だけ）がすべて下の候補になる。走る経路で，あいだの直線が短くて速度を
変えきれない（ターンどうしが直線なしで続くなら同じ速度でなければならない）ところや，スタート直後・ゴール直前で
加速・減速しきれないところは，実機（fast_plan::fitSpeeds）が下の候補に落とす。いちばん速い候補が設計されていなければ
生成をエラーで止める（ビルドが止まる）。

run_presets.json の形:

    {
      "1500_dia": {
        "turn_speed": 1500,       … ターンの速度の既定 [mm/s]。ソルバーの直線のコストの始点・終点の速度にも使う
        "speeds": {"S90": 900},   … 任意。種類ごとのターンの速度 [mm/s]（S90 L90 T180 IN45 OUT45 V90 IN135 OUT135）
        "max_speed": 2000,        … 縦横の直線の最高速度 [mm/s]（どのターンの速度以上）
        "max_speed_dia": 1500,    … 斜めの直線の最高速度 [mm/s]（斜めのターンの速度以上。diagonal が false でも書く）
        "accel": 8000,            … 直線の加速度 [mm/s^2]
        "decel": 8000,            … 直線の減速度 [mm/s^2]
        "diagonal": true,         … 斜めの経路を使うか
        "fan": false,             … 任意（省略で false）。ファンを回して走るか
        "wall_edge": true,        … 任意（省略で false）。区画中央から入るターン（L90・T180・IN45・IN135）の前の直線で壁切れの補正をかけるか
        "s90": false,             … 任意（斜めありのときだけ。省略で true＝設計があれば使う）。小回り90°を使うか
        "v90": false,             … 任意（斜めありのときだけ。省略で true＝必ず使う）。V90 を使うか
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
from slalom_presets import PRESET_LIST, make_speed_key, parse_speed_key

TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))

REQUIRED_KEYS = ["turn_speed", "max_speed", "max_speed_dia", "accel", "decel", "diagonal"]
OPTIONAL_KEYS = ["speeds", "fan", "wall_edge", "s90", "v90", "note"]

ORTHO_TURNS = ["S90", "L90", "T180"]
DIAGONAL_TURNS = ["IN45", "OUT45", "V90", "IN135", "OUT135"]
# RunPreset::turns の並び（ソルバーの TurnKind：solver/core/solver_options.h）
KIND_ORDER = [("L90", "TURN_L90"), ("T180", "TURN_180"), ("IN45", "TURN_IN45"), ("OUT45", "TURN_OUT45"),
              ("IN135", "TURN_IN135"), ("OUT135", "TURN_OUT135"), ("V90", "TURN_V90"), ("S90", "TURN_S90")]


def designed_speeds(slalom_params, label, fan):
    """そのターン・ファンの条件で設計されている速度（基本の組だけ）。速い順"""
    out = []
    for key in slalom_params.get(label, {}):
        speed, f = parse_speed_key(key)
        if f == fan and key == make_speed_key(speed, fan):
            out.append(speed)
    return sorted(out, reverse=True)


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
            hint = "（s90_speed・start_speed は無くなりました：\"speeds\": {\"S90\": …} を使う。スタート直後は自動で下の候補に落とす）" \
                if {"s90_speed", "start_speed"} & set(unknown) else ""
            raise GenError(f"{where}: 足りないキー {missing}，知らないキー {unknown}{hint}")

        for key in ["turn_speed", "max_speed", "max_speed_dia", "accel", "decel"]:
            if float(preset[key]) <= 0.0:
                raise GenError(f"{where}: {key} は正の値にしてください")
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

        kinds = ORTHO_TURNS + (DIAGONAL_TURNS if diagonal else [])
        speeds = preset.get("speeds", {})
        if not isinstance(speeds, dict) or any(k not in kinds for k in speeds):
            raise GenError(f"{where}: speeds のキーは {', '.join(kinds)} です")
        if "S90" in speeds and not use_s90:
            raise GenError(f"{where}: \"s90\": false なら speeds に S90 は書けません")
        if "V90" in speeds and not use_v90:
            raise GenError(f"{where}: \"v90\": false なら speeds に V90 は書けません")

        ladders = {}   # 種類 → [(速度, C++ の名前)]（速い順）
        for cpp_name in kinds:
            if (cpp_name == "S90" and not use_s90) or (cpp_name == "V90" and not use_v90):
                continue
            turn = by_cpp_name[cpp_name]
            top = float(speeds.get(cpp_name, preset["turn_speed"]))
            if top <= 0.0:
                raise GenError(f"{where}: {cpp_name} の速度は正の値にしてください")
            have = designed_speeds(slalom_params, turn.label, fan)
            if top not in have:
                # 斜めありなら小回り90°は任意（速度を書いていなければ，無いときはジグザグを斜めで走る）
                if cpp_name == "S90" and diagonal and "S90" not in speeds:
                    continue
                raise GenError(f"{where}: {turn.label} の {top:g}mm/s ファン{'ON' if fan else 'OFF'} は slalom_params.json に設計されていません"
                               f"（設計済み: {', '.join(f'{v:g}' for v in reversed(have)) or 'なし'}）"
                               + ("。斜めを使わないなら \"diagonal\": false" if cpp_name in DIAGONAL_TURNS else ""))
            ladders[cpp_name] = [(v, cpp_ident(turn.cpp_name, v, fan)) for v in have if v <= top]

        tops = {k: l[0][0] for k, l in ladders.items()}
        if float(preset["max_speed"]) < max(tops.values()):
            raise GenError(f"{where}: max_speed は いちばん速いターン（{max(tops.values()):g}mm/s）以上にしてください")
        dia_tops = [v for k, v in tops.items() if k in DIAGONAL_TURNS]
        if dia_tops and float(preset["max_speed_dia"]) < max(dia_tops):
            raise GenError(f"{where}: max_speed_dia は いちばん速い斜めのターン（{max(dia_tops):g}mm/s）以上にしてください")
        if float(preset["max_speed_dia"]) < float(preset["turn_speed"]) or float(preset["max_speed"]) < float(preset["turn_speed"]):
            raise GenError(f"{where}: max_speed・max_speed_dia は turn_speed 以上にしてください")

        entries.append({
            "name": name,
            "ident": f"P_{name}",
            "ladders": ladders,
            "fan": fan,
            "diagonal": diagonal,
            "wall_edge": wall_edge,
            "values": preset,
        })
    if not entries:
        raise GenError("run_presets.json にプリセットがありません")
    # メニューは Fast → ファン（OFF / ON）→ 縦横 / 斜め → プリセット なので，ファンOFFを先に，同じファンの中は
    # 斜めなしを先に並べる（同じ段の中は書いた順）
    entries.sort(key=lambda e: (e["fan"], e["diagonal"]))
    return entries


def ladder_ident(kind, ladder, fan):
    return f"LADDER_{kind}_{ladder[0][0]:g}".replace(".", "p") + ("_FAN" if fan else "")


def describe(e):
    """ヘッダのコメント：種類ごとのいちばん速い候補"""
    tops = {k: l[0][0] for k, l in e["ladders"].items()}
    parts = []
    for k, _ in KIND_ORDER:
        if k in tops:
            parts.append(f"{k} {tops[k]:g}")
    gone = [k for k in (ORTHO_TURNS + (DIAGONAL_TURNS if e["diagonal"] else [])) if k not in tops]
    return "，".join(parts) + " mm/s" + (f"（{'・'.join(gone)} なし）" if gone else "")


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
        "// ターンの候補（速い順。いちばん速いものがプリセットの速度，下は fast_plan::fitSpeeds が落とす先）",
    ]
    seen = set()
    for e in entries:
        for kind, ladder in e["ladders"].items():
            ident = ladder_ident(kind, ladder, e["fan"])
            if ident in seen:
                continue
            seen.add(ident)
            out.append(f"inline constexpr const slalom::Param* {ident}[] = {{"
                       + ", ".join(f"&config::slalom::{c}" for _, c in ladder) + "};")
    out.append("")
    for e in entries:
        v = e["values"]
        note = f"  メモ: {v['note']}" if v.get("note") else ""
        extra = (("，斜めあり" if e["diagonal"] else "，斜めなし") + ("，ファンON" if e["fan"] else "")
                 + ("，壁切れ補正" if e["wall_edge"] else ""))
        out.append(f"// {e['name']}: ターン {describe(e)}，直線 {v['max_speed']:g} / 斜め {v['max_speed_dia']:g}mm/s"
                   f"{extra}{note}")
        turns = []
        for kind, _ in KIND_ORDER:
            ladder = e["ladders"].get(kind)
            turns.append(f"{{{ladder_ident(kind, ladder, e['fan'])}, {len(ladder)}}}" if ladder else "{nullptr, 0}")
        out.append(
            f"inline constexpr RunPreset {e['ident']} = {{\"{e['name']}\", {fmt(v['turn_speed'])}, {fmt(v['max_speed'])}, "
            f"{fmt(v['max_speed_dia'])}, {fmt(v['accel'])}, {fmt(v['decel'])}, {{{', '.join(turns)}}}, "
            f"{'true' if e['diagonal'] else 'false'}, {'true' if e['fan'] else 'false'}, "
            f"{'true' if e['wall_edge'] else 'false'}}};")
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
        if kinds and kinds[-1][0] == e["fan"] and kinds[-1][1] == e["diagonal"]:
            kinds[-1][3] += 1
        else:
            kinds.append([e["fan"], e["diagonal"], i, 1])
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
