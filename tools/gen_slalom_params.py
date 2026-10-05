#!/usr/bin/env python3
"""スラロームのパラメータのヘッダ（config/slalom_params.hpp）を生成する。

  設計値: slalom_params.json  … slalom_profile_designer.py が保存する（シミュレータの値）
  調整値: slalom_tuning.json  … 実機での手動調整。設計値への差分だけを書く

CMake（CMakeLists.txt）がビルドのたびに，どちらかのJSONかこのスクリプトが変わっていれば再生成する。
標準ライブラリだけで動く（仮想環境は不要）。

slalom_tuning.json の形（キーは slalom_params.json と同じ「表示名」→「速度」。ファンONは "500_fan" のように "_fan" が付く）:

    {
      "小回り90°": {
        "500": {
          "base_saved_at": "2026-10-02T03:34:10",
          "delta": { "Set_pri_offset": -2.0, "Set_post_offset": 1.5 },
          "delta_left": { "Set_post_offset": 3.2 },
          "note": "出口で右に1mmずれる。電池8.1V"
        }
      }
    }

  - base_saved_at … どの設計値に対して調整したか（slalom_params.json の saved_at）。
    設計値をデザイナーで保存し直すと一致しなくなり，生成をエラーで止める（古い差分をそのまま使わないため）。
    差分がまだ有効なら base_saved_at を新しい saved_at に書き換える
  - delta … 左右どちらの旋回にも足す量。使えるキーは DELTA_KEYS
  - delta_left / delta_right … 左旋回だけ・右旋回だけに足す量（delta に重ねて足す）。キーは delta と同じ
  - note … 任意（調整の理由・条件）。ヘッダのコメントに出す

使い方:
    python3 tools/gen_slalom_params.py                 # 標準出力へ
    python3 tools/gen_slalom_params.py --out path.hpp  # ファイルへ
"""
import argparse
import json
import os
import sys

from slalom_presets import PRESET_BY_LABEL, PRESET_LIST, parse_slalom_key, slalom_key_label, slalom_key_order

TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))

# JSONのキー → (Paramのメンバ, 単位)。速度は表のキーなので差分にできない
DELTA_KEYS = {
    "Set_low_AngVel": ("omega_max", "dps"),
    "Set_Low_AngAcl": ("alpha", "dps/s"),
    "Set_pri_offset": ("pre_offset", "mm"),
    "Set_post_offset": ("post_offset", "mm"),
}

DIRS = ("left", "right")
DIR_NAME = {"left": "左", "right": "右"}

# slalom_tuning.json の差分のキー → 足す向き
DELTA_FIELDS = {"delta": DIRS, "delta_left": ("left",), "delta_right": ("right",)}
TUNING_FIELDS = {"base_saved_at", "note", *DELTA_FIELDS}

ANCHOR = {"edge": "edge", "center": "center", "diag": "diagonal"}


class GenError(Exception):
    pass


def load_json(path, missing_ok=False):
    if missing_ok and not os.path.exists(path):
        return {}
    try:
        with open(path, encoding="utf-8") as f:
            return json.load(f)
    except (OSError, json.JSONDecodeError) as e:
        raise GenError(f"{path} を読めません: {e}")


def fmt(x):
    """C++のfloatリテラル（500.0 -> 500.f, 12.5 -> 12.5f）"""
    s = f"{float(x):g}"
    return (s + ".f") if ("." not in s and "e" not in s) else (s + "f")


def cpp_ident(cpp_name, speed, fan, variant=""):
    """S90, 500.0, False -> S90_500 / S90, 512.5, True -> S90_512p5_FAN / S90, 500.0, False, "b" -> S90_500_B"""
    return (f"{cpp_name}_{speed:g}".replace(".", "p") + ("_FAN" if fan else "")
            + (f"_{variant.upper()}" if variant else ""))


def build_entries(params, tuning):
    entries = []
    for label, by_speed in params.items():
        preset = PRESET_BY_LABEL.get(label)
        if preset is None:
            raise GenError(f"slalom_params.json の「{label}」は slalom_presets.py にないターンです")
        for speed_key in by_speed:
            try:
                parse_slalom_key(speed_key)
            except ValueError as e:
                raise GenError(f"slalom_params.json の「{label}」: {e}") from None
        for speed_key, design in sorted(by_speed.items(), key=lambda kv: slalom_key_order(kv[0])):
            speed, fan, variant = parse_slalom_key(speed_key)
            where = (f"{label} / {speed:g}mm/s ファン{'ON' if fan else 'OFF'}"
                     + (f" 組 {variant}" if variant else ""))
            if float(design["Set_Speed"]) != speed:
                raise GenError(f"{where}: キーの速度と保存データの Set_Speed（{design['Set_Speed']}）が一致しません")
            if "fan" in design and bool(design["fan"]) != fan:
                raise GenError(f"{where}: キーのファン（{'ON' if fan else 'OFF'}）と保存データの fan（{design['fan']}）が一致しません")
            base = {key: float(design[key]) for key in DELTA_KEYS}
            values = {d: dict(base) for d in DIRS}
            tune = tuning.get(label, {}).get(speed_key)
            note = ""
            if tune is not None:
                if tune.get("base_saved_at") != design.get("saved_at"):
                    raise GenError(
                        f"{where}: 調整の base_saved_at ({tune.get('base_saved_at')}) が設計値の saved_at "
                        f"({design.get('saved_at')}) と一致しません。設計値が保存し直されています。"
                        f"差分を見直して slalom_tuning.json の base_saved_at を更新してください")
                unknown = set(tune) - TUNING_FIELDS
                if unknown:
                    raise GenError(f"{where}: 知らない項目 {', '.join(sorted(unknown))}（使えるのは {', '.join(sorted(TUNING_FIELDS))}）")
                for field, dirs in DELTA_FIELDS.items():
                    for key, d in tune.get(field, {}).items():
                        if key not in DELTA_KEYS:
                            raise GenError(f"{where}: {field} の調整できないキー {key}（使えるのは {', '.join(DELTA_KEYS)}）")
                        for dir_ in dirs:
                            values[dir_][key] += float(d)
                note = tune.get("note", "")
            # 設計値から変わったキー（ヘッダのコメントに出す）
            applied = [key for key in DELTA_KEYS if any(values[d][key] != base[key] for d in DIRS)]

            for dir_ in DIRS:
                v = values[dir_]
                ramp = v["Set_low_AngVel"] ** 2 / (2.0 * v["Set_Low_AngAcl"])
                if preset.angle - 2.0 * ramp < 0.0:
                    raise GenError(
                        f"{where}（{DIR_NAME[dir_]}旋回）: 最大角速度まで加速しきれません（加速・減速で{2 * ramp:.1f}° > 旋回角{preset.angle:g}°）。"
                        f"シミュレータもこの形（三角形）は正しく扱えないので，角速度を下げるか角加速度を上げてください")

            entries.append({
                "ident": cpp_ident(preset.cpp_name, speed, fan, variant),
                "speed_name": slalom_key_label(speed_key),
                "variant": variant,
                "label": label,
                "preset": preset,
                "speed": float(design["Set_Speed"]),
                "base": base,
                "values": values,
                "applied": applied,
                "note": note,
                "saved_at": design.get("saved_at", ""),
                "sim_total": design.get("result", {}).get("total_dist"),
                "fan": fan,   # キーが決める（"_fan" ならON）
                "k_slip": design.get("Set_K_SP", 0.0),
                "c_slip": design.get("Set_C_SP", 0.0),
            })

    # メニューの並びを保存した順によらず一定にする：種類は slalom_presets.py の順，同じ種類の中は速度の昇順
    order = {p.label: i for i, p in enumerate(PRESET_LIST)}
    entries.sort(key=lambda e: (order[e["label"]], e["speed"], e["fan"], e["variant"]))

    # 表にない調整は書き間違いの可能性が高いので止める
    for label, by_speed in tuning.items():
        for speed_key in by_speed:
            if speed_key not in params.get(label, {}):
                raise GenError(f"slalom_tuning.json の「{label} / {speed_key}」に対応する設計値がありません")
    return entries


def motion(v):
    """slalom::Motion の初期化子 {omega_max, alpha, pre_offset, post_offset}"""
    return (f"{{{fmt(v['Set_low_AngVel'])}, {fmt(v['Set_Low_AngAcl'])}, "
            f"{fmt(v['Set_pri_offset'])}, {fmt(v['Set_post_offset'])}}}")


def render(entries):
    out = [
        "// 自動生成ファイル：tools/gen_slalom_params.py が tools/slalom_params.json（設計値）と",
        "// tools/slalom_tuning.json（実機での調整の差分）から生成する。編集しないこと（JSONを編集する）",
        "#pragma once",
        "",
        "#include <array>",
        '#include "common/slalom.hpp"',
        "",
        "namespace config::slalom {",
        "using ::slalom::Anchor;",
        "using ::slalom::Param;",
        "",
    ]
    for e in entries:
        p, v = e["preset"], e["values"]
        sim = f", sim total {e['sim_total']:g}mm" if e["sim_total"] is not None else ""
        fan = "ON" if e["fan"] else "OFF"
        group = f" 組 {e['variant']}" if e["variant"] else ""
        out.append(f"// {e['label']} {e['speed']:g}mm/s{group}（設計値 saved {e['saved_at']}{sim}）")
        out.append(f"//   ファン {fan}，滑り係数 K {e['k_slip']:g}，c {e['c_slip']:g}mm")
        if e["applied"]:
            for key in e["applied"]:
                member, unit = DELTA_KEYS[key]
                b = e["base"][key]
                l, r = (v[d][key] for d in DIRS)
                if l == r:
                    out.append(f"//   調整 {member}: 左右 {b:g} → {l:g} {unit}（{l - b:+g}）")
                else:
                    side = [f"{DIR_NAME[d]} {b:g} → {x:g}（{x - b:+g}）" if x != b else f"{DIR_NAME[d]} {b:g}（調整なし）"
                            for d, x in zip(DIRS, (l, r))]
                    out.append(f"//   調整 {member} [{unit}]: {'，'.join(side)}")
            if e["note"]:
                out.append(f"//   メモ: {e['note']}")
        else:
            out.append("//   調整なし")
        out.append(
            f"inline constexpr Param {e['ident']} = {{\"{e['ident']}\", \"{e['speed_name']}\", {fmt(p.angle)}, "
            f"Anchor::{ANCHOR[p.entry]}, Anchor::{ANCHOR[p.exit]}, {fmt(e['speed'])}, "
            f"{motion(v['left'])}, {motion(v['right'])}, {'true' if e['fan'] else 'false'}}};")
        out.append("")
    out.append("// すべてのパラメータ（種類の順，同じ種類の中は速度の昇順）")
    out.append(f"inline constexpr std::array<Param, {len(entries)}> ALL = {{")
    out += [f"    {e['ident']}," for e in entries]
    out += ["};", ""]

    # ALLの中で同じ種類が並ぶ範囲（並べ替え済みなので連続している）
    groups = []
    for i, e in enumerate(entries):
        name = e["preset"].cpp_name
        if groups and groups[-1][0] == name:
            groups[-1][2] += 1
        else:
            groups.append([name, i, 1])
    out.append("// 種類ごとの範囲：ALL[first]からcount個（メニューで種類→速度の順に選ぶ）")
    out.append(f"inline constexpr std::array<::slalom::TurnGroup, {len(groups)}> TURNS = {{{{")
    out += [f"    {{\"{name}\", {first}, {count}}}," for name, first, count in groups]
    out += ["}};", "", "} // namespace config::slalom", ""]
    return "\n".join(out)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--params", default=os.path.join(TOOLS_DIR, "slalom_params.json"))
    ap.add_argument("--tuning", default=os.path.join(TOOLS_DIR, "slalom_tuning.json"))
    ap.add_argument("--out", help="出力先（省略で標準出力）")
    args = ap.parse_args()

    try:
        params = load_json(args.params)
        tuning = load_json(args.tuning, missing_ok=True)
        entries = build_entries(params, tuning)
    except GenError as e:
        print(f"gen_slalom_params: エラー: {e}", file=sys.stderr)
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
