#!/usr/bin/env python3
"""スラロームのパラメータのヘッダ（config/slalom_params.hpp）を生成する。

  設計値: slalom_params.json  … slalom_profile_designer.py が保存する（シミュレータの値）
  調整値: slalom_tuning.json  … 実機での手動調整。設計値への差分だけを書く

CMake（CMakeLists.txt）がビルドのたびに，どちらかのJSONかこのスクリプトが変わっていれば再生成する。
標準ライブラリだけで動く（仮想環境は不要）。

slalom_tuning.json の形（キーは slalom_params.json と同じ「表示名」→「速度」）:

    {
      "小回り90°": {
        "500": {
          "base_saved_at": "2026-10-02T03:34:10",
          "delta": { "Set_pri_offset": -2.0, "Set_post_offset": 1.5 },
          "note": "出口で右に1mmずれる。電池8.1V"
        }
      }
    }

  - base_saved_at … どの設計値に対して調整したか（slalom_params.json の saved_at）。
    設計値をデザイナーで保存し直すと一致しなくなり，生成をエラーで止める（古い差分をそのまま使わないため）。
    差分がまだ有効なら base_saved_at を新しい saved_at に書き換える
  - delta … 足す量。使えるキーは DELTA_KEYS
  - note … 任意（調整の理由・条件）。ヘッダのコメントに出す

使い方:
    python3 tools/gen_slalom_params.py                 # 標準出力へ
    python3 tools/gen_slalom_params.py --out path.hpp  # ファイルへ
"""
import argparse
import json
import os
import sys

from slalom_presets import PRESET_BY_LABEL

TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))

# JSONのキー → (Paramのメンバ, 単位)。速度は表のキーなので差分にできない
DELTA_KEYS = {
    "Set_low_AngVel": ("omega_max", "dps"),
    "Set_Low_AngAcl": ("alpha", "dps/s"),
    "Set_pri_offset": ("pre_offset", "mm"),
    "Set_post_offset": ("post_offset", "mm"),
}

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


def cpp_ident(cpp_name, speed_key):
    return f"{cpp_name}_{speed_key.replace('.', 'p')}"


def build_entries(params, tuning):
    entries = []
    for label, by_speed in params.items():
        preset = PRESET_BY_LABEL.get(label)
        if preset is None:
            raise GenError(f"slalom_params.json の「{label}」は slalom_presets.py にないターンです")
        for speed_key, design in sorted(by_speed.items(), key=lambda kv: float(kv[0])):
            where = f"{label} / {speed_key}mm/s"
            values = {key: float(design[key]) for key in DELTA_KEYS}
            tune = tuning.get(label, {}).get(speed_key)
            applied = []
            note = ""
            if tune is not None:
                if tune.get("base_saved_at") != design.get("saved_at"):
                    raise GenError(
                        f"{where}: 調整の base_saved_at ({tune.get('base_saved_at')}) が設計値の saved_at "
                        f"({design.get('saved_at')}) と一致しません。設計値が保存し直されています。"
                        f"差分を見直して slalom_tuning.json の base_saved_at を更新してください")
                for key, d in tune.get("delta", {}).items():
                    if key not in DELTA_KEYS:
                        raise GenError(f"{where}: 調整できないキー {key}（使えるのは {', '.join(DELTA_KEYS)}）")
                    if float(d) != 0.0:
                        applied.append((key, values[key], float(d)))
                        values[key] += float(d)
                note = tune.get("note", "")

            ramp = values["Set_low_AngVel"] ** 2 / (2.0 * values["Set_Low_AngAcl"])
            if preset.angle - 2.0 * ramp < 0.0:
                raise GenError(
                    f"{where}: 最大角速度まで加速しきれません（加速・減速で{2 * ramp:.1f}° > 旋回角{preset.angle:g}°）。"
                    f"シミュレータもこの形（三角形）は正しく扱えないので，角速度を下げるか角加速度を上げてください")

            entries.append({
                "ident": cpp_ident(preset.cpp_name, speed_key),
                "label": label,
                "preset": preset,
                "speed": float(design["Set_Speed"]),
                "values": values,
                "applied": applied,
                "note": note,
                "saved_at": design.get("saved_at", ""),
                "sim_total": design.get("result", {}).get("total_dist"),
            })

    # 表にない調整は書き間違いの可能性が高いので止める
    for label, by_speed in tuning.items():
        for speed_key in by_speed:
            if speed_key not in params.get(label, {}):
                raise GenError(f"slalom_tuning.json の「{label} / {speed_key}mm/s」に対応する設計値がありません")
    return entries


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
        out.append(f"// {e['label']} {e['speed']:g}mm/s（設計値 saved {e['saved_at']}{sim}）")
        if e["applied"]:
            for key, base, d in e["applied"]:
                member, unit = DELTA_KEYS[key]
                out.append(f"//   調整 {member}: {base:g} → {base + d:g} {unit}（{d:+g}）")
            if e["note"]:
                out.append(f"//   メモ: {e['note']}")
        else:
            out.append("//   調整なし")
        out.append(
            f"inline constexpr Param {e['ident']} = {{\"{e['ident']}\", {fmt(p.angle)}, "
            f"Anchor::{ANCHOR[p.entry]}, Anchor::{ANCHOR[p.exit]}, {fmt(e['speed'])}, "
            f"{fmt(v['Set_low_AngVel'])}, {fmt(v['Set_Low_AngAcl'])}, "
            f"{fmt(v['Set_pri_offset'])}, {fmt(v['Set_post_offset'])}}};")
        out.append("")
    out.append("// すべてのパラメータ（試験のメニューに並べる）")
    out.append(f"inline constexpr std::array<Param, {len(entries)}> ALL = {{")
    out += [f"    {e['ident']}," for e in entries]
    out += ["};", "", "} // namespace config::slalom", ""]
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
