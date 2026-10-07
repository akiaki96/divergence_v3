#!/usr/bin/env python3
"""斜めの直線の制御（common/diag_control.hpp）の表のヘッダ（config/diag_table.hpp）を生成する。

  diag_table.json … 切れ目からの距離 → 横のセンサーの基準値と横ずれへの感度（--make で作り，リポジトリに入れる）

表の作り方（--make）：tools/diag_sensor.py の出力（reference.json の形）を3つ使う。
  --ref    基準にする走行（横のずれが 0 に近い走行。左右の入り方を同じ数ずつ）の表 → 基準値。
           走行の横の平均（runs[].lateral_mm：切れ目の位相から出したもの）が 0 でなければ，感度 × 平均の分を
           引いて中心線上の値にする
  --sens   横の位置だけが違う2組の走行（例：入45°の補正前の左入り・右入り）の表 → 感度
           感度 = 2組の表の差 / 2組の横の平均の差（runs[].lateral_mm，左が正）。左のセンサーは左へ寄ると値が
           増え，右のセンサーは右へ寄ると増えるので，右は符号を反転する
切れ目からの距離 BIN_MM ごとにまとめ，基準値の「使う」区間（diag_sensor.py の use）で感度が --min-sens 以上の
ところだけを使う（感度 0 は「使わない」）。

使い方:
    python3 tools/gen_diag_table.py --make --ref post.json --sens pre_left.json pre_right.json   # diag_table.json を作る
    python3 tools/gen_diag_table.py                    # ヘッダを標準出力へ
    python3 tools/gen_diag_table.py --out path.hpp     # ヘッダをファイルへ（CMake がビルドのたびに呼ぶ）
"""
import argparse
import datetime
import json
import os
import statistics
import sys

from gen_slalom_params import GenError, fmt, load_json

TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_TABLE = os.path.join(TOOLS_DIR, "diag_table.json")

# diag_sensor.py の表の列 → このヘッダの配列（左のセンサーは左の切れ目，右は右の切れ目からの距離）
SIDES = [("ir_l", "left", "LEFT", +1.0), ("ir_r", "right", "RIGHT", -1.0)]


def _bins(table, bin_mm):
    """2 mm の表を bin_mm ごとにまとめる。{中心: (平均, 使う)}（n で重み付け）"""
    out = {}
    for s, m, n, use in zip(table["since"], table["mean"], table["n"], table["use"]):
        if m is None or n == 0:
            continue
        k = int(s // bin_mm)
        acc = out.setdefault(k, [0.0, 0, True])
        acc[0] += m * n
        acc[1] += n
        acc[2] = acc[2] and use
    return {k: (sum_ / n, use) for k, (sum_, n, use) in out.items() if n > 0}


def _mean_lateral(result):
    lats = [r["lateral_mm"] for r in result.get("runs", [])]
    if not lats:
        raise GenError("runs[].lateral_mm がありません（diag_sensor.py の新しい出力を使う）")
    return statistics.fmean(lats)


def make(ref_path, sens_paths, bin_mm, min_sens, out_path):
    ref = load_json(ref_path)
    a, b = (load_json(p) for p in sens_paths)
    d_lat = _mean_lateral(a) - _mean_lateral(b)
    if abs(d_lat) < 5.0:
        raise GenError(f"感度の2組の横の差 {d_lat:+.1f} mm が小さすぎます（5 mm 以上）")
    print(f"sensitivity from lateral {_mean_lateral(a):+.2f} vs {_mean_lateral(b):+.2f} mm (diff {d_lat:+.2f})")

    ref_lat = _mean_lateral(ref)
    print(f"reference runs: mean lateral {ref_lat:+.2f} mm (left +) -> shifted to the center line")

    sides = {}
    for col, key, _, sign in SIDES:
        r = _bins(ref["tables"][col], bin_mm)
        ta = _bins(a["tables"][col], bin_mm)
        tb = _bins(b["tables"][col], bin_mm)
        ks = sorted(r)
        raw = {k: sign * (ta[k][0] - tb[k][0]) / d_lat for k in ks if k in ta and k in tb}
        # 感度は隣と平均してなめらかにする（2組の差なので基準値より雑音が大きい）
        sens = {}
        for k in raw:
            near = [raw[j] for j in (k - 1, k, k + 1) if j in raw]
            sens[k] = statistics.fmean(near)
        std = {}
        for s, sd, use in zip(ref["tables"][col]["since"], ref["tables"][col]["std"], ref["tables"][col]["use"]):
            if sd is not None and use:
                std.setdefault(int(s // bin_mm), []).append(sd)
        rows = []
        for k in range(ks[0], ks[-1] + 1):
            m, use = r.get(k, (None, False))
            s = sens.get(k, 0.0)
            ok = m is not None and use and s >= min_sens
            if ok:
                m -= s * sign * ref_lat   # 基準の走行がその側へ寄っていた分を引く（右のセンサーは右へ寄ると増える）
            rows.append({"since": round((k + 0.5) * bin_mm, 2), "ref": round(m, 1) if m is not None else 0.0,
                         "sens": round(s, 2) if ok else 0.0})
        used = [x for x in rows if x["sens"] > 0]
        if not used:
            raise GenError(f"{col}: 使える区間がありません")
        noise = [statistics.fmean(std[int(x["since"] // bin_mm)]) / x["sens"]
                 for x in used if int(x["since"] // bin_mm) in std]
        print(f"{col}: use {used[0]['since'] - bin_mm / 2:g}..{used[-1]['since'] + bin_mm / 2:g} mm since the edge, "
              f"{len(used)} bins, sensitivity {used[0]['sens']:.1f}..{max(x['sens'] for x in used):.1f} count/mm, "
              f"run-to-run std in mm: median {statistics.median(noise):.1f}")
        sides[key] = rows

    table = {
        "generated_at": datetime.date.today().isoformat(),
        "ref": os.path.basename(ref_path),
        "ref_runs": [r["file"] for r in ref.get("runs", [])],
        "ref_lateral_mm": round(ref_lat, 2),
        "sens": [os.path.basename(p) for p in sens_paths],
        "sens_lateral_diff_mm": round(d_lat, 2),
        "thresholds": ref.get("thresholds"),
        "bin_mm": bin_mm,
        "min_sens": min_sens,
        "left": sides["left"],
        "right": sides["right"],
    }
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(table, f, indent=1, ensure_ascii=False)
        f.write("\n")
    print(f"wrote {out_path}")


def check(table):
    bin_mm = float(table["bin_mm"])
    for _, key, _, _ in SIDES:
        rows = table.get(key)
        if not rows:
            raise GenError(f"diag_table.json に {key} がありません")
        for r0, r1 in zip(rows, rows[1:]):
            if abs(r1["since"] - r0["since"] - bin_mm) > 1e-3:
                raise GenError(f"{key}: since が {bin_mm:g} mm おきではありません（{r0['since']} → {r1['since']}）")
        if not any(r["sens"] > 0 for r in rows):
            raise GenError(f"{key}: 使える区間がありません")
        if any(r["sens"] < 0 for r in rows):
            raise GenError(f"{key}: 感度が負です")


def render(table):
    bin_mm = float(table["bin_mm"])
    left, right = table["left"], table["right"]
    if abs(left[0]["since"] - right[0]["since"]) > 1e-3 or len(left) != len(right):
        raise GenError("left と right の since の並びが違います")
    out = [
        "// 自動生成ファイル：tools/gen_diag_table.py が tools/diag_table.json から生成する。",
        "// 編集しないこと（斜めのセンサーのログから gen_diag_table.py --make で JSON を作り直す）",
        f"// 表の作成: {table.get('generated_at', '?')}，基準 {', '.join(table.get('ref_runs', []))}",
        f"// 感度: {', '.join(table.get('sens', []))}（横の差 {table.get('sens_lateral_diff_mm', '?')} mm）",
        "#pragma once",
        "",
        "#include <cstddef>",
        "",
        "namespace config::diag_table {",
        "",
        "// 切れ目からの距離 SINCE0_MM + i·STEP_MM での横のセンサーの基準値（中心線上，並べ方 A＝柱の両側に壁）と",
        "// 横ずれへの感度 [count/mm]（そのセンサーの側へ寄ると値が増える向きを正）。感度 0 は「使わない」",
        "struct Entry {",
        "    float ref;",
        "    float sens;",
        "};",
        "",
        f"inline constexpr float SINCE0_MM = {fmt(left[0]['since'])};",
        f"inline constexpr float STEP_MM = {fmt(bin_mm)};",
        f"inline constexpr std::size_t SIZE = {len(left)};",
        "",
    ]
    for _, key, name, _ in SIDES:
        out.append(f"inline constexpr Entry {name}[SIZE] = {{")
        out += [f"    {{{fmt(r['ref'])}, {fmt(r['sens'])}}},   // {r['since']:g}" for r in table[key]]
        out += ["};", ""]
    out += ["} // namespace config::diag_table", ""]
    return "\n".join(out)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--table", default=DEFAULT_TABLE, help="diag_table.json")
    ap.add_argument("--out", help="ヘッダの出力先（省略で標準出力）")
    ap.add_argument("--make", action="store_true", help="diag_sensor.py の出力から diag_table.json を作る")
    ap.add_argument("--ref", help="--make：基準値の表（diag_sensor.py --out）")
    ap.add_argument("--sens", nargs=2, metavar=("A", "B"), help="--make：横の位置が違う2組の表")
    ap.add_argument("--bin", type=float, default=4.0, help="--make：区間の幅 [mm]（既定 4）")
    ap.add_argument("--min-sens", type=float, default=2.5,
                    help="--make：これより感度の低い区間は使わない [count/mm]（既定 2.5）")
    args = ap.parse_args()

    try:
        if args.make:
            if not args.ref or not args.sens:
                raise GenError("--make には --ref と --sens が要ります")
            make(args.ref, args.sens, args.bin, args.min_sens, args.table)
            return 0
        table = load_json(args.table)
        check(table)
        text = render(table)
    except GenError as e:
        print(f"gen_diag_table: エラー: {e}", file=sys.stderr)
        return 1

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
