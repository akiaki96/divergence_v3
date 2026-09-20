#!/bin/sh
# 回転角速度制御（2自由度FF・Tiスケジュール）のホスト検証を実行する。実機・ARMツールチェーン不要（c++のみ）。
#   使い方: sh tools/host_test/run.sh
# motorDriver.cpp から補間・FF関数を切り出し（ff_funcs.inc），実際の pid.cpp と一緒にコンパイルして実行する。
set -e
ROOT=$(cd "$(dirname "$0")/../.." && pwd)
OUT=$(mktemp -d)
trap 'rm -rf "$OUT"' EXIT

awk '/^\/\/ 区分線形補間：/{f=1} f{print} /^static float omega_accel_ff/{g=1} g&&/^}/{exit}' \
    "$ROOT/Core/Src/device/motorDriver.cpp" > "$OUT/ff_funcs.inc"
[ -s "$OUT/ff_funcs.inc" ] || { echo "motorDriver.cpp から関数を切り出せませんでした" >&2; exit 2; }

c++ -std=c++17 -Wall -I"$ROOT/Core/Inc" -I"$OUT" \
    "$ROOT/tools/host_test/omega_ff_test.cpp" "$ROOT/Core/Src/common/pid.cpp" -o "$OUT/omega_ff_test"
"$OUT/omega_ff_test"
