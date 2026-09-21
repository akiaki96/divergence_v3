#!/bin/sh
# 回転角速度制御・回転FF試験のホスト検証を実行する。実機・ARMツールチェーン不要（c++のみ）。
#   使い方: sh tools/host_test/run.sh
#  1. omega_ff_test   : motorDriver.cpp から補間・FF関数を切り出し（ff_funcs.inc），実際の pid.cpp と一緒に
#                       コンパイルして単体テストと閉ループ（簡易プラント族）の比較を行う
#  2. battery_guard_test : motor_id.cpp から低電圧ガードを切り出し（battery_guard.inc），スタブで判定と点滅を検証する
set -e
ROOT=$(cd "$(dirname "$0")/../.." && pwd)
OUT=$(mktemp -d)
trap 'rm -rf "$OUT"' EXIT

awk '/^\/\/ 区分線形補間：/{f=1} f{print} /^static float omega_accel_ff/{g=1} g&&/^}/{exit}' \
    "$ROOT/Core/Src/device/motorDriver.cpp" > "$OUT/ff_funcs.inc"
[ -s "$OUT/ff_funcs.inc" ] || { echo "motorDriver.cpp から関数を切り出せませんでした" >&2; exit 2; }

awk '/^constexpr float kOmegaTestMinBatteryV/{f=1} f{print} /^static bool omega_test_battery_ok/{g=1} g&&/^}/{exit}' \
    "$ROOT/Core/Src/test/motor_id.cpp" > "$OUT/battery_guard.inc"
[ -s "$OUT/battery_guard.inc" ] || { echo "motor_id.cpp から低電圧ガードを切り出せませんでした" >&2; exit 2; }

c++ -std=c++17 -Wall -I"$ROOT/Core/Inc" -I"$OUT" \
    "$ROOT/tools/host_test/omega_ff_test.cpp" "$ROOT/Core/Src/common/pid.cpp" -o "$OUT/omega_ff_test"
"$OUT/omega_ff_test"

echo
c++ -std=c++17 -Wall -I"$OUT" "$ROOT/tools/host_test/battery_guard_test.cpp" -o "$OUT/battery_guard_test"
"$OUT/battery_guard_test"
