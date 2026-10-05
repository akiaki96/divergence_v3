#!/usr/bin/env bash
# ホスト（Mac）でデバイスに依存しない部分の単体試験を実行する。使い方: tools/host_test/run.sh
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
OUT="${TMPDIR:-/tmp}/divergence_host_test"
mkdir -p "$OUT"

c++ -std=c++20 -Wall -Wextra -O1 \
    -I "$ROOT/Core/Inc" -I "$ROOT/external/etl/include" \
    "$ROOT/Core/Src/common/wall_edge.cpp" "$ROOT/tools/host_test/test_wall_edge.cpp" \
    -o "$OUT/test_wall_edge"
"$OUT/test_wall_edge"

# 前壁の距離による S90 の入口の補正（換算表のヘッダはビルドと同じく JSON から生成する）
python3 "$ROOT/tools/gen_front_distance.py" --calib "$ROOT/tools/ir_calibration.json" \
    --out "$OUT/generated/config/front_distance_table.hpp"
c++ -std=c++20 -Wall -Wextra -O1 \
    -I "$ROOT/Core/Inc" -I "$OUT/generated" \
    "$ROOT/Core/Src/common/front_correction.cpp" "$ROOT/tools/host_test/test_front_correction.cpp" \
    -o "$OUT/test_front_correction"
"$OUT/test_front_correction"

# 探索のソルバーの先読み（ソルバーはビルドと同じく external/micromouse_simulator/solver のソースをそのまま使う）
SOLVER="$ROOT/external/micromouse_simulator/solver"
SOLVER_INC=()
for d in "$SOLVER"/algorithms/*/; do
    [[ "$d" == */_template/ ]] || SOLVER_INC+=(-I "$d")
done
SOLVER_SRC=("$SOLVER"/core/*.cpp)
for f in "$SOLVER"/algorithms/*/*.cpp; do
    [[ "$f" == */_template/* ]] || SOLVER_SRC+=("$f")
done
c++ -std=c++20 -Wall -Wextra -O2 \
    -I "$ROOT/Core/Inc" -I "$ROOT/external/etl/include" -I "$SOLVER/core" "${SOLVER_INC[@]}" \
    "${SOLVER_SRC[@]}" "$ROOT/Core/Src/app/search_lookahead.cpp" "$ROOT/tools/host_test/test_search_lookahead.cpp" \
    -o "$OUT/test_search_lookahead"
"$OUT/test_search_lookahead"
