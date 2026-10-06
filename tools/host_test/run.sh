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

# 直線の台形（最短走行・探索の既知の直進・立て直しの後退）
c++ -std=c++20 -Wall -Wextra -O1 \
    -I "$ROOT/Core/Inc" \
    "$ROOT/Core/Src/common/trapezoid.cpp" "$ROOT/tools/host_test/test_trapezoid.cpp" \
    -o "$OUT/test_trapezoid"
"$OUT/test_trapezoid"

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

# 最短走行の手順（壁切れの補正に教える区画境界）。ACT_* と uint8_vector のためにソルバーの core を見る
c++ -std=c++20 -Wall -Wextra -O1 \
    -I "$ROOT/Core/Inc" -I "$ROOT/external/etl/include" -I "$SOLVER/core" \
    "$ROOT/Core/Src/common/wall_edge.cpp" "$ROOT/Core/Src/common/trapezoid.cpp" "$ROOT/Core/Src/app/fast_plan.cpp" \
    "$ROOT/tools/host_test/test_fast_plan.cpp" \
    -o "$OUT/test_fast_plan"
"$OUT/test_fast_plan"

# 迷路の保存（走行中の追記）。フラッシュは試験の中の偽物。capture/applyToSolver のためにソルバーの core を使う
c++ -std=c++20 -Wall -Wextra -O1 \
    -I "$ROOT/Core/Inc" -I "$ROOT/external/etl/include" -I "$SOLVER/core" \
    "$SOLVER"/core/*.cpp "$ROOT/Core/Src/app/maze_store.cpp" "$ROOT/tools/host_test/test_maze_store.cpp" \
    -o "$OUT/test_maze_store"
"$OUT/test_maze_store"

# 前壁の読み落とし・自己位置のずれの手がかり（700_s1100_4 の再現）。ソルバー・先読み・換算表を使う
c++ -std=c++20 -Wall -Wextra -O1 \
    -I "$ROOT/Core/Inc" -I "$ROOT/external/etl/include" -I "$OUT/generated" -I "$SOLVER/core" "${SOLVER_INC[@]}" \
    "${SOLVER_SRC[@]}" "$ROOT/Core/Src/app/search_lookahead.cpp" "$ROOT/Core/Src/common/front_correction.cpp" \
    "$ROOT/tools/host_test/test_position_loss.cpp" \
    -o "$OUT/test_position_loss"
"$OUT/test_position_loss"
