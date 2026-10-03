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
