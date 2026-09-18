#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
CASADI="$ROOT/.runtime/togt_mpc_python/casadi"
MAVLINK="$ROOT/.runtime/mavlink_c_v2"
SOURCE="$ROOT/cpp_mpc/controller.cpp"
OUTPUT="$ROOT/cpp_mpc/controller"
COMMON_FLAGS=(
  -O3 -DNDEBUG -std=c++17 -D_GLIBCXX_USE_CXX11_ABI=0
  -Wno-address-of-packed-member
  -I"$CASADI/include" -I"$MAVLINK"
)

# Keep the editor's indexer on the exact flags used by the successful build.
python3 - "$ROOT/cpp_mpc/compile_commands.json" "$ROOT/cpp_mpc" \
  "$SOURCE" "$CASADI/include" "$MAVLINK" <<'PY'
import json
import sys
from pathlib import Path

output, directory, source, casadi_include, mavlink_include = sys.argv[1:]
arguments = [
    "/usr/bin/g++", "-O3", "-DNDEBUG", "-std=c++17",
    "-D_GLIBCXX_USE_CXX11_ABI=0", "-Wno-address-of-packed-member",
    f"-I{casadi_include}", f"-I{mavlink_include}", "-c", source,
]
Path(output).write_text(json.dumps([{
    "directory": directory,
    "file": source,
    "arguments": arguments,
}], ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
PY

g++ "${COMMON_FLAGS[@]}" "$SOURCE" -L"$CASADI" -Wl,-rpath,"$CASADI" \
  -lcasadi -pthread -o "$OUTPUT"
