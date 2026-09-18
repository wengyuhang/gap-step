#!/usr/bin/env bash
set -euo pipefail
method_dir="$(cd "$(dirname "$0")/.." && pwd)"
repo="$(cd "$method_dir/../.." && pwd)"
source_root="$repo/复现/TOGT-Planner-reproduction/source"
build_dir="$method_dir/native/build"
mkdir -p "$build_dir"

g++ -std=c++17 -O3 -DNDEBUG -fPIC -shared \
  "$repo/convex_timevarying_window/togt/native/togt_analytic.cpp" \
  "$source_root/src/system/quadrotor_manifold.cpp" \
  "$source_root/src/system/quadrotor_params.cpp" \
  "$source_root/src/planner/traj_params.cpp" \
  "$source_root/src/base/parameter_base.cpp" \
  "$source_root/src/type/quad_state.cpp" \
  "$source_root/src/type/command.cpp" \
  "$source_root/src/rotation/rotation_utils.cpp" \
  "$source_root/src/math/math_utils.cpp" \
  -I"$source_root/include" \
  -I"$repo/复现/TOGT-Planner-reproduction/deps/eigen-3.4.0" \
  -I"$source_root/build/_deps/rapidjson-src/include" \
  -o "$build_dir/libtogt_analytic.so"

g++ -std=c++17 -O3 -DNDEBUG \
  "$method_dir/native/global_corridor_togt.cpp" \
  -I"$source_root/include" \
  -I"$repo/复现/TOGT-Planner-reproduction/deps/eigen-3.4.0" \
  -L"$source_root/build" -ldrolib \
  -L"$build_dir" -ltogt_analytic \
  -Wl,-rpath,"$build_dir" \
  -o "$build_dir/global_corridor_togt"
