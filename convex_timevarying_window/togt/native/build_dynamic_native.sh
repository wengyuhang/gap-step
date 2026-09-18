#!/usr/bin/env bash
set -euo pipefail
repo="$(cd "$(dirname "$0")/../../.." && pwd)"
source_root="$repo/复现/TOGT-Planner-reproduction/source"
native_build="$repo/convex_timevarying_window/togt/native/build"
mkdir -p "$native_build"
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
  -o "$native_build/libtogt_analytic.so"
g++ -std=c++17 -O3 -DNDEBUG \
  "$repo/convex_timevarying_window/togt/native/togt_dynamic_native.cpp" \
  -I"$source_root/include" \
  -I"$repo/复现/TOGT-Planner-reproduction/deps/eigen-3.4.0" \
  -L"$source_root/build" -ldrolib \
  -L"$native_build" -ltogt_analytic \
  -Wl,-rpath,"$native_build" \
  -o "$source_root/build/togt_dynamic_native"
