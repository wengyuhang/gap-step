#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
casadi=/home/jack/anaconda3/envs/wyh/lib/python3.11/site-packages/casadi
togt='/home/jack/wyh/时变窗口/静态凸窗口/复现/TOGT-Planner-ICRA2024/include'
src=source_snapshot
tmp=$(mktemp -d)
trap 'rm -f -- "$tmp/slsqp.o"; rmdir -- "$tmp"' EXIT
gfortran -O3 -std=legacy -frecursive -c "$src/vendor/slsqp/slsqp_optmz.f" -o "$tmp/slsqp.o"
c++ -std=gnu++17 -O3 -ffp-contract=off -I"$src/include" -I"$src/vendor/native_minco_pv" -I"$casadi/include" -I"$casadi/include/eigen3" -I"$togt" \
  "$src/src/sip_planner.cpp" "$src/vendor/native_minco_pv/minco_pv.cpp" "$tmp/slsqp.o" \
  -Wl,--disable-new-dtags -Wl,-rpath,"$casadi:$casadi/../numpy.libs" "$casadi/libcasadi.so" \
  "$casadi/../numpy.libs/libopenblas64_p-r0-0cf96a72.3.23.dev.so" -lgfortran -lquadmath -o bin/sip_planner
c++ -std=gnu++17 -O3 -ffp-contract=off -I"$src/include" -I"$src/vendor/native_minco_pv" -I"$casadi/include/eigen3" -I"$togt" \
  audit.cpp "$src/vendor/native_minco_pv/minco_pv.cpp" -o bin/audit
