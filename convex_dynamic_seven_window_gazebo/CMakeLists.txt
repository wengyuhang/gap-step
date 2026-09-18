cmake_minimum_required(VERSION 3.16)
project(convex_dynamic_gate_motion LANGUAGES CXX)

find_package(gz-cmake3 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-sim8 REQUIRED)

add_library(PeriodicGateMotion SHARED src/PeriodicGateMotion.cc)
target_compile_features(PeriodicGateMotion PRIVATE cxx_std_17)
target_link_libraries(PeriodicGateMotion PRIVATE
  gz-plugin2::gz-plugin2
  gz-sim8::gz-sim8)

add_library(TrajectoryReplay SHARED src/TrajectoryReplay.cc)
target_compile_features(TrajectoryReplay PRIVATE cxx_std_17)
target_link_libraries(TrajectoryReplay PRIVATE
  gz-plugin2::gz-plugin2
  gz-sim8::gz-sim8)
