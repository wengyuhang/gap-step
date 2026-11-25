cmake_minimum_required(VERSION 3.18)
project(multimodal_initialization LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_subdirectory("../heuristic_initialization" "shared")
set(NATIVE_PACKAGE "${CMAKE_CURRENT_SOURCE_DIR}/../../SLSQP的c++实现")
add_executable(generate_multimodal src/generate_multimodal.cpp)
target_include_directories(generate_multimodal PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/../pareto_initialization/src"
  "${CASADI_ROOT}/include" "${CASADI_ROOT}/include/eigen3"
  "${NATIVE_PACKAGE}/include" "${NATIVE_PACKAGE}/vendor/togt/include")
target_link_libraries(generate_multimodal PRIVATE libcmaes::cmaes "${CASADI_ROOT}/libcasadi.so")
target_compile_options(generate_multimodal PRIVATE -O3 -ffp-contract=off)
set_target_properties(generate_multimodal PROPERTIES BUILD_RPATH "${CASADI_ROOT}")
enable_testing()
add_test(NAME multimodal_unit COMMAND generate_multimodal --unit)
