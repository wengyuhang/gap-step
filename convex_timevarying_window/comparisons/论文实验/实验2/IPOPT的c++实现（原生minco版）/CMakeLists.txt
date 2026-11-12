cmake_minimum_required(VERSION 3.18)
project(ipopt_native_minco LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CASADI_ROOT "" CACHE PATH "CasADi C++ SDK containing include and libcasadi.so")
if(NOT EXISTS "${CASADI_ROOT}/include/casadi/casadi.hpp")
  message(FATAL_ERROR "Set CASADI_ROOT to the installed CasADi C++ SDK")
endif()
set(NUMPY_BLAS "${CASADI_ROOT}/../numpy.libs/libopenblas64_p-r0-0cf96a72.3.23.dev.so"
    CACHE FILEPATH "Original ILP64 OpenBLAS used by the numerical geometry oracle")
if(NOT EXISTS "${NUMPY_BLAS}")
  message(FATAL_ERROR "Set NUMPY_BLAS to the original ILP64 OpenBLAS shared library")
endif()

file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/cases")
file(ARCHIVE_EXTRACT INPUT "${CMAKE_CURRENT_SOURCE_DIR}/assets/cases.tar.gz"
     DESTINATION "${CMAKE_CURRENT_BINARY_DIR}/cases")

add_library(native_minco_pv STATIC vendor/minco_pv/minco_pv.cpp)
target_include_directories(native_minco_pv PUBLIC
  vendor/minco_pv vendor/togt/include "${CASADI_ROOT}/include/eigen3")
target_compile_options(native_minco_pv PRIVATE -O3 -ffp-contract=off)

add_executable(sip_planner src/sip_planner.cpp)
add_executable(matrix src/matrix.cpp)
add_executable(restored_inputs src/restored_inputs.cpp)
add_executable(restored_experiment src/restored_experiment.cpp)
add_executable(restored_release_trial src/restored_release_trial.cpp)
add_executable(mesh_check src/mesh_check.cpp)
add_executable(numpy_frame_check tests/numpy_frame_check.cpp)

foreach(target sip_planner matrix restored_inputs restored_experiment restored_release_trial mesh_check numpy_frame_check)
  target_include_directories(${target} PRIVATE include vendor/togt/include "${CASADI_ROOT}/include/eigen3")
  target_compile_options(${target} PRIVATE -O3 -ffp-contract=off)
endforeach()

target_link_libraries(sip_planner PRIVATE native_minco_pv "${CASADI_ROOT}/libcasadi.so" "${NUMPY_BLAS}")
target_include_directories(sip_planner PRIVATE "${CASADI_ROOT}/include")
target_link_libraries(numpy_frame_check PRIVATE "${NUMPY_BLAS}")
foreach(target sip_planner numpy_frame_check)
  set_target_properties(${target} PROPERTIES BUILD_RPATH "${CASADI_ROOT};${CASADI_ROOT}/../numpy.libs")
  target_link_options(${target} PRIVATE -Wl,--disable-new-dtags)
endforeach()

include(CTest)
if(BUILD_TESTING)
  file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/parity_fixtures")
  file(ARCHIVE_EXTRACT INPUT "${CMAKE_CURRENT_SOURCE_DIR}/assets/parity_fixtures.tar.gz"
       DESTINATION "${CMAKE_CURRENT_BINARY_DIR}/parity_fixtures")
  add_executable(restored_unit tests/restored_unit.cpp)
  target_include_directories(restored_unit PRIVATE include "${CASADI_ROOT}/include")
  target_compile_options(restored_unit PRIVATE -O3 -ffp-contract=off)
  target_link_libraries(restored_unit PRIVATE native_minco_pv "${CASADI_ROOT}/libcasadi.so")
  set_target_properties(restored_unit PROPERTIES BUILD_RPATH "${CASADI_ROOT}")
  foreach(method ours center)
    add_test(NAME native_${method}_kernel_and_gradient
      COMMAND restored_unit
        "${CMAKE_CURRENT_BINARY_DIR}/cases/inputs/a25_f6_${method}"
        "${CMAKE_CURRENT_BINARY_DIR}/cases/assets/a25_f6/scene.json"
        "${CMAKE_CURRENT_BINARY_DIR}/parity_fixtures/legacy"
        "${CMAKE_CURRENT_BINARY_DIR}/native_${method}_validation.json")
    set_tests_properties(native_${method}_kernel_and_gradient PROPERTIES
      ENVIRONMENT "OPENBLAS_NUM_THREADS=1;OMP_NUM_THREADS=1;MKL_NUM_THREADS=1")
  endforeach()
endif()
