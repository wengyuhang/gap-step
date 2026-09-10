"""C ABI binding for the released TOGT analytic-gradient implementation."""

from __future__ import annotations

import ctypes
from pathlib import Path
import subprocess

import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
TOGT = REPO / "复现" / "TOGT-Planner-reproduction" / "source"
SOURCE = HERE / "native" / "togt_analytic.cpp"
LIBRARY = HERE / "native" / "build" / "libtogt_analytic.so"


def build_native(force: bool = False) -> Path:
    if LIBRARY.is_file() and not force:
        return LIBRARY
    if not TOGT.is_dir():
        raise FileNotFoundError(f"released TOGT source is unavailable: {TOGT}")
    LIBRARY.parent.mkdir(parents=True, exist_ok=True)
    reproduction = TOGT.parent
    sources = [
        SOURCE,
        TOGT / "src/system/quadrotor_manifold.cpp",
        TOGT / "src/system/quadrotor_params.cpp",
        TOGT / "src/planner/traj_params.cpp",
        TOGT / "src/base/parameter_base.cpp",
        TOGT / "src/type/quad_state.cpp",
        TOGT / "src/type/command.cpp",
        TOGT / "src/rotation/rotation_utils.cpp",
        TOGT / "src/math/math_utils.cpp",
    ]
    command = [
        "g++", "-std=c++17", "-O3", "-DNDEBUG", "-fPIC", "-shared",
        f"-I{TOGT / 'include'}",
        f"-I{reproduction / 'deps/eigen-3.4.0'}",
        f"-I{TOGT / 'build/_deps/rapidjson-src/include'}",
        *(str(path) for path in sources),
        "-o", str(LIBRARY),
    ]
    subprocess.run(command, check=True)
    return LIBRARY


class AnalyticTOGTCore:
    def __init__(self, build_if_missing: bool = True):
        path = build_native() if build_if_missing else LIBRARY
        self.library = ctypes.CDLL(str(path))
        pointer = ctypes.POINTER(ctypes.c_double)
        self.function = self.library.togt_objective_gradient
        self.function.argtypes = [
            ctypes.c_int, pointer, pointer, pointer, pointer, pointer,
            pointer, pointer, ctypes.c_char_p, ctypes.c_int,
        ]
        self.function.restype = ctypes.c_int
        self.propagate_function = self.library.minco_propagate_gradient
        self.propagate_function.argtypes = [
            ctypes.c_int, pointer, pointer, pointer, pointer, pointer, pointer,
            pointer, pointer, ctypes.c_char_p, ctypes.c_int,
        ]
        self.propagate_function.restype = ctypes.c_int
        self.sample_dynamics_function = self.library.togt_sample_dynamics
        self.sample_dynamics_function.argtypes = [
            ctypes.c_int, pointer, pointer, ctypes.c_char_p, ctypes.c_int,
        ]
        self.sample_dynamics_function.restype = ctypes.c_int

    def value_and_gradient(self, head_pvaj, tail_pvaj, points, durations):
        arrays = [np.ascontiguousarray(value, dtype=np.float64) for value in
                  (head_pvaj, tail_pvaj, points, durations)]
        head, tail, locations, times = arrays
        if head.shape != (3, 4) or tail.shape != (3, 4):
            raise ValueError("boundary PVAJ states must have shape (3, 4)")
        if locations.shape != (len(times) - 1, 3):
            raise ValueError("points must have shape (piece_count - 1, 3)")
        cost = np.zeros(1, dtype=np.float64)
        grad_points = np.empty_like(locations)
        grad_times = np.empty_like(times)
        error = ctypes.create_string_buffer(512)
        pointer = ctypes.POINTER(ctypes.c_double)
        raw = lambda value: value.ctypes.data_as(pointer)
        status = self.function(
            len(times), raw(head), raw(tail), raw(locations), raw(times),
            raw(cost), raw(grad_points), raw(grad_times), error, len(error),
        )
        if status:
            raise RuntimeError(error.value.decode("utf-8", errors="replace"))
        return float(cost[0]), grad_points, grad_times

    def propagate_minco_gradient(
        self, head_pvaj, tail_pvaj, points, durations,
        partial_coefficients, partial_durations,
    ):
        arrays = [np.ascontiguousarray(value, dtype=np.float64) for value in (
            head_pvaj, tail_pvaj, points, durations,
            partial_coefficients, partial_durations,
        )]
        head, tail, locations, times, grad_coefficients, grad_times_in = arrays
        if head.shape != (3, 4) or tail.shape != (3, 4):
            raise ValueError("boundary PVAJ states must have shape (3, 4)")
        if locations.shape != (len(times) - 1, 3):
            raise ValueError("points must have shape (piece_count - 1, 3)")
        if grad_coefficients.shape == (len(times), 8, 3):
            grad_coefficients = np.ascontiguousarray(
                grad_coefficients.reshape(8 * len(times), 3)
            )
        if grad_coefficients.shape != (8 * len(times), 3):
            raise ValueError("partial coefficient gradient has the wrong shape")
        if grad_times_in.shape != times.shape:
            raise ValueError("partial duration gradient has the wrong shape")
        grad_points = np.empty_like(locations)
        grad_times = np.empty_like(times)
        error = ctypes.create_string_buffer(512)
        pointer = ctypes.POINTER(ctypes.c_double)
        raw = lambda value: value.ctypes.data_as(pointer)
        status = self.propagate_function(
            len(times), raw(head), raw(tail), raw(locations), raw(times),
            raw(grad_coefficients), raw(grad_times_in),
            raw(grad_points), raw(grad_times), error, len(error),
        )
        if status:
            raise RuntimeError(error.value.decode("utf-8", errors="replace"))
        return grad_points, grad_times

    def sample_dynamics(self, pvajs):
        """Evaluate PVAJS samples with the released C++ ``QuadManifold``."""
        states = np.ascontiguousarray(pvajs, dtype=np.float64)
        if states.ndim != 3 or states.shape[1:] != (5, 3):
            raise ValueError("pvajs must have shape (sample_count, 5, 3)")
        output = np.empty((len(states), 12), dtype=np.float64)
        error = ctypes.create_string_buffer(512)
        pointer = ctypes.POINTER(ctypes.c_double)
        raw = lambda value: value.ctypes.data_as(pointer)
        status = self.sample_dynamics_function(
            len(states), raw(states), raw(output), error, len(error),
        )
        if status:
            raise RuntimeError(error.value.decode("utf-8", errors="replace"))
        return {
            "speed": output[:, 0],
            "tilt": output[:, 1],
            "body_rate": output[:, 2:5],
            "collective_thrust": output[:, 5],
            "rotor_thrusts": output[:, 6:10],
            "regular_branch": output[:, 10].astype(bool),
            "instantaneous_penalty": output[:, 11],
        }


__all__ = ["AnalyticTOGTCore", "build_native", "LIBRARY"]
