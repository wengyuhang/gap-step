"""C ABI binding for the released TOGT analytic-gradient implementation."""

from __future__ import annotations

import ctypes
from pathlib import Path
import subprocess

import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE.parent
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


__all__ = ["AnalyticTOGTCore", "build_native", "LIBRARY"]
