"""x500 dynamic TOGT with analytic free-terminal-state gradients."""
import ctypes
from pathlib import Path
import subprocess
from convex_dynamic_seven_window_gazebo.x500_togt.native_backend import (
    AnalyticTOGTCore as X500Core, TOGT,
)
from convex_timevarying_window.togt.native_backend import AnalyticTOGTCore as OriginalCore

HERE = Path(__file__).resolve().parent


def build_tail():
    source = HERE / "native/togt_tail.cpp"
    shared = HERE.parents[1] / "convex_dynamic_seven_window_gazebo/x500_togt/native/togt_x500.cpp"
    library = HERE / "native/build/libv3_togt.so"
    if library.exists() and library.stat().st_mtime >= max(source.stat().st_mtime, shared.stat().st_mtime):
        return library
    library.parent.mkdir(parents=True, exist_ok=True)
    names = ["system/quadrotor_manifold.cpp", "system/quadrotor_params.cpp",
             "planner/traj_params.cpp", "base/parameter_base.cpp", "type/quad_state.cpp",
             "type/command.cpp", "rotation/rotation_utils.cpp", "math/math_utils.cpp"]
    subprocess.run(["g++", "-std=c++17", "-O3", "-DNDEBUG", "-fPIC", "-shared",
                    f"-I{TOGT / 'include'}", f"-I{TOGT.parent / 'deps/eigen-3.4.0'}",
                    f"-I{TOGT / 'build/_deps/rapidjson-src/include'}", str(source),
                    *(str(TOGT / "src" / n) for n in names), "-o", str(library)], check=True)
    return library


class Core(X500Core):
    value_and_gradient_with_tail = OriginalCore.value_and_gradient_with_tail

    def __init__(self):
        super().__init__()
        self.tail_library = ctypes.CDLL(str(build_tail()))
        self.tail_function = self.tail_library.v3_objective_tail
        pointer = ctypes.POINTER(ctypes.c_double)
        self.tail_function.argtypes = [ctypes.c_int] + [pointer]*8 + [ctypes.c_char_p, ctypes.c_int]
        self.tail_function.restype = ctypes.c_int
