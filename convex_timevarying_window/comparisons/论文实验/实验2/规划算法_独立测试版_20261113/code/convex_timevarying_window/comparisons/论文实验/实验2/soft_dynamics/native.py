"""Arbitrary-polynomial access to the existing native x500 TOGT penalty."""
import ctypes
from pathlib import Path
import subprocess

import casadi
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[4]
NATIVE = ROOT/'convex_dynamic_seven_window_gazebo/x500_togt/native'
LIBRARY = HERE/'native/build/libpolynomial_penalty.so'


def build():
    LIBRARY.parent.mkdir(exist_ok=True)
    eigen = Path(casadi.__file__).parent/'include/eigen3'
    command = ['g++', '-std=c++17', '-O3', '-DNDEBUG', '-shared', '-fPIC',
        '-I'+str(eigen), '-I'+str(HERE/'vendor/include'), '-I'+str(NATIVE),
        str(HERE/'native/polynomial_penalty.cpp'), '-L'+str(NATIVE/'build'), '-ltogt_x500',
        '-Wl,-rpath,'+str(NATIVE/'build'), '-o', str(LIBRARY)]
    subprocess.run(command, check=True)


class Penalty:
    def __init__(self):
        if not LIBRARY.exists():
            build()
        self.library = ctypes.CDLL(str(LIBRARY))
        pointer = ctypes.POINTER(ctypes.c_double)
        self.function = self.library.togt_polynomial_penalty_gradient
        self.function.argtypes = [ctypes.c_int, pointer, pointer, pointer, pointer, pointer,
            ctypes.c_char_p, ctypes.c_int]
        self.function.restype = ctypes.c_int

    def evaluate(self, coefficients, durations):
        coefficients = np.ascontiguousarray(coefficients, dtype=np.float64)
        durations = np.ascontiguousarray(durations, dtype=np.float64)
        if coefficients.shape != (len(durations), 8, 3):
            raise ValueError('Expected normalized degree-seven coefficients')
        cost = np.zeros(1)
        gradient = np.empty_like(coefficients)
        time_gradient = np.empty_like(durations)
        error = ctypes.create_string_buffer(512)
        pointer = ctypes.POINTER(ctypes.c_double)
        raw = lambda value: value.ctypes.data_as(pointer)
        code = self.function(len(durations), raw(coefficients), raw(durations), raw(cost),
            raw(gradient), raw(time_gradient), error, len(error))
        if code:
            raise RuntimeError(error.value.decode(errors='replace'))
        return float(cost[0]), gradient, time_gradient
