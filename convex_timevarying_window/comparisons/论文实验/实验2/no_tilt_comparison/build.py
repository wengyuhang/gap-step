"""Atomically rebuild the two native libraries from pinned TOGT sources."""
from pathlib import Path
import subprocess

import casadi

from ..soft_dynamics import native


def build():
    vendor = native.HERE/'vendor'
    eigen = Path(casadi.__file__).parent/'include/eigen3'
    target = native.NATIVE/'build/libtogt_x500.so'
    temporary = target.with_suffix('.new.so')
    sources = sorted((vendor/'src').rglob('*.cpp'))
    assert len(sources) == 8
    command = ['g++', '-std=c++17', '-O3', '-DNDEBUG', '-shared', '-fPIC',
        '-I'+str(eigen), '-I'+str(vendor/'include'),
        str(native.NATIVE/'togt_x500.cpp'), *map(str, sources), '-o', str(temporary)]
    subprocess.run(command, check=True)
    temporary.replace(target)
    previous = native.LIBRARY
    try:
        native.LIBRARY = previous.with_suffix('.new.so')
        native.build()
        native.LIBRARY.replace(previous)
    finally:
        native.LIBRARY = previous


if __name__ == '__main__':
    build()
