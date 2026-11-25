"""Isolated loading of the frozen backend for expanded-window tests."""
from pathlib import Path
import sys,importlib.util
HERE=Path(__file__).resolve().parent
PARENT=HERE.parent
FROZEN=Path('/tmp/first_formal_soft_reproduction_code_20261112')

def load():
    sys.path.insert(0,str(FROZEN));sys.path.insert(1,str(PARENT))
    spec=importlib.util.spec_from_file_location('expanded_track_scene',HERE/'scenario.py')
    scene=importlib.util.module_from_spec(spec);sys.modules[spec.name]=scene;spec.loader.exec_module(scene)
    from convex_timevarying_window.comparisons.论文实验.实验2.higher_swing import run as higher
    from convex_timevarying_window.comparisons.论文实验.实验2.soft_dynamics import run as common,best_center_release
    from convex_timevarying_window.comparisons.论文实验.实验2.wang2023 import rollout
    higher.scenario=common.scenario=best_center_release.scenario=rollout.scenario=scene
    return scene,higher,common,best_center_release,rollout
