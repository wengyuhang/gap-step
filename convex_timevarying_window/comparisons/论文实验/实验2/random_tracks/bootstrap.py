"""Load the already reproduced frozen algorithms without another source copy."""
from pathlib import Path
import sys, importlib.util
HERE=Path(__file__).resolve().parent
FROZEN=Path('/tmp/first_formal_soft_reproduction_code_20261112')
def load():
    sys.path.insert(0,str(FROZEN))
    spec=importlib.util.spec_from_file_location('random_track_scene',HERE/'scenario.py')
    scenario=importlib.util.module_from_spec(spec);sys.modules[spec.name]=scenario;spec.loader.exec_module(scenario)
    from convex_timevarying_window.comparisons.论文实验.实验2.higher_swing import run as higher
    from convex_timevarying_window.comparisons.论文实验.实验2.soft_dynamics import run as common,best_center_release
    from convex_timevarying_window.comparisons.论文实验.实验2.wang2023 import rollout
    higher.scenario=common.scenario=best_center_release.scenario=rollout.scenario=scenario
    return scenario,higher,common,best_center_release,rollout
