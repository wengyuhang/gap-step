import json
import numpy as np
from convex_timevarying_window.comparisons.large_window_sweep import audit as parent
from .scenario import build_case,LOWER,UPPER


def audit(path):
    scene=json.loads((path/'scene.json').read_text())
    assert scene['horizontal_spacing_scale']==3.
    previous=parent.build_case;parent.build_case=build_case
    try:g=parent.audit(path)
    finally:parent.build_case=previous
    pos=np.load(path/'truth_audit.npz')['positions']
    g['workspace_center_margin_m']=float(min(np.min(pos-LOWER),np.min(np.array(UPPER)-pos)))
    g['main_passed']=bool(g['physical_success'] and g['sampled_envelope_passed']
        and g['workspace_center_margin_m']>=0 and g['exit_before_next_entry']
        and g['no_reverse_after_entry'] and g['truth_returned'] and g['realtime_passed'])
    return g
