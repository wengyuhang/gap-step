"""Identity-based passage deadlines, independent of horizon numbering."""
import numpy as np

TOLERANCE_S = .02


def passage_map(plan):
    if plan is None: return {}
    return {plan.first_gate+i:float(t) for i,t in enumerate(plan.crossing_times[:plan.gate_count])}


def shared_deadlines(incumbent, ids):
    old = passage_map(incumbent)
    return np.array([old.get(wi,np.inf)+TOLERANCE_S for wi in ids])


def comparison(incumbent,candidate):
    old,new = passage_map(incumbent),passage_map(candidate)
    return [dict(gate=wi,old_time_s=old[wi],new_time_s=new[wi],delay_s=new[wi]-old[wi])
            for wi in sorted(old.keys() & new.keys())]
