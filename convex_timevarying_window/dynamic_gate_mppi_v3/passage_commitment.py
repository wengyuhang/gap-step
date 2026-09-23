"""Phase agreement at ACTUAL activation, separate from predicted progress."""
from enum import IntEnum


class Phase(IntEnum):
    APPROACH=0
    ENTERED_PRE_CENTER=1
    ENTERED_POST_CENTER=2
    CLEARED=3


def activation_phase_matches(progress,state,now,expected_first,expected_phase):
    first=progress.first;track=progress.track;cfg=progress.config
    slab=cfg.radius+cfg.error_margin+cfg.frame_thickness
    if expected_first<first or expected_first>first+1:return False
    if expected_first>first:
        window=track.windows[track.order[first]]
        _,z=window.world_to_local(state[:3],now)
        if z*progress.signs[first]<=slab:return False
    if expected_first>=len(track.order):return True
    window=track.windows[track.order[expected_first]]
    _,z=window.world_to_local(state[:3],now);z*=progress.signs[expected_first]
    actual=(Phase.APPROACH if z<=-slab else Phase.ENTERED_PRE_CENTER if z<0
            else Phase.ENTERED_POST_CENTER if z<=slab else Phase.CLEARED)
    return int(actual)==int(expected_phase)
