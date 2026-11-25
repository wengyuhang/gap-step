"""Canonical stationary auxiliary problem, independent of irrelevant frequency."""
import copy
from dataclasses import fields,replace
from . import continuation as common

_static_types={}


def amplitude_track(target,factor):
    if not 0.<=factor<=1.:raise ValueError('continuation factor outside [0,1]')
    track=copy.deepcopy(target)
    for w,b in zip(track.windows,target.windows):
        if factor==0.:
            typ=type(b.motion)
            if typ not in _static_types:
                class StationaryMotion(typ):
                    @property
                    def translation_period(self):return float('inf')
                _static_types[typ]=StationaryMotion
            values={f.name:getattr(b.motion,f.name) for f in fields(b.motion)}
            values.update(amplitude=0.,frequency=0.)
            w.motion=_static_types[typ](**values)
        else:w.motion=replace(b.motion,amplitude=factor*b.motion.amplitude)
    return track


class ContinuationPlanner(common.ContinuationPlanner):
    def solve(self,*args,**kwargs):
        previous=common.amplitude_track;common.amplitude_track=amplitude_track
        try:
            plan,result=super().solve(*args,**kwargs)
            result.update(static_auxiliary_frequency_zero=True,static_effective_period_infinite=True)
            return plan,result
        finally:common.amplitude_track=previous
