"""Two-factor, whole-course motion conditions on the accepted TOGT layout."""
from dataclasses import asdict, dataclass, replace
import numpy as np
from ..togt_course import scenario as original

ROOT=original.ROOT; ASSETS=original.ASSETS
START=original.START; ANCHORS=original.ANCHORS; pose=original.pose
BASE_FREQUENCIES=(.30,.40,.50,.60,.55,.45,.35)
AMPLITUDE_RATIOS=(.25,.50,.75)
FREQUENCY_FACTORS=(1.,3.,6.)
CONDITIONS={'mixed_base':(.25,1.), 'a25_f3':(.25,3.), 'a25_f6':(.25,6.),
            'a50_f1':(.50,1.), 'a50_f3':(.50,3.), 'a50_f6':(.50,6.),
            'a75_f1':(.75,1.), 'a75_f3':(.75,3.), 'a75_f6':(.75,6.)}
LEVELS=('static',*CONDITIONS)


@dataclass(frozen=True)
class Case:
    layout:str='togt_whole'
    level:str='static'
    def __post_init__(self):
        if self.layout!='togt_whole' or self.level not in LEVELS:
            raise ValueError('unknown whole-course condition')


def horizontal_width(w):
    return float(2*w.aperture.radius if w.aperture.kind=='circle'
                 else np.ptp(w.aperture.physical_vertices[:,1]))


def profile(level):
    if level not in LEVELS:raise ValueError(level)
    rho,scale=(0.,1.) if level=='static' else CONDITIONS[level]
    tr,_=original.build_case(original.Case())
    return tuple((scale*f,rho*horizontal_width(w)) for f,w in zip(BASE_FREQUENCIES,tr.windows))


def build_case(case):
    tr,_=original.build_case(original.Case())
    for w,(f,a) in zip(tr.windows,profile(case.level)):
        w.motion=replace(w.motion,frequency=f,amplitude=a)
    return replace(tr,name='togt_whole_'+case.level),None


def manifest(case):
    result=original.manifest(original.Case())
    tr,_=build_case(case)
    rho,scale=(0.,1.) if case.level=='static' else CONDITIONS[case.level]
    result.update(case=asdict(case),challenge_gates=list(range(1,8)),
        escalation='two-factor factorial: all seven amplitudes and all seven frequencies participate',
        amplitude_to_horizontal_opening_width=rho,frequency_factor=scale,
        base_frequencies_hz=list(BASE_FREQUENCIES),phase_rule='all initial phases zero',
        fixed_layout_across_conditions=True)
    for i,(row,w) in enumerate(zip(result['windows'],tr.windows)):
        f=w.motion.frequency;a=w.motion.amplitude
        peak=2*np.pi*f*(a if w.motion.kind=='translation' else w.motion.length*w.motion.angle)
        row.update(frequency_hz=f,amplitude_m=a,horizontal_opening_width_m=horizontal_width(w),
            amplitude_to_width=rho,base_frequency_hz=BASE_FREQUENCIES[i],
            peak_center_speed_mps=float(peak))
    speeds=[w['peak_center_speed_mps'] for w in result['windows']]
    result['peak_center_speed_range_mps']=[min(speeds),max(speeds)]
    result['amplitude_range_m']=[min(a for _,a in profile(case.level)),max(a for _,a in profile(case.level))]
    result['frequency_range_hz']=[min(f for f,_ in profile(case.level)),max(f for f,_ in profile(case.level))]
    return result
