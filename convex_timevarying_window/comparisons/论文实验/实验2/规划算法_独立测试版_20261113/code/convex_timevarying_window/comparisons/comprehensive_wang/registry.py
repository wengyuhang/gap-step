"""User-approved factorial comparison, one flight per cell (no repeats)."""
from dataclasses import dataclass,asdict
import numpy as np

FREQUENCIES=(.4,.7,1.)
AMPLITUDES=(.5,1.2,2.)
PHASE_DEGREES=(0,)
HARD=((.7,2.),(1.,1.2),(1.,2.))


@dataclass(frozen=True)
class Trial:
    stage:str
    method:str
    frequency:float
    amplitude:float
    phase:float
    phase_degrees:int

    @property
    def name(self):
        return f'{self.stage}_{self.method}_f{self.frequency:g}_a{self.amplitude:g}_p{self.phase_degrees:03d}'

    @property
    def pair(self):
        return f'f{self.frequency:g}_a{self.amplitude:g}_p{self.phase_degrees:03d}'


def trials():
    rows=[];pair=0
    for f in FREQUENCIES:
        for a in AMPLITUDES:
            for deg in PHASE_DEGREES:
                methods=('ours','wang') if pair%2==0 else ('wang','ours')
                for method in methods:rows.append(Trial('main',method,f,a,float(np.radians(deg)),deg))
                pair+=1
    for f,a in HARD:
        for deg in PHASE_DEGREES:
            for method in ('center','no_soft','no_dense'):
                rows.append(Trial('ablation',method,f,a,float(np.radians(deg)),deg))
    assert len(rows)==27 and len({r.name for r in rows})==27
    return tuple(rows)
