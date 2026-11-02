import numpy as np
from flint import arb,ctx

from convex_timevarying_window.dynamic_passage_velocity_minco.hard_dynamics import flat_actuators
from nonconvex_timevarying_window.comparisons.asymmetric_dumbbell_sc_minco.global_optimality.continuous_dynamics import Tape,controls,enclosure,split


def test_arb_actuator_enclosure_contains_perturbed_inputs():
    ctx.prec=128
    center=np.array([[1.,5.,.2],[.3,1.,.5],[1.,-.4,.2],[.5,.2,-.1]])
    box=[[arb(float(v),.0001) for v in row] for row in center]
    bounds=Tape().evaluate(box)
    rng=np.random.default_rng(82)
    for _ in range(8):
        inp=center+rng.uniform(-.00009,.00009,center.shape)
        values=flat_actuators(*[row for row in inp])
        for bb,vv in zip(bounds,values):
            for b,v in zip(bb,np.asarray(vv).ravel()):
                assert float(b.lower())<=v<=float(b.upper())


def test_arb_bernstein_bounds_after_subdivision():
    rng=np.random.default_rng(83);cc=rng.normal(size=(8,3));duration=1.37
    ctrl=controls(cc,duration,2)
    left,right=split(ctrl,arb(.5));bounds=enclosure(right)
    for u in np.linspace(.5,1.,17):
        value=sum(cc[k]*k*(k-1)*u**(k-2)/duration**2 for k in range(2,8))
        for b,v in zip(bounds,value):
            assert float(b.lower())<=v<=float(b.upper())
