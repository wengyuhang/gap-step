from dataclasses import asdict
import casadi as ca
import numpy as np
from convex_timevarying_window.checked_soft_racing.planner import clearance_penalty
from convex_timevarying_window.comparisons.original_fixed_plane_deadline_no_postcheck.planner import Config as Old
from convex_timevarying_window.comparisons.original_fixed_plane_deadline_soft_conservative.config import Config
from convex_timevarying_window.comparisons.original_fixed_plane_deadline_soft_conservative import flight


def test_only_two_declared_fields_change():
    old,new=asdict(Old()),asdict(Config())
    assert {k:(old[k],v) for k,v in new.items() if v!=old[k]}=={
        'soft_clearance_buffer':(.15,.25),'soft_clearance_weight':(1.,10.)}
    assert new['tracking_margin']+new['soft_clearance_buffer']==.6


def test_penalty_and_inward_gradient_dominate_at_every_distance():
    old,new=Old(),Config()
    h=old.radius+old.frame_thickness+old.tracking_margin
    po,pn=h+old.soft_clearance_buffer,h+new.soft_clearance_buffer
    # Analytic sufficient condition for ALL squared distances, not just samples.
    assert pn>po
    assert new.soft_clearance_weight/(pn*pn-h*h)**3 >= old.soft_clearance_weight/(po*po-h*h)**3
    q=ca.SX.sym('q')
    fo,fn=clearance_penalty(q,old),clearance_penalty(q,new)
    f=ca.Function('penalties',[q],[ca.vertcat(fo,fn,ca.jacobian(fo,q),ca.jacobian(fn,q))])
    for distance in np.linspace(0,pn+.5,1001):
        a,b,da,db=np.asarray(f(distance**2)).ravel()
        assert b>=a-1e-10
        assert -db>=-da-1e-10
    assert float(f(h*h)[1])==10.


def test_flight_uses_existing_worker_and_new_configuration():
    from convex_timevarying_window.comparisons.original_fixed_plane_deadline_no_postcheck.adapter import Worker
    assert flight.Worker is Worker
    assert flight.Config is Config
