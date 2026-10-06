from dataclasses import replace
import numpy as np
from convex_timevarying_window.dynamic_gate_mppi_v3.config import Config
from convex_timevarying_window.dynamic_gate_mppi_v3.geometry import Geometry
from convex_timevarying_window.dynamic_gate_mppi_v3.tests.test_passage import single_gate


def test_tangential_velocity_does_not_consume_plane_normal_clearance():
    cfg=replace(Config(),axis_workspace_bounds=True)
    p=np.array([[-10.,21.635,4.],[-10.,21.635,4.032]])
    v=np.tile([0.,0.,2.],(2,1));t=np.array([0.,.016])
    a=Geometry(single_gate(),cfg).interval_clearances(p,v,t)
    b=Geometry(single_gate(),replace(cfg,axis_workspace_bounds=False)).interval_clearances(p,v,t)
    assert a.min()>0 and b.min()<0


def test_normal_bound_still_rejects_same_margin_violation():
    cfg=replace(Config(),axis_workspace_bounds=True)
    p=np.array([[-10.,21.66,4.],[-10.,21.66,4.]])
    assert Geometry(single_gate(),cfg).interval_clearances(p,np.zeros_like(p),[0.,.01]).min()<0
