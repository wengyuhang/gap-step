from types import SimpleNamespace
import numpy as np
from convex_timevarying_window.gs_mppi.direct_ctbr.safety import filter_current,SLEW_PER_SEND
from convex_timevarying_window.gs_mppi.direct_ctbr.evaluation import assess


def test_first_command_search_recovers_when_local_projection_fails():
    class Engine:
        config=SimpleNamespace(min_rotor_thrust=.19,max_rotor_thrust=8.54858)
        def rollout(self,state,controls,reference,poses,first,phase,mode,dt,
                    gpu,projection_iterations,previous_command):
            command=controls[0,0]
            ok=command[0]<=24.000001
            metrics=np.array([0.,0. if ok else 4.,.1 if ok else -.1,
                              first,phase,0.,1. if ok else 0.,1. if ok else 0.])
            return metrics,np.empty((2 if ok else 1,17)),command[None] if ok else np.empty((0,4))
    state=np.zeros(17)
    check,action,attempts,fallback=filter_current(
        Engine(),state,np.array([30.,.2,0.,0.]),0.,0,0,
        poses=np.zeros((5,1,12)),reference=np.zeros((2,9)))
    assert check[1]==0 and fallback and attempts>1
    assert np.isclose(action[0],24.)


def test_publication_slew_is_respected_when_a_safe_command_fits():
    class Engine:
        config=SimpleNamespace(min_rotor_thrust=.19,max_rotor_thrust=8.54858)
        def rollout(self,state,controls,reference,poses,first,phase,mode,dt,
                    gpu,projection_iterations,previous_command):
            command=controls[0,0]
            metrics=np.array([0.,0.,1.,first,phase,0.,1.,1.])
            return metrics,np.empty((2,17)),command[None]
    previous=np.array([20.,0.,0.,0.]);proposal=np.array([29.,2.,-2.,1.])
    check,action,attempts,fallback=filter_current(
        Engine(),np.zeros(17),proposal,0.,0,0,previous=previous,
        poses=np.zeros((5,1,12)),reference=np.zeros((2,9)))
    assert check[1]==0 and attempts==1 and not fallback
    assert np.all(np.abs(action-previous)<=SLEW_PER_SEND+1e-10)


def test_slew_relaxes_only_when_needed_for_interval_safety():
    class Engine:
        config=SimpleNamespace(min_rotor_thrust=.19,max_rotor_thrust=8.54858)
        def rollout(self,state,controls,reference,poses,first,phase,mode,dt,
                    gpu,projection_iterations,previous_command):
            command=controls[0,0]
            safe=command[1]>=1.
            metrics=np.array([0.,0. if safe else 16.,1. if safe else -1.,
                              first,phase,0.,1. if safe else 0.,1. if safe else 0.])
            return metrics,np.empty((2 if safe else 1,17)),command[None] if safe else np.empty((0,4))
    previous=np.array([20.,0.,0.,0.]);proposal=np.array([20.,2.,0.,0.])
    check,action,attempts,fallback=filter_current(
        Engine(),np.zeros(17),proposal,0.,0,0,previous=previous,
        poses=np.zeros((5,1,12)),reference=np.zeros((2,9)))
    assert check[1]==0 and 1.<=action[1]<=4*SLEW_PER_SEND[1]
    assert attempts==3 and not fallback


def test_race_time_only_exists_after_strict_safe_realtime_completion():
    result=dict(completed=True,collision_detected=False,
                crossings=[dict(gate=i,time=float(i+1),local_point=[0.,0.]) for i in range(7)],flight_time_s=36.,
                motion_rate=10,translation_scale=4,rotation_scale=1,phase=0.,
                solves=[dict(cycle_ms=60.,future_warning=False,activation_filter_fallback=False)],
                reference_updates=[dict(solve_ms=70.,accepted=True)])
    audit=dict(min_sampled_body_clearance_m=.2,all_six_workspace_faces_passed=True,
               strict_sampled_success=True,togt=dict(deadline_rejections=0))
    sender=np.zeros((1,12));sender[0,1]=3.
    baseline=dict(rate=10,translation_scale=4,rotation_scale=1,phase=0.,
                  realtime_success=True,flight_time_s=50.)
    good=assess(result,audit,sender,baseline)
    assert good['realtime_success'] and good['race_time_s']==36.
    assert good['baseline']['time_saved_s']==14.
    bad=assess({**result,'completed':False,'crossings':result['crossings'][:4],
                'flight_time_s':None,'failure':'no_action'},audit,sender,baseline)
    assert not bad['realtime_success'] and bad['race_time_s'] is None
