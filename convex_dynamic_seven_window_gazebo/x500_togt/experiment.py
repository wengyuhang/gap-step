#!/usr/bin/env python3
"""Replan the frozen seven-window course with the PX4 Gazebo x500 model."""
from __future__ import annotations

import argparse
from dataclasses import asdict
import json
from pathlib import Path
import time

import numpy as np

from convex_timevarying_window.geometry import ConvexAperture, PeriodicConvexWindow
from convex_timevarying_window.togt import experiment as baseline
from convex_timevarying_window.togt.native_objective import NativeJointTOGTObjective
from nonconvex_timevarying_window.sc_dynatogt.dynamics import DynamicLimits, ObjectiveWeights, PenaltyWeights, QuadrotorParameters
from nonconvex_timevarying_window.sc_dynatogt.environment import MotionProfile, SCWindowTrack
from nonconvex_timevarying_window.sc_dynatogt.optimizer import OptimizationConfig, _minimize_togt_lbfgs
from nonconvex_timevarying_window.sc_dynatogt.time_mapping import k_from_durations
from convex_dynamic_seven_window_gazebo.x500_togt.native_backend import AnalyticTOGTCore

HERE = Path(__file__).resolve().parent
X500_MASS = 2.0643076923076924
X500_INERTIA = np.array([0.02383948, 0.02394241, 0.04399995])
X500_ROTOR_ARM = 0.174
X500_MOMENT_CONSTANT = 0.016
X500_MAX_ROTOR_THRUST = 8.54858
PX4_CONTROLLER_MAX_VELOCITY = 12.0
X500_MAX_RATE_XY = np.deg2rad(220.0)
X500_MAX_RATE_Z = np.deg2rad(200.0)
X500_MAX_TILT = np.deg2rad(45.0)
# Union of the four official rotor collision boxes, projected into the gate plane.
X500_FRAME_RADIUS = 0.3859447197013662
MARGIN_FACTOR = 1.1
WINDOW_MARGIN = MARGIN_FACTOR * 2.0 * X500_FRAME_RADIUS
AUDIT_STEP = 0.001


def apertures():
    def regular(count, physical_radius):
        a = np.linspace(0, 2*np.pi, count, endpoint=False)
        dirs = np.column_stack((np.cos(a), np.sin(a)))
        return ConvexAperture("polygon", margin=WINDOW_MARGIN,
                              physical_vertices=physical_radius*dirs,
                              safe_vertices=(physical_radius-WINDOW_MARGIN)*dirs)
    def rectangle(hw, hh):
        signs = np.array([[-1.,-1.], [1.,-1.], [1.,1.], [-1.,1.]])
        return ConvexAperture("polygon", margin=WINDOW_MARGIN,
                              physical_vertices=signs*np.array([hw,hh]),
                              safe_vertices=signs*np.array([hw-WINDOW_MARGIN/2, hh-WINDOW_MARGIN/2]))
    return (rectangle(1.45,1.15), ConvexAperture("circle", radius=1.35, margin=WINDOW_MARGIN),
            regular(5,1.50), ConvexAperture("circle", radius=1.25, margin=WINDOW_MARGIN),
            regular(6,1.45), ConvexAperture("circle", radius=1.40, margin=WINDOW_MARGIN),
            rectangle(1.35,1.08))


def build_track():
    phases=(.20,-.60,.90,-.30,1.10,-1.00,.70)
    ta=np.array([[.18,.13,.10],[.14,.20,.12],[.20,.12,.15],[.16,.18,.11],[.13,.16,.18],[.19,.15,.13],[.15,.19,.16]])
    ra=np.deg2rad(np.array([[5,7,10],[7,5,9],[6,8,7],[8,6,10],[5,9,8],[7,8,6],[9,5,7]],float))
    windows=[]
    for i, aperture in enumerate(apertures()):
        motion=MotionProfile(translation_amplitude=ta[i], rotation_amplitude=ra[i], scale_amplitude=0,
                             translation_period=10+.8*i, rotation_period=8.5+.7*i,
                             scale_period=9, phase=phases[i], scale_enabled=False)
        windows.append(PeriodicConvexWindow(name=f"W{i+1}_{baseline.SHAPE_NAMES[i]}", aperture=aperture,
                       center0=baseline.CENTERS[i], angles0_rpy=baseline.ANGLES_RPY[i], motion=motion))
    track=SCWindowTrack(name="seven_convex_periodic_3d_closed_x500", start=baseline.START,
                        goal=baseline.GOAL, windows=tuple(windows), order=tuple(range(7)))
    config=OptimizationConfig(initial_speed=5.0, minimum_initial_duration=.2, max_iterations=0,
        max_line_search_steps=64, memory_size=256, past_iterations=32, function_tolerance=1e-5,
        gradient_tolerance=0, samples_per_segment=None, include_window_time_gradient=True,
        objective_weights=ObjectiveWeights(time=1, snap_energy=0),
        penalty_weights=PenaltyWeights(velocity=0, collective_thrust=0, body_rate=1000, rotor_thrust=1000),
        dynamic_limits=DynamicLimits(max_velocity=60.0, min_collective_thrust=0,
          max_collective_thrust=4*X500_MAX_ROTOR_THRUST, max_body_rate_xy=X500_MAX_RATE_XY,
          max_body_rate_z=X500_MAX_RATE_Z, min_rotor_thrust=0, max_rotor_thrust=X500_MAX_ROTOR_THRUST),
        quadrotor=QuadrotorParameters(mass=X500_MASS, inertia=X500_INERTIA, arm_length=X500_ROTOR_ARM,
          yaw_moment_coefficient=X500_MOMENT_CONSTANT, mixing_matrix=np.array([
            [1,1,1,1],[-.174,.174,.174,-.174],[-.174,.174,-.174,.174],[-.016,-.016,.016,.016]])))
    return track, config


class X500Objective(NativeJointTOGTObjective):
    def __init__(self, track, config):
        super().__init__(track, config)
        self.core = AnalyticTOGTCore()


def dynamic_audit(trajectory, config):
    nodes=max(int(np.ceil(float(t)/AUDIT_STEP))+1 for t in trajectory.durations)
    s=trajectory.sample(samples_per_segment=nodes)
    pvajs=np.stack((s.position,s.velocity,s.acceleration,s.jerk,s.snap),axis=1)
    d=AnalyticTOGTCore().sample_dynamics(pvajs); regular=d["regular_branch"]
    rate=d["body_rate"]; rotor=d["rotor_thrusts"]
    ext={"max_velocity":float(d["speed"].max()), "max_tilt":float(d["tilt"].max()),
         "max_body_rate_xy":float(np.linalg.norm(rate[:,:2],axis=1).max()),
         "max_abs_body_rate_z":float(np.abs(rate[regular,2]).max()),
         "min_rotor_thrust":float(rotor[regular].min()), "max_rotor_thrust":float(rotor[regular].max()),
         "max_instantaneous_penalty":float(d["instantaneous_penalty"].max())}
    tests={"tilt":ext["max_tilt"]<=X500_MAX_TILT+1e-9,
           "body_rate_xy":ext["max_body_rate_xy"]<=X500_MAX_RATE_XY+1e-9,
           "body_rate_z":ext["max_abs_body_rate_z"]<=X500_MAX_RATE_Z+1e-9,
           "rotor_thrust":ext["min_rotor_thrust"]>=-1e-9 and ext["max_rotor_thrust"]<=X500_MAX_ROTOR_THRUST+1e-9}
    return {"passed":bool(all(tests.values())),"per_constraint":tests,"extrema":ext,
            "samples_per_segment":nodes,"maximum_step_bound_seconds":AUDIT_STEP,
            "velocity":{"used_for_dynamic_acceptance":False,"peak_mps":ext["max_velocity"],
                        "px4_position_controller_limit_mps":PX4_CONTROLLER_MAX_VELOCITY},
            "evidence":"full-flight dense sampling with the x500 C++ QuadManifold; not a continuous certificate"}


def safety_audit(trajectory, track):
    count=int(np.ceil(trajectory.total_time/AUDIT_STEP))+1
    times=np.linspace(0,trajectory.total_time,count); positions=np.real(trajectory.evaluate(times,0))
    rows=[]
    for i,w in enumerate(track.windows):
        clearance=[]
        for t,p in zip(times,positions):
            local,plane=w.world_to_local(p,float(t))
            clearance.append(np.hypot(float(w.aperture.boundary_distance(local)),plane)-X500_FRAME_RADIUS)
        j=int(np.argmin(clearance)); rows.append({"window_index":i,"window_name":w.name,
            "passed":bool(clearance[j]>=-1e-9),"minimum_margin":float(clearance[j]),"minimum_margin_time":float(times[j])})
    crossing_times=np.cumsum(trajectory.durations)[:-1]; crossings=[]
    for ci,(wi,t) in enumerate(zip(track.order,crossing_times)):
        local,plane=track.windows[wi].world_to_local(trajectory.evaluate(float(t),0),float(t))
        crossings.append({"crossing_index":ci,"window_index":wi,"time":float(t),
                          "plane_error":abs(float(plane)),"inside_aperture":bool(track.windows[wi].aperture.contains(local))})
    cp=all(x["plane_error"]<=1e-7 and x["inside_aperture"] for x in crossings)
    return {"passed":bool(all(r["passed"] for r in rows) and cp),"sphere_frame_clearance_passed":bool(all(r["passed"] for r in rows)),
            "prescribed_crossings_passed":cp,"per_window":rows,"crossings":crossings,"sample_count":count,
            "maximum_step_bound_seconds":AUDIT_STEP,"acceptance_margin_added":0.0,
            "vehicle_sphere_radius":X500_FRAME_RADIUS,
            "safety_model":"direction-independent circumscribed x500 footprint sphere",
            "role":"conservative planning proxy only; final collision acceptance uses the official x500 SDF and Gazebo sleeve meshes",
            "evidence":"full-flight dense sphere-proxy distance sampling; not a continuous certificate or final Gazebo collision verdict"}


def jsonable(v):
    if isinstance(v,np.ndarray): return v.tolist()
    if isinstance(v,np.generic): return v.item()
    if isinstance(v,dict): return {str(k):jsonable(x) for k,x in v.items()}
    if isinstance(v,(list,tuple)): return [jsonable(x) for x in v]
    return v


def main(argv=None):
    p=argparse.ArgumentParser(); p.add_argument("--outdir",type=Path,required=True)
    p.add_argument("--initial-result",type=Path); a=p.parse_args(argv)
    out=a.outdir.resolve(); out.mkdir(parents=True,exist_ok=False); (out/"figures").mkdir(); total=time.perf_counter()
    stage=time.perf_counter(); track,config=build_track(); objective=X500Objective(track,config); x0=objective.initial_guess()
    if a.initial_result:
        x0=np.asarray(json.loads(a.initial_result.read_text())["decision_vector"],dtype=float)
    initial_cost,_=objective.value_and_gradient(x0); prep=time.perf_counter()-stage
    evals=0; last=time.perf_counter()
    def measured(x):
        nonlocal evals,last
        evals+=1; ans=objective.scipy_value_and_gradient(x); now=time.perf_counter()
        if now-last>20: print(f"L-BFGS evaluations={evals} cost={ans[0]:.9g}",flush=True); last=now
        return ans
    print(f"x500 margin={WINDOW_MARGIN:.9f}m preprocessing={prep:.6f}s initial_cost={initial_cost:.9g}",flush=True)
    stage=time.perf_counter(); opt=_minimize_togt_lbfgs(measured,x0,config); lbfgs=time.perf_counter()-stage
    cost,grad=objective.value_and_gradient(opt.x); final=objective.forward(opt.x)
    print(f"L-BFGS done in {lbfgs:.6f}s T={final.trajectory.total_time:.9f}s",flush=True)
    stage=time.perf_counter(); dynamics=dynamic_audit(final.trajectory,config); dt=time.perf_counter()-stage
    print(f"dynamic audit: {'PASS' if dynamics['passed'] else 'FAIL'}",flush=True)
    stage=time.perf_counter(); safety=safety_audit(final.trajectory,track); st=time.perf_counter()-stage
    print(f"safety audit: {'PASS' if safety['passed'] else 'FAIL'}",flush=True)
    result={"flight_time_seconds":final.trajectory.total_time,"objective":cost,
      "dynamic_soft_integral":cost-final.trajectory.total_time,
      "optimizer":{"success":bool(opt.success),"status":int(opt.status),"message":str(opt.message),"iterations":int(opt.nit),
                   "evaluations":int(opt.nfev),"measured_evaluations":evals,"gradient_inf_norm":float(np.linalg.norm(grad,np.inf)),
                   "invalid_trial_count":objective.invalid_trial_count},
      "durations":final.durations,"traversal_times":final.traversal_times,"waypoints":final.waypoints,
      "local_points":final.local_points,"decision_vector":np.asarray(opt.x),"dynamic_audit":dynamics,"safety_audit":safety,
      "timing":{"preprocessing_seconds":prep,"lbfgs_seconds":lbfgs,"dynamic_audit_seconds":dt,"safety_audit_seconds":st,
                "total_wall_seconds":time.perf_counter()-total},
      "x500_model":{"source":"PX4 Gazebo x500/model.sdf and x500_base/model.sdf","mass":X500_MASS,"inertia_diagonal":X500_INERTIA,
        "rotor_arm_xy":X500_ROTOR_ARM,"moment_constant":X500_MOMENT_CONSTANT,"max_rotor_thrust":X500_MAX_ROTOR_THRUST,
        "physical_max_velocity":None,"px4_position_controller_max_velocity":PX4_CONTROLLER_MAX_VELOCITY,
        "max_rate_xy":X500_MAX_RATE_XY,"max_rate_z":X500_MAX_RATE_Z,"max_tilt":X500_MAX_TILT,
        "frame_radius":X500_FRAME_RADIUS,"window_margin_factor":MARGIN_FACTOR,"window_margin":WINDOW_MARGIN},
      "protocol":{"reoptimized_D_and_K":True,"acceptance_uses_extra_margin":False,
        "planner_constraint_buffer":{"max_tilt":0.78},"velocity_constraint_enabled":False,
        "initial_result":str(a.initial_result.resolve()) if a.initial_result else None,
        "optimization_config":asdict(config)}}
    (out/"result.json").write_text(json.dumps(jsonable(result),ensure_ascii=False,indent=2)+"\n")
    baseline.plot_route(out/"figures"/"route_overview.png",final.trajectory,track,final.traversal_times)
    print(f"result={out/'result.json'}",flush=True); return 0 if opt.success else 2

if __name__ == "__main__": raise SystemExit(main())
