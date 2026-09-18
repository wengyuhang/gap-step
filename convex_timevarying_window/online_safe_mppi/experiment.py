#!/usr/bin/env python3
"""Closed-loop benchmark for viability-triggered gate-point MPPI.

The experiment deliberately uses only a short local rollout horizon.  The
complete lap is audited afterwards, but that audit is not part of the online
controller timing or decision loop.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass, asdict
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import platform
import sys
import time

import numpy as np
import torch

from convex_timevarying_window.conditional_dual_constraint_cem.safety_penalty import (
    polygon_halfspaces,
)
from convex_timevarying_window.togt.experiment import build_track
from convex_timevarying_window.togt.native_backend import AnalyticTOGTCore


BODY_RADIUS = 0.37942273679894306
G = 9.8066


def unit(x, eps=1.0e-9):
    x = np.asarray(x, dtype=float)
    return x / max(float(np.linalg.norm(x)), eps)


def closest_on_segment(point, a, b):
    edge = b - a
    alpha = float(np.dot(point - a, edge) / max(np.dot(edge, edge), 1.0e-12))
    return a + np.clip(alpha, 0.0, 1.0) * edge


@dataclass
class GatePlan:
    gate_index: int
    made_at: float
    cross_time: float
    approach_point: np.ndarray
    point: np.ndarray
    exit_point: np.ndarray
    point_local: np.ndarray
    dynamic_target: bool
    incoming_sign: float
    cause: str
    solve_ms: float
    commitment_slack_s: float


@dataclass
class Config:
    controller_dt: float = 0.05
    rollout_steps: int = 24
    rollouts: int = 1024
    temperature: float = 12.0
    noise_sigma: float = 0.8
    max_snap: float = 100.0
    max_jerk: float = 15.0
    max_acceleration: float = 9.0
    max_speed: float = 8.0
    cruise_speed: float = 6.5
    acceleration_response_time: float = 0.20
    rollout_projection_iterations: int = 4
    safety_margin: float = 0.055
    swept_margin: float = 0.12
    max_plan_age: float = 0.80
    min_replan_dwell: float = 0.40
    point_shift_trigger: float = 0.25
    time_shift_trigger: float = 0.20
    terminal_time_reserve: float = 0.22
    approach_distance: float = 8.0
    departure_distance: float = 2.5
    gate_point_trust_radius: float = 0.10
    max_flight_time: float = 90.0


class Course:
    def __init__(self, device: torch.device):
        self.track, _ = build_track()
        self.windows = self.track.windows
        self.device = device
        self.start = np.asarray(self.track.start, dtype=float)
        self.goal = np.asarray(self.track.goal, dtype=float)
        self._geometries = []
        for window in self.windows:
            if window.aperture.kind == "circle":
                self._geometries.append(("circle", float(window.aperture.radius)))
            else:
                vertices = np.asarray(window.aperture.physical_vertices, dtype=float)
                halfspaces = polygon_halfspaces(vertices)
                self._geometries.append(("polygon", vertices, halfspaces))

    def pose(self, index: int, absolute_time: float):
        center, rotation, center_rate, rotation_rate = self.windows[index].state_at(absolute_time)
        return center, rotation, center_rate, rotation_rate

    def safe_local_polygon(self, index: int, clearance: float):
        kind, *geometry = self._geometries[index]
        if kind == "circle":
            return kind, float(geometry[0]) - clearance
        vertices, halfspaces = geometry
        # Every polygon in this frozen course is centred and tangential.  A
        # radial scaling therefore gives the exact Euclidean inset.
        apothem = float(np.min(halfspaces.offsets))
        scale = max(0.02, (apothem - clearance) / apothem)
        return kind, vertices * scale

    def project_safe(self, index: int, point_local: np.ndarray, clearance: float):
        kind, safe = self.safe_local_polygon(index, clearance)
        q = np.asarray(point_local, dtype=float)
        if kind == "circle":
            norm = float(np.linalg.norm(q))
            return q if norm <= safe else q * safe / max(norm, 1.0e-12)
        hs = polygon_halfspaces(safe)
        if np.all(hs.normals @ q <= hs.offsets + 1.0e-10):
            return q
        candidates = [closest_on_segment(q, a, b) for a, b in zip(safe, np.roll(safe, -1, axis=0))]
        return min(candidates, key=lambda value: float(np.dot(value - q, value - q)))

    def frame_clearance(self, positions: np.ndarray, times: np.ndarray):
        """Independent dense sphere-to-frame audit of an executed trajectory."""
        minimum = math.inf
        collision_count = 0
        per_gate = []
        for index, window in enumerate(self.windows):
            gate_minimum = math.inf
            gate_collisions = 0
            for p, t in zip(positions, times):
                q, z = window.world_to_local(p, float(t))
                boundary = float(window.aperture.boundary_distance(q))
                clearance = math.sqrt(z * z + boundary * boundary) - BODY_RADIUS
                gate_minimum = min(gate_minimum, clearance)
                gate_collisions += int(clearance < 0.0)
            minimum = min(minimum, gate_minimum)
            collision_count += gate_collisions
            per_gate.append({"gate": index + 1, "minimum_clearance_m": gate_minimum,
                             "collision_samples": gate_collisions})
        return minimum, collision_count, per_gate

    def point_clearance(self, index: int, position: np.ndarray, absolute_time: float):
        q,z=self.windows[index].world_to_local(position,absolute_time)
        boundary=float(self.windows[index].aperture.boundary_distance(q))
        return math.sqrt(z*z+boundary*boundary)-BODY_RADIUS


def corridor_path_length(position, point, normal, incoming_sign, outgoing,
                         approach_distance, departure_distance,
                         current_signed_distance):
    """Length of the approach/crossing/departure polyline actually tracked."""
    approach = point + approach_distance * incoming_sign * normal
    departure = point - departure_distance * incoming_sign * normal
    if current_signed_distance > approach_distance:
        inbound = np.linalg.norm(position-approach) + approach_distance
    else:
        # Once inside the alignment corridor the controller tracks the opening
        # directly; it does not turn around to revisit the corridor entrance.
        inbound = np.linalg.norm(position-point)
    return float(inbound + departure_distance + np.linalg.norm(outgoing-departure))


class GatePointPlanner:
    """Fast dynamic traversal-point/time update inspired by TOGT's gate map."""

    def __init__(self, course: Course, config: Config, center_only: bool = False):
        self.course = course
        self.config = config
        self.center_only = center_only
        self.times_ms = []

    def candidate(self, state: np.ndarray, now: float, gate_index: int,
                  force_center: bool = False, locked_local: np.ndarray | None = None):
        p, v = state[:3], state[3:6]
        center_now, _, _, _ = self.course.pose(gate_index, now)
        eta = max(0.35, np.linalg.norm(center_now - p) /
                  max(self.config.cruise_speed, np.linalg.norm(v), 2.0))
        best = None
        for offset in np.linspace(-0.45, 0.45, 7):
            cross_time = now + max(0.25, eta + float(offset))
            center, rotation, _, _ = self.course.pose(gate_index, cross_time)
            normal = rotation[:, 2]
            incoming_source = self.course.start if gate_index == 0 else self.course.pose(gate_index - 1, now)[0]
            incoming_sign = float(np.sign(np.dot(incoming_source - center, normal)) or 1.0)
            if gate_index + 1 < len(self.course.windows):
                next_time=cross_time+2.5
                next_center,next_rotation,_,_=self.course.pose(gate_index+1,next_time)
                next_normal=next_rotation[:,2]
                next_sign=float(np.sign(np.dot(center-next_center,next_normal)) or 1.0)
                outgoing=(next_center+self.config.approach_distance
                          *next_sign*next_normal)
            else:
                outgoing = self.course.goal
            center_current,rotation_current,_,_=self.course.pose(gate_index,now)
            current_signed=float(incoming_sign*rotation_current[:,2]@(p-center_current))
            if self.center_only or force_center or gate_index == 4:
                local_candidates=[np.zeros(2)]
            elif locked_local is not None:
                local_candidates=[np.asarray(locked_local,dtype=float)]
            else:
                ray = outgoing - p
                denominator = float(np.dot(normal, ray))
                alpha = float(np.dot(normal, center - p) / denominator) if abs(denominator) > 1.0e-8 else 0.5
                intersection = p + np.clip(alpha, 0.0, 1.0) * ray
                local_ray = (rotation.T @ (intersection - center))[:2]
                local_ray = self.course.project_safe(
                    gate_index, local_ray, BODY_RADIUS + self.config.safety_margin
                    + self.config.swept_margin + 0.025 + (0.18 if gate_index in (4, 5, 6) else 0.0)
                )
                local_ray=unit(local_ray)*min(
                    np.linalg.norm(local_ray),self.config.gate_point_trust_radius)
                local_candidates=[scale*local_ray for scale in (0.0,0.25,0.5,0.75,1.0)]
            for local in local_candidates:
                point = center + rotation[:, :2] @ local
                path_length = corridor_path_length(
                    p,point,normal,incoming_sign,outgoing,
                    self.config.approach_distance,self.config.departure_distance,
                    current_signed,
                )
                timing_mismatch = abs((cross_time - now) - path_length /
                                      max(self.config.cruise_speed, 1.0))
                # Offset can shorten the geometric path but costs robustness
                # and lateral control effort.  Center remains an explicit
                # candidate, so free-point planning cannot win by definition.
                offset_penalty=0.80*float(np.dot(local,local))
                score = path_length + 1.5 * timing_mismatch + offset_penalty
                # First leave the aperture along its normal; turning toward the
                # next gate while the body still overlaps the frame is unsafe.
                exit_point = point - self.config.departure_distance * incoming_sign * normal
                approach_point = point + self.config.approach_distance * incoming_sign * normal
                item = (score, cross_time, approach_point, point, exit_point, incoming_sign,
                        np.asarray(local, dtype=float))
                if best is None or item[0] < best[0]:
                    best = item
        assert best is not None
        return best[1:]

    def plan(self, state, now, gate_index, cause, commitment_slack,
             force_center=False, locked_local=None):
        started = time.perf_counter()
        cross_time, approach_point, point, exit_point, incoming_sign, point_local = self.candidate(
            state, now, gate_index, force_center=force_center,locked_local=locked_local)
        elapsed = (time.perf_counter() - started) * 1000.0
        self.times_ms.append(elapsed)
        return GatePlan(gate_index, now, cross_time, approach_point, point, exit_point,
                        point_local, True, incoming_sign, cause, elapsed, commitment_slack)


def torch_norm_clip(values, maximum):
    scale = torch.clamp(maximum / torch.linalg.vector_norm(values, dim=-1, keepdim=True).clamp_min(1.0e-8), max=1.0)
    return values * scale


def torch_dynamics_step(acceleration, jerk, command, cfg: Config):
    """Vectorized counterpart of :func:`integrate_step` for MPPI rollouts."""
    dt=cfg.controller_dt
    response=acceleration+(command-acceleration)*(dt/cfg.acceleration_response_time)
    response=torch_norm_clip(response,cfg.max_acceleration)
    jerk_target=torch_norm_clip((response-acceleration)/dt,cfg.max_jerk)
    desired_snap=(jerk_target-jerk)/dt
    centers=(
        torch.zeros_like(desired_snap),
        -jerk/dt,
        -(acceleration+jerk*dt)/(dt*dt),
    )
    guard=1.0-5.0e-3
    radii=(cfg.max_snap*guard,cfg.max_jerk*guard/dt,
           cfg.max_acceleration*guard/(dt*dt))
    snap=desired_snap
    corrections=[torch.zeros_like(snap) for _ in centers]
    for _ in range(cfg.rollout_projection_iterations):
        for index,(center,radius) in enumerate(zip(centers,radii)):
            value=snap+corrections[index]
            delta=value-center
            projected=center+torch_norm_clip(delta,radius)
            corrections[index]=value-projected
            snap=projected
    snap=torch_norm_clip(snap,cfg.max_snap*(1.0-1.0e-5))
    next_jerk=jerk+snap*dt
    next_acceleration=acceleration+next_jerk*dt
    return next_acceleration,next_jerk,snap


def locally_relevant_gates(course: Course, cfg: Config, state: np.ndarray,
                           now: float):
    """Cull frames that cannot intersect the finite-horizon reachable ball.

    ``bounds[i]`` is a conservative lower bound on separation between every
    reachable body centre and gate ``i``'s frame during the MPPI horizon.
    Positive bounds therefore make omission from rollout collision queries
    safe under the benchmark's bounded-acceleration model.
    """
    horizon=cfg.rollout_steps*cfg.controller_dt
    reachable=(np.linalg.norm(state[3:6])*horizon
               +0.5*cfg.max_acceleration*horizon*horizon
               +BODY_RADIUS+cfg.safety_margin+cfg.swept_margin)
    relevant=[]; bounds=[]
    for index,window in enumerate(course.windows):
        center,_,_,_=course.pose(index,now)
        motion=window.motion
        center_speed_bound=(np.linalg.norm(np.asarray(motion.translation_amplitude,float))
                            *2.0*math.pi/motion.translation_period)
        geometry=course._geometries[index]
        frame_radius=(float(geometry[1]) if geometry[0]=="circle" else
                      float(np.linalg.norm(geometry[1],axis=1).max()))
        lower=(np.linalg.norm(state[:3]-center)-center_speed_bound*horizon
               -frame_radius-reachable)
        bounds.append(float(lower))
        if lower<=0.0:
            relevant.append(index)
    return relevant,bounds


class LocalSafeMPPI:
    def __init__(self, course: Course, config: Config, seed: int):
        self.course, self.config = course, config
        self.device = course.device
        self.generator = torch.Generator(device=self.device).manual_seed(seed)
        self.controls = torch.zeros((config.rollout_steps, 3), device=self.device)
        self.latencies_ms = []
        self.safe_ratios = []
        self.no_safe_count = 0
        self.relevant_gate_counts = []

    def _guidance(self, state: np.ndarray, now: float, plan: GatePlan):
        """Build a smooth feasible mean sequence before stochastic refinement."""
        cfg=self.config; dt=cfg.controller_dt
        p=torch.tensor(state[:3],device=self.device,dtype=torch.float32)
        v=torch.tensor(state[3:6],device=self.device,dtype=torch.float32)
        a=torch.tensor(state[6:9],device=self.device,dtype=torch.float32)
        j=torch.tensor(state[9:12],device=self.device,dtype=torch.float32)
        fixed_point=torch.tensor(plan.point,device=self.device,dtype=torch.float32)
        fixed_approach=torch.tensor(plan.approach_point,device=self.device,dtype=torch.float32)
        fixed_exit=torch.tensor(plan.exit_point,device=self.device,dtype=torch.float32)
        point_local=torch.tensor(plan.point_local,device=self.device,dtype=torch.float64)
        sequence=[]
        for step in range(cfg.rollout_steps):
            t=torch.tensor([now+(step+1)*dt],device=self.device)
            center,rotation=self._pose_torch(plan.gate_index,t)
            if plan.dynamic_target:
                point=center[0]+rotation[0,:,:2]@point_local
                approach=point+cfg.approach_distance*plan.incoming_sign*rotation[0,:,2]
                exit_point=point-cfg.departure_distance*plan.incoming_sign*rotation[0,:,2]
            else:
                point,approach,exit_point=fixed_point,fixed_approach,fixed_exit
            signed=float((torch.dot(rotation[0,:,2],p-center[0])*plan.incoming_sign).item())
            near_gate=float(torch.linalg.vector_norm(p-point).item()) < 0.8
            target=exit_point if signed<=0.0 and near_gate else (
                approach if signed>cfg.approach_distance else point)
            delta=target-p; distance=torch.linalg.vector_norm(delta).clamp_min(0.1)
            # Reduce speed only in the final stopping ball; gates are crossed
            # at racing speed and use the exit point to prevent braking there.
            gate_speed=3.0 if plan.gate_index==4 else (4.0 if plan.gate_index in (5,6) else 5.2)
            speed_cap=cfg.cruise_speed if signed<=0.0 and near_gate else min(cfg.cruise_speed,gate_speed)
            desired_speed=min(speed_cap,max(1.2,2.2*float(distance)))
            desired_v=desired_speed*delta/distance
            desired_v[2]=desired_v[2].clamp(-3.0,3.0)
            desired_a=torch_norm_clip((desired_v-v)/0.50,cfg.max_acceleration*0.85)
            sequence.append(desired_a)
            a_next,j,_=torch_dynamics_step(a,j,desired_a,cfg)
            p=p+v*dt+0.5*a_next*dt**2
            v=v+a_next*dt
            a=a_next
        return torch.stack(sequence)

    def _pose_torch(self, index: int, times: torch.Tensor):
        w = self.course.windows[index]
        motion = w.motion
        phase_t = torch.tensor([0.0, 0.7, 1.4], device=self.device) + float(motion.phase)
        phase_r = torch.tensor([0.0, 0.9, 1.8], device=self.device) + float(motion.phase)
        ta = torch.tensor(motion.translation_amplitude, device=self.device)
        ra = torch.tensor(motion.rotation_amplitude, device=self.device)
        center = torch.tensor(w.center0, device=self.device)[None, :] + ta[None, :] * torch.sin(
            times[:, None] * (2.0 * math.pi / motion.translation_period) + phase_t[None, :]
        )
        angles = torch.tensor(w.angles0_rpy, device=self.device)[None, :] + ra[None, :] * torch.sin(
            times[:, None] * (2.0 * math.pi / motion.rotation_period) + phase_r[None, :]
        )
        roll, pitch, yaw = angles.unbind(-1)
        cr, sr, cp, sp, cy, sy = map(torch.cos, (roll, roll, pitch, pitch, yaw, yaw))
        # Correct the sine terms after using a compact tuple construction.
        sr, sp, sy = torch.sin(roll), torch.sin(pitch), torch.sin(yaw)
        rotation = torch.stack((
            cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr,
            sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr,
            -sp, cp*sr, cp*cr,
        ), dim=-1).reshape(-1, 3, 3)
        return center, rotation

    def _gate_clearance(self, positions, absolute_times, gate_index):
        center, rotation = self._pose_torch(gate_index, absolute_times)
        # positions [K,H,3], pose [H,3]/[H,3,3]
        local = torch.einsum("hji,khi->khj", rotation, positions - center[None, :, :])
        planar, normal = local[..., :2], local[..., 2]
        geometry = self.course._geometries[gate_index]
        if geometry[0] == "circle":
            boundary = torch.abs(torch.linalg.vector_norm(planar, dim=-1) - float(geometry[1]))
        else:
            vertices = torch.tensor(geometry[1], device=self.device, dtype=positions.dtype)
            starts, edges = vertices, torch.roll(vertices, -1, 0) - vertices
            relative = planar[..., None, :] - starts
            alpha = (relative * edges).sum(-1) / (edges * edges).sum(-1)
            closest = starts + alpha.clamp(0.0, 1.0)[..., None] * edges
            boundary = torch.linalg.vector_norm(planar[..., None, :] - closest, dim=-1).amin(-1)
        return torch.sqrt(normal.square() + boundary.square()) - BODY_RADIUS

    def command(self, state: np.ndarray, now: float, plan: GatePlan):
        cfg = self.config
        if self.device.type == "cuda": torch.cuda.synchronize()
        started = time.perf_counter()
        x0 = torch.tensor(state, device=self.device, dtype=torch.float32)
        k, h, dt = cfg.rollouts, cfg.rollout_steps, cfg.controller_dt
        guidance=self._guidance(state,now,plan)
        self.controls=0.15*self.controls+0.85*guidance
        # Temporally correlated exploration is markedly less jerky than IID noise.
        raw = torch.randn((k, h, 3), device=self.device, generator=self.generator)
        noise = raw.clone()
        for step in range(1, h):
            noise[:, step] = 0.72 * noise[:, step-1] + math.sqrt(1.0-0.72**2) * noise[:, step]
        noise *= cfg.noise_sigma
        noise[0].zero_()
        controls = torch_norm_clip(self.controls[None, :, :] + noise, cfg.max_acceleration)
        p = x0[:3].expand(k, 3).clone(); v = x0[3:6].expand(k, 3).clone()
        a = x0[6:9].expand(k, 3).clone()
        j = x0[9:12].expand(k, 3).clone()
        positions=[]; velocities=[]; accelerations=[]; jerks=[]; snaps=[]
        for step in range(h):
            command = controls[:, step]
            a_next,j,snap = torch_dynamics_step(a,j,command,cfg)
            p = p + v*dt + 0.5*a_next*dt**2
            v = v + a_next*dt
            a = a_next
            v = torch_norm_clip(v, cfg.max_speed)
            positions.append(p); velocities.append(v); accelerations.append(a); jerks.append(j); snaps.append(snap)
        positions=torch.stack(positions,1); velocities=torch.stack(velocities,1)
        accelerations=torch.stack(accelerations,1); jerks=torch.stack(jerks,1)
        snaps=torch.stack(snaps,1)
        times = now + dt * torch.arange(1, h+1, device=self.device)
        center, rotation = self._pose_torch(plan.gate_index, times)
        if plan.dynamic_target:
            point_local=torch.tensor(plan.point_local,device=self.device,dtype=center.dtype)
            point=center+torch.einsum("hij,j->hi",rotation[:,:,:2],point_local)
            approach=point+cfg.approach_distance*plan.incoming_sign*rotation[:,:,2]
            exit_point=point-cfg.departure_distance*plan.incoming_sign*rotation[:,:,2]
        else:
            point=torch.tensor(plan.point,device=self.device,dtype=torch.float32).expand(h,3)
            approach=torch.tensor(plan.approach_point,device=self.device,dtype=torch.float32).expand(h,3)
            exit_point=torch.tensor(plan.exit_point,device=self.device,dtype=torch.float32).expand(h,3)
        signed = torch.einsum("hi,khi->kh", rotation[:,:,2], positions-center[None,:,:]) * plan.incoming_sign
        passed = (signed <= 0.0) & (torch.linalg.vector_norm(positions-point[None,:,:],dim=-1) < 0.8)
        point_offset=positions-point[None,:,:]
        normals=rotation[:,:,2].to(positions.dtype)
        normal_offset=torch.einsum("hi,khi->kh",normals,point_offset)
        lateral_offset=point_offset-normal_offset[...,None]*normals[None,:,:]
        lateral_error=torch.linalg.vector_norm(lateral_offset,dim=-1)
        illegal_bypass=(signed < 1.5) & (lateral_error >= 0.65)
        target = torch.where(passed[...,None], exit_point[None,:,:],
                             torch.where((signed>cfg.approach_distance)[...,None],
                                         approach[None,:,:],point[None,:,:]))
        direction = target-positions
        gate_speed=3.0 if plan.gate_index==4 else (4.0 if plan.gate_index in (5,6) else 5.2)
        speed=torch.where(passed,torch.full_like(signed,cfg.cruise_speed),
                          torch.full_like(signed,min(cfg.cruise_speed,gate_speed)))
        desired_v = speed[...,None] * direction / torch.linalg.vector_norm(direction,dim=-1,keepdim=True).clamp_min(0.25)
        desired_v[:,:,2]=desired_v[:,:,2].clamp(-3.0,3.0)
        velocity_error=(velocities-desired_v).square().sum(-1)
        distance=torch.linalg.vector_norm(direction,dim=-1)
        cost = 0.020*distance.sum(-1) + 0.012*velocity_error.sum(-1)
        desired_signed=torch.clamp(
            gate_speed*(plan.cross_time-times),-cfg.departure_distance,cfg.approach_distance)
        phase_active=(torch.abs(plan.cross_time-times)<=cfg.rollout_steps*dt).to(signed.dtype)
        # The optimized time is a soft phase preference, not a command to
        # wait for a stale deadline.  Progress and viability remain dominant.
        cost += 0.001*((signed-desired_signed[None,:]).square()
                       *phase_active[None,:]).sum(-1)
        cost += 0.006*controls.square().sum((1,2)) + 0.001*jerks.square().sum((1,2))
        cost += 0.010*(controls[:,1:]-controls[:,:-1]).square().sum((1,2))
        cost += 5.0*distance[:,-1] + 0.12*velocity_error[:,-1]
        cost -= 22.0*passed.any(-1).float()
        invalid = (positions[:,:,2] < 0.45).any(-1) | (positions[:,:,2] > 9.5).any(-1)
        invalid |= illegal_bypass.any(-1)
        invalid |= (torch.linalg.vector_norm(accelerations,dim=-1)>cfg.max_acceleration+1.0e-5).any(-1)
        invalid |= (torch.linalg.vector_norm(jerks,dim=-1)>cfg.max_jerk+1.0e-5).any(-1)
        invalid |= (torch.linalg.vector_norm(snaps,dim=-1)>cfg.max_snap+1.0e-5).any(-1)
        thrust = accelerations + torch.tensor([0.0,0.0,G],device=self.device)
        tilt_cos = thrust[:,:,2] / torch.linalg.vector_norm(thrust,dim=-1).clamp_min(1.0e-6)
        invalid |= (tilt_cos < math.cos(math.radians(48.0))).any(-1)
        # The online safety query covers only the short rollout tube.  A swept
        # inflation closes the gap between 40 ms nodes without a lap-wide audit.
        minimum_frame_clearance=torch.full((k,),float("inf"),device=self.device)
        relevant_gates,_=locally_relevant_gates(self.course,cfg,state,now)
        self.relevant_gate_counts.append(len(relevant_gates))
        for gate_index in relevant_gates:
            clearance = self._gate_clearance(positions, times, gate_index)
            minimum_frame_clearance=torch.minimum(minimum_frame_clearance,clearance.amin(-1))
            invalid |= (clearance < cfg.safety_margin + cfg.swept_margin).any(-1)
        safe = ~invalid
        safe_ratio = float(safe.float().mean().item())
        self.safe_ratios.append(safe_ratio)
        if not bool(safe.any()):
            self.no_safe_count += 1
            # Fall back to the analytically guided sequence around which the
            # sampler was built.  Using a terminal rollout state here would
            # feed a sampled failure back into the real vehicle.
            # Lexicographic survival shield: when the complete short horizon
            # has no feasible sample, execute the first action of the rollout
            # with the largest worst-case frame clearance.  It never silently
            # relabels that rollout safe; the no-safe event remains recorded.
            survival_score=minimum_frame_clearance-5.0*illegal_bypass.any(-1).float()
            command=controls[int(torch.argmax(survival_score).item()),0]
        else:
            cost = torch.where(safe, cost, torch.full_like(cost, 1.0e9))
            finite = cost[safe]
            weights = torch.exp(-(cost-finite.min())/cfg.temperature) * safe.float()
            weights /= weights.sum().clamp_min(1.0e-12)
            update = (weights[:,None,None]*noise).sum(0)
            self.controls = torch_norm_clip(self.controls + 0.15*update,cfg.max_acceleration)
            command = self.controls[0].clone()
        self.controls[:-1] = self.controls[1:].clone()
        self.controls[-1].zero_()
        if self.device.type == "cuda": torch.cuda.synchronize()
        elapsed=(time.perf_counter()-started)*1000.0
        self.latencies_ms.append(elapsed)
        return command.detach().cpu().numpy(), safe_ratio


def passage(course: Course, index: int, p0, p1, t0, t1, incoming_sign):
    def signed(alpha):
        absolute_time=t0+alpha*(t1-t0)
        position=p0+alpha*(p1-p0)
        center,rotation,_,_=course.pose(index,absolute_time)
        return float(np.dot(rotation[:,2],position-center))*incoming_sign
    z0=signed(0.0); z1=signed(1.0)
    if z0 <= 0.0 or z1 > 0.0: return False, None, None
    lo,hi=0.0,1.0
    for _ in range(120):
        mid=0.5*(lo+hi)
        if signed(mid)>0.0: lo=mid
        else: hi=mid
    alpha=0.5*(lo+hi); cross=p0+alpha*(p1-p0); tc=t0+alpha*(t1-t0)
    q,z=course.windows[index].world_to_local(cross,tc)
    clearance=float(course.windows[index].aperture.boundary_distance(q))-BODY_RADIUS
    inside=course.windows[index].aperture.contains(q)
    return bool(inside and clearance >= 0.0), tc, clearance


def commitment_slack(course: Course, cfg: Config, state, now, plan):
    p,v=state[:3],state[3:6]
    center,rotation,_,_=course.pose(plan.gate_index,now)
    distance=abs(float(np.dot(rotation[:,2],p-center)))
    closing=max(1.0,abs(float(np.dot(rotation[:,2],v))))
    time_to_plane=distance/closing
    lateral_error=np.linalg.norm((rotation.T@(plan_point_at(course,plan,now)-p))[:2])
    maneuver=math.sqrt(2.0*lateral_error/max(cfg.max_acceleration,1.0))
    return time_to_plane-maneuver-cfg.terminal_time_reserve


def plan_point_at(course: Course, plan: GatePlan, absolute_time: float):
    if not plan.dynamic_target:
        return plan.point
    center, rotation, _, _ = course.pose(plan.gate_index, absolute_time)
    return center + rotation[:, :2] @ plan.point_local


def integrate_step(state: np.ndarray, command: np.ndarray, cfg: Config,
                   disturbance: np.ndarray | None = None):
    """Execute one controller step and return the snap actually applied."""
    dt=cfg.controller_dt
    disturbance=np.zeros(3) if disturbance is None else np.asarray(disturbance,float)
    p,v,a=state[:3],state[3:6],state[6:9]
    jerk_previous=state[9:12]
    acceleration_response=(
        a+(np.asarray(command,float)-a)*(dt/cfg.acceleration_response_time)
        + disturbance
    )
    acceleration_response=unit(acceleration_response)*min(
        np.linalg.norm(acceleration_response),cfg.max_acceleration)
    jerk_target=(acceleration_response-a)/dt
    jerk_target=unit(jerk_target)*min(np.linalg.norm(jerk_target),cfg.max_jerk)
    desired_snap=(jerk_target-jerk_previous)/dt
    # Project the snap onto the intersection of three balls: snap, next
    # jerk, and next acceleration.  Dykstra's method avoids the old bug in
    # which clipping acceleration after snap silently violated snap again.
    centers=(
        np.zeros(3),
        -jerk_previous/dt,
        -(a+jerk_previous*dt)/(dt*dt),
    )
    projection_guard=1.0-1.0e-6
    radii=(cfg.max_snap*projection_guard,
           cfg.max_jerk*projection_guard/dt,
           cfg.max_acceleration*projection_guard/(dt*dt))
    snap=desired_snap.copy(); corrections=[np.zeros(3) for _ in centers]
    for _ in range(40):
        for index,(center,radius) in enumerate(zip(centers,radii)):
            value=snap+corrections[index]
            delta=value-center
            projected=center+unit(delta)*min(np.linalg.norm(delta),radius)
            corrections[index]=value-projected
            snap=projected
    jerk=jerk_previous+snap*dt
    acceleration=a+jerk*dt
    position=p+v*dt+0.5*acceleration*dt**2
    velocity=v+acceleration*dt
    velocity=unit(velocity)*min(np.linalg.norm(velocity),cfg.max_speed)
    return np.r_[position,velocity,acceleration,jerk],snap


def command_is_one_step_safe(course: Course, cfg: Config, state: np.ndarray,
                             now: float, command: np.ndarray, plan: GatePlan,
                             departure_mode: bool, disturbance=None):
    """Revalidate the final composed command over the next 40 ms.

    This is a discrete one-step check, not a continuous invariant-set proof.
    It exists to ensure that sequential filters never execute a command that
    they have not checked after composition.
    """
    next_state,applied_snap=integrate_step(state,command,cfg,disturbance)
    p0,p1=state[:3],next_state[:3]
    a1=next_state[6:9]
    applied_jerk=next_state[9:12]
    if (np.linalg.norm(a1)>cfg.max_acceleration+1.0e-6
            or np.linalg.norm(applied_jerk)>cfg.max_jerk+1.0e-6
            or np.linalg.norm(applied_snap)>cfg.max_snap+1.0e-6):
        return False
    thrust=a1+np.array([0.0,0.0,G])
    if (p1[2]<0.45 or p1[2]>9.5
            or thrust[2]/max(np.linalg.norm(thrust),1.0e-9)<math.cos(math.radians(48.0))):
        return False
    for alpha in (0.25,0.50,0.75,1.0):
        position=p0+alpha*(p1-p0)
        absolute_time=now+alpha*cfg.controller_dt
        if any(course.point_clearance(i,position,absolute_time)<0.0
               for i in range(len(course.windows))):
            return False
    if not departure_mode:
        crossed,_,_=passage(course,plan.gate_index,p0,p1,now,
                             now+cfg.controller_dt,plan.incoming_sign)
        center,rotation,_,_=course.pose(plan.gate_index,now+cfg.controller_dt)
        signed=plan.incoming_sign*float(rotation[:,2]@(p1-center))
        point=plan_point_at(course,plan,now+cfg.controller_dt)
        offset=p1-point
        lateral=offset-rotation[:,2]*float(rotation[:,2]@offset)
        if signed<0.0 and not crossed and np.linalg.norm(lateral)>=0.65:
            return False
    return True


def select_verified_command(course: Course, cfg: Config, state: np.ndarray,
                            now: float, proposed: np.ndarray, plan: GatePlan,
                            departure_mode: bool):
    """Return a revalidated command, or ``None`` instead of flying blind."""
    proposed=torch_norm_clip(
        torch.as_tensor(proposed,dtype=torch.float64),cfg.max_acceleration
    ).numpy()
    if command_is_one_step_safe(course,cfg,state,now,proposed,plan,departure_mode):
        return proposed,False
    candidates=[np.zeros(3),-4.0*state[3:6]]
    for x in (-1.0,0.0,1.0):
        for y in (-1.0,0.0,1.0):
            for z in (-1.0,0.0,1.0):
                direction=np.array([x,y,z])
                if np.linalg.norm(direction)>0.0:
                    candidates.append(cfg.max_acceleration*unit(direction))
    candidates=[unit(c)*min(np.linalg.norm(c),cfg.max_acceleration) for c in candidates]
    candidates.sort(key=lambda c:float(np.dot(c-proposed,c-proposed)))
    for candidate in candidates:
        if command_is_one_step_safe(course,cfg,state,now,candidate,plan,departure_mode):
            return candidate,True
    return None,True


def local_barrier_filter(course: Course, cfg: Config, state, now, command):
    """Minimum-change short-lookahead barrier around moving frames.

    The known gate motion is queried at seven points over 1.2 s.  This catches
    a frame that is about to sweep back into the vehicle, while remaining a
    bounded local calculation rather than a whole-lap collision check.
    """
    started=time.perf_counter(); p=state[:3]; v=state[3:6]; a=state[6:9]
    clearances=np.asarray([course.point_clearance(i,p,now) for i in range(len(course.windows))])
    current_index=int(np.argmin(clearances)); clearance=float(clearances[current_index])
    predicted=[]
    for index in range(len(course.windows)):
        for tau in np.linspace(0.0,1.2,7):
            future_p=p+v*tau+0.5*a*tau*tau
            predicted.append((course.point_clearance(index,future_p,now+tau),index,tau,future_p))
    predicted_clearance,index,tau,query_p=min(predicted,key=lambda item:item[0])
    output=np.asarray(command,float).copy(); intervened=False
    if predicted_clearance < 0.55:
        eps=1.0e-3; gradient=np.zeros(3)
        for axis in range(3):
            delta=np.zeros(3); delta[axis]=eps
            gradient[axis]=(course.point_clearance(index,query_p+delta,now+tau)-
                            course.point_clearance(index,query_p-delta,now+tau))/(2.0*eps)
        norm=np.linalg.norm(gradient)
        if norm>1.0e-8:
            gradient/=norm
            lookahead=2.0e-3
            future_v=v+a*tau
            d_dot=(course.point_clearance(index,query_p+future_v*lookahead,
                                           now+tau+lookahead)-predicted_clearance)/lookahead
            safe_buffer=0.32
            closing=max(0.0,-d_dot)
            braking=closing*closing/(2.0*max(predicted_clearance-safe_buffer,0.05))
            recovery=2.0*max(0.0,safe_buffer-predicted_clearance)/max(tau*tau,0.04)
            required_actual=max(braking+0.8*closing,recovery)
            response=cfg.controller_dt/cfg.acceleration_response_time
            required_command=(required_actual-(1.0-response)*float(gradient@a))/response
            deficit=required_command-float(gradient@output)
            if deficit>0.0:
                output+=deficit*gradient; intervened=True
                normal_component=min(max(required_command,0.0),cfg.max_acceleration)
                tangent=output-float(gradient@output)*gradient
                tangent_limit=math.sqrt(max(0.0,cfg.max_acceleration**2-normal_component**2))
                tangent=unit(tangent)*min(np.linalg.norm(tangent),tangent_limit)
                output=normal_component*gradient+tangent
    return output,intervened,(time.perf_counter()-started)*1000.0,clearance,current_index


def ordered_plane_filter(course: Course, cfg: Config, state, now, command, plan, enabled):
    """Prevent bypassing the active gate plane outside its planned opening."""
    started=time.perf_counter(); output=np.asarray(command,float).copy(); intervened=False
    if enabled:
        p=state[:3]; v=state[3:6]; a=state[6:9]
        center,rotation,center_rate,_=course.pose(plan.gate_index,now)
        gradient=plan.incoming_sign*rotation[:,2]
        signed=float(gradient@(p-center))
        to_point=plan_point_at(course, plan, now)-p
        lateral=to_point-gradient*float(gradient@to_point)
        lateral_error=float(np.linalg.norm(lateral))
        if signed<cfg.approach_distance and lateral_error>=0.45:
            d_dot=float(gradient@(v-center_rate))
            closing=max(0.0,-d_dot)
            available_distance=max(signed-0.80,0.10)
            required_actual=(closing*closing/(2.0*available_distance)
                             +0.80*closing)
            response=cfg.controller_dt/cfg.acceleration_response_time
            required_command=(required_actual-(1.0-response)*float(gradient@a))/response
            intervened=True
            normal_component=min(max(required_command,float(gradient@output),0.0),cfg.max_acceleration)
            lateral_velocity=v-gradient*float(gradient@v)
            tangent=3.0*lateral-2.0*lateral_velocity
            tangent_limit=math.sqrt(max(0.0,cfg.max_acceleration**2-normal_component**2))
            tangent=unit(tangent)*min(np.linalg.norm(tangent),tangent_limit)
            output=normal_component*gradient+tangent
    return output,intervened,(time.perf_counter()-started)*1000.0


def workspace_barrier_filter(cfg: Config, state: np.ndarray, command: np.ndarray):
    """Anticipate the lower/upper workspace boundary under braking limits."""
    output=np.asarray(command,float).copy(); intervened=False
    z,vz,az=float(state[2]),float(state[5]),float(state[8])
    lower_buffer,upper_buffer=0.65,9.30
    required_actual=None
    sign=1.0
    if vz<0.0 and z<2.0:
        distance=max(z-lower_buffer,0.05)
        required_actual=vz*vz/(2.0*distance)+0.8*(-vz)
    elif vz>0.0 and z>7.8:
        distance=max(upper_buffer-z,0.05)
        required_actual=vz*vz/(2.0*distance)+0.8*vz
        sign=-1.0
    if required_actual is not None:
        response=cfg.controller_dt/cfg.acceleration_response_time
        required_command=(required_actual-(1.0-response)*sign*az)/response
        if sign*output[2]<required_command:
            output[2]=sign*required_command; intervened=True
            output=unit(output)*min(np.linalg.norm(output),cfg.max_acceleration)
    return output,intervened


def online_deadline_satisfied(cycle_times, planner_times, mppi_times, deadline_ms=100.0):
    """All online components, including the composed cycle, meet the deadline."""
    groups=(cycle_times,planner_times,mppi_times)
    return all(len(values)>0 and float(np.max(values))<=deadline_ms for values in groups)


def simulate(method: str, seed: int, course: Course, cfg: Config):
    center_only = method == "center_periodic"
    planner=GatePointPlanner(course,cfg,center_only=center_only)
    mppi=LocalSafeMPPI(course,cfg,seed=1000+seed)
    rng=np.random.default_rng(seed)
    state=np.zeros(12); state[:3]=course.start
    now=0.0; gate_index=0; replans=[]; crossings=[]; departure_mode=False
    gate_started_at=0.0; center_fallback=False
    plan=planner.plan(state,now,gate_index,"initial",math.inf); replans.append(plan)
    # CUDA/PyTorch initialization is completed before take-off.  The warm-up
    # command is not executed and is excluded from online timing statistics.
    mppi.command(state,now,plan)
    mppi.latencies_ms.clear(); mppi.safe_ratios.clear(); mppi.no_safe_count=0
    mppi.relevant_gate_counts.clear()
    mppi.controls.zero_()
    positions=[state[:3].copy()]; velocities=[state[3:6].copy()]
    accelerations=[state[6:9].copy()]; jerks=[state[9:12].copy()]
    applied_snaps=[np.zeros(3)]; times=[now]
    previous_safe_ratio=1.0
    barrier_times=[]; barrier_interventions=0; ordered_times=[]; ordered_interventions=0
    workspace_interventions=0
    shield_times=[]; shield_interventions=0; cycle_times=[]
    online_minimum_clearance=math.inf
    finished=False; failure=None
    while now < cfg.max_flight_time:
        cycle_started=time.perf_counter()
        if gate_index < len(course.windows):
            gate_age=now-gate_started_at
            stalled=(gate_age>7.0 and np.linalg.norm(state[3:6])<2.0) or gate_age>15.0
            if not departure_mode and not center_fallback and stalled:
                center_fallback=True
                plan=planner.plan(state,now,gate_index,"center_fallback",math.inf,force_center=True)
                replans.append(plan)
            if departure_mode and (now >= departure_until or np.linalg.norm(state[:3]-plan.point)<0.45):
                plan=planner.plan(state,now,gate_index,"departure_clear",math.inf,
                                  force_center=center_fallback,locked_local=plan.point_local)
                replans.append(plan); departure_mode=False
            if not departure_mode:
                slack=commitment_slack(course,cfg,state,now,plan)
                cand_time,_,cand_point,_,_,_=planner.candidate(state,now,gate_index)
                point_shift=float(np.linalg.norm(cand_point-plan.point)); time_shift=abs(cand_time-plan.cross_time)
                cause=None
                if now-plan.made_at >= cfg.max_plan_age:
                    cause="max_dwell"
                elif previous_safe_ratio < 0.05:
                    cause="rollout_viability"
                elif slack < 0.35 and (point_shift > cfg.point_shift_trigger or time_shift > cfg.time_shift_trigger):
                    cause="commitment"
                elif point_shift > 2.0*cfg.point_shift_trigger:
                    cause="crossing_shift"
                if method == "fixed_gatepoint": cause=None
                if method == "gatepoint_periodic":
                    cause="periodic" if now-plan.made_at >= 0.20 else None
                if cause and now-plan.made_at >= cfg.min_replan_dwell:
                    plan=planner.plan(state,now,gate_index,cause,slack,
                                      force_center=center_fallback,
                                      locked_local=(None if cause=="rollout_viability"
                                                    else plan.point_local))
                    replans.append(plan)
        acceleration_command,safe_ratio=mppi.command(state,now,plan); previous_safe_ratio=safe_ratio
        acceleration_command,intervened,barrier_ms,current_clearance,_=local_barrier_filter(
            course,cfg,state,now,acceleration_command)
        barrier_times.append(barrier_ms); barrier_interventions+=int(intervened)
        online_minimum_clearance=min(online_minimum_clearance,current_clearance)
        acceleration_command,ordered_intervened,ordered_ms=ordered_plane_filter(
            course,cfg,state,now,acceleration_command,plan,
            enabled=(gate_index<len(course.windows) and not departure_mode))
        ordered_times.append(ordered_ms); ordered_interventions+=int(ordered_intervened)
        acceleration_command,workspace_intervened=workspace_barrier_filter(
            cfg,state,acceleration_command)
        workspace_interventions+=int(workspace_intervened)
        shield_started=time.perf_counter()
        acceleration_command,shielded=select_verified_command(
            course,cfg,state,now,acceleration_command,plan,
            departure_mode=(departure_mode or gate_index==len(course.windows)))
        shield_times.append((time.perf_counter()-shield_started)*1000.0)
        shield_interventions+=int(shielded)
        if acceleration_command is None:
            failure="no_verified_one_step_control"
            cycle_times.append((time.perf_counter()-cycle_started)*1000.0)
            break
        dt=cfg.controller_dt; p0=state[:3].copy(); t0=now
        # Smooth fourth-order execution model plus repeatable bounded wind.
        wind=0.18*np.array([math.sin(0.71*now+seed),math.sin(0.53*now+0.4*seed),
                            0.35*math.sin(0.91*now-0.2*seed)])
        wind += rng.uniform(-0.025,0.025,3)
        state,snap_command=integrate_step(state,acceleration_command,cfg,wind)
        p_new,v_new,a_new,j_new=state[:3],state[3:6],state[6:9],state[9:12]
        now+=dt
        positions.append(p_new.copy()); velocities.append(v_new.copy()); accelerations.append(a_new.copy())
        jerks.append(j_new.copy()); applied_snaps.append(snap_command.copy()); times.append(now)
        crossed,tc,clearance=(False,None,None)
        if gate_index < len(course.windows):
            crossed,tc,clearance=passage(course,gate_index,p0,p_new,t0,now,plan.incoming_sign)
        if crossed:
            crossings.append({"gate":gate_index+1,"time_s":tc,"clearance_m":clearance})
            _,cross_rotation,_,_=course.pose(gate_index,float(tc))
            departure_distance=cfg.departure_distance
            crossing_target=plan_point_at(course,plan,float(tc))
            departure=crossing_target-departure_distance*plan.incoming_sign*cross_rotation[:,2]
            gate_index+=1
            gate_started_at=now; center_fallback=False
            if gate_index==len(course.windows):
                # Once all gates are validly crossed, use a synthetic goal plan.
                if np.linalg.norm(p_new-course.goal)<0.45 and np.linalg.norm(v_new)<1.0:
                    finished=True; break
                # Keep the final gate's exit guidance until clear, then finish
                # with a direct dynamically smooth stop at the shared start.
                plan=GatePlan(gate_index-1,now,now+1.0,departure,departure,departure,
                              np.zeros(2),False,plan.incoming_sign,"finish",0.0,math.inf)
            else:
                plan=planner.plan(state,now,gate_index,"gate_pass",math.inf); replans.append(plan)
                plan.approach_point=departure.copy(); plan.point=departure.copy()
                plan.exit_point=departure.copy(); plan.made_at=now
                plan.dynamic_target=False
                departure_mode=True; departure_until=now+1.60
        if gate_index==len(course.windows):
            # Replace the gate-local MPPI target with goal behaviour after the
            # aircraft is safely on the departure side of W7.
            direction=course.goal-state[:3]
            if np.linalg.norm(direction)<0.45 and np.linalg.norm(state[3:6])<1.0:
                finished=True; break
            if np.linalg.norm(state[:3]-plan.point)<0.55:
                # Re-use W7 only as a distant safety geometry; its signed side
                # now selects the exit/goal target, both equal to the goal.
                plan.approach_point=course.goal.copy(); plan.point=course.goal.copy()
                plan.exit_point=course.goal.copy(); plan.made_at=now
                plan.dynamic_target=False
        cycle_times.append((time.perf_counter()-cycle_started)*1000.0)
    positions=np.asarray(positions); velocities=np.asarray(velocities)
    accelerations=np.asarray(accelerations); jerks=np.asarray(jerks); times=np.asarray(times)
    if not finished and failure is None: failure="timeout_or_incomplete_order"
    # Dense interpolation is an independent post-run audit, never an online check.
    dense_t=np.arange(0.0,times[-1]+1.0e-9,0.005)
    dense_p=np.column_stack([np.interp(dense_t,times,positions[:,i]) for i in range(3)])
    min_clearance,collision_samples,per_gate=course.frame_clearance(dense_p,dense_t)
    workspace_ok=bool(positions[:,2].min()>=0.45-1.0e-9
                      and positions[:,2].max()<=9.5+1.0e-9)
    snap=np.asarray(applied_snaps)
    pvajs=np.stack((positions,velocities,accelerations,jerks,snap),axis=1)
    dynamics=AnalyticTOGTCore().sample_dynamics(pvajs)
    rotors=np.asarray(dynamics["rotor_thrusts"])
    body_rate=np.asarray(dynamics["body_rate"])
    speed_norm=np.linalg.norm(velocities,axis=1)
    acceleration_norm=np.linalg.norm(accelerations,axis=1)
    jerk_norm=np.linalg.norm(jerks,axis=1)
    snap_norm=np.linalg.norm(snap,axis=1)
    kinematic_limits_ok=bool(speed_norm.max()<=cfg.max_speed+1.0e-6
                     and acceleration_norm.max()<=cfg.max_acceleration+1.0e-6
                     and jerk_norm.max()<=cfg.max_jerk+1.0e-6
                     and snap_norm.max()<=cfg.max_snap+1.0e-6)
    dynamics_ok=bool(kinematic_limits_ok
                     and rotors.min()>=0.25-1.0e-6 and rotors.max()<=5.0+1.0e-6
                     and np.abs(body_rate).max()<=10.0+1.0e-6)
    if not dynamics_ok and failure is None:
        failure="sampled_dynamics_violation"
    if collision_samples>0 and failure is None:
        failure="dense_sampled_collision"
    if not workspace_ok and failure is None:
        failure="workspace_height_violation"
    timing_ok=online_deadline_satisfied(cycle_times,planner.times_ms,mppi.latencies_ms)
    if not timing_ok and failure is None:
        failure="online_deadline_violation"
    successful=bool(finished and collision_samples==0 and len(crossings)==7
                    and dynamics_ok and workspace_ok and timing_ok)
    def stats(values):
        a=np.asarray(values,float)
        return {"mean":float(np.mean(a)),"p50":float(np.percentile(a,50)),
                "p95":float(np.percentile(a,95)),"p99":float(np.percentile(a,99)),
                "maximum":float(np.max(a))}
    result={
        "method":method,"seed":seed,"success":successful,"failure":failure,
        "sampled_dynamics_satisfied":dynamics_ok,
        "sampled_kinematic_limits_satisfied":kinematic_limits_ok,
        "sampled_workspace_height_satisfied":workspace_ok,
        "online_timing_satisfied":timing_ok,
        "flight_time_s":float(times[-1]),"gates_crossed":len(crossings),"crossings":crossings,
        "minimum_dense_clearance_m":float(min_clearance),"dense_collision_samples":collision_samples,
        "per_gate_clearance":per_gate,"replan_count":len(replans),
        "replan_causes":{cause:sum(p.cause==cause for p in replans) for cause in sorted({p.cause for p in replans})},
        "planner_latency_ms":stats(planner.times_ms),"mppi_latency_ms":stats(mppi.latencies_ms),
        "barrier_latency_ms":stats(barrier_times),"barrier_interventions":barrier_interventions,
        "ordered_filter_latency_ms":stats(ordered_times),
        "ordered_filter_interventions":ordered_interventions,
        "workspace_filter_interventions":workspace_interventions,
        "final_shield_latency_ms":stats(shield_times),
        "final_shield_interventions":shield_interventions,
        "cycle_latency_ms":stats(cycle_times),
        "cycle_over_40ms":int(np.count_nonzero(np.asarray(cycle_times)>40.0)),
        "cycle_over_100ms":int(np.count_nonzero(np.asarray(cycle_times)>100.0)),
        "online_minimum_clearance_m":float(online_minimum_clearance),
        "mppi_over_100ms":int(np.count_nonzero(np.asarray(mppi.latencies_ms)>100.0)),
        "mean_safe_rollout_ratio":float(np.mean(mppi.safe_ratios)),"no_safe_rollout_cycles":mppi.no_safe_count,
        "mean_relevant_gates_per_cycle":float(np.mean(mppi.relevant_gate_counts)),
        "maximum_relevant_gates_per_cycle":int(np.max(mppi.relevant_gate_counts)),
        "maximum_speed_mps":float(speed_norm.max()),
        "maximum_acceleration_mps2":float(acceleration_norm.max()),
        "maximum_jerk_mps3":float(jerk_norm.max()),
        "maximum_snap_mps4":float(snap_norm.max()),
        "minimum_height_m":float(positions[:,2].min()),
        "maximum_height_m":float(positions[:,2].max()),
        "maximum_body_rate_rps":float(np.abs(body_rate).max()),
        "minimum_rotor_thrust_n":float(rotors.min()),"maximum_rotor_thrust_n":float(rotors.max()),
        "sample_count":len(times),"dense_audit_step_s":0.005,
    }
    return result, {"time":times,"position":positions,"velocity":velocities,
                    "acceleration":accelerations,"jerk":jerks,"snap":snap}


def summarize(records):
    output={}
    for method in sorted({r["method"] for r in records}):
        subset=[r for r in records if r["method"]==method]
        success=[r for r in subset if r["success"]]
        output[method]={
            "runs":len(subset),"successes":len(success),
            "success_rate":len(success)/len(subset),
            "mean_flight_time_success_s":float(np.mean([r["flight_time_s"] for r in success])) if success else None,
            "mean_replans":float(np.mean([r["replan_count"] for r in subset])),
            "worst_minimum_clearance_m":float(min(r["minimum_dense_clearance_m"] for r in subset)),
            "worst_mppi_p99_ms":float(max(r["mppi_latency_ms"]["p99"] for r in subset)),
            "worst_planner_max_ms":float(max(r["planner_latency_ms"]["maximum"] for r in subset)),
            "total_mppi_over_100ms":sum(r["mppi_over_100ms"] for r in subset),
        }
    return output


def sha256_file(path: Path):
    digest=hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda:handle.read(1024*1024),b""):
            digest.update(block)
    return digest.hexdigest()


def provenance(device):
    root=Path(__file__).resolve().parents[2]
    files=[
        Path(__file__).resolve(),
        Path(__file__).resolve().with_name("analyze_results.py"),
        root/"convex_timevarying_window"/"togt"/"experiment.py",
        root/"convex_dynamic_seven_window_gazebo"/"course_spec.json",
    ]
    return {
        "created_at":datetime.now().astimezone().isoformat(),
        "python":sys.version,
        "platform":platform.platform(),
        "numpy":np.__version__,
        "torch":torch.__version__,
        "cuda_runtime":torch.version.cuda,
        "device":str(device),
        "gpu":torch.cuda.get_device_name(device) if device.type=="cuda" else None,
        "sha256":{str(path.relative_to(root)):sha256_file(path)
                  for path in files if path.exists()},
    }


def main(argv=None):
    parser=argparse.ArgumentParser()
    parser.add_argument("--seeds",type=int,default=10)
    parser.add_argument("--seed-offset",type=int,default=0)
    parser.add_argument("--methods",nargs="+",default=["center_periodic","fixed_gatepoint","gatepoint_periodic","proposed"])
    parser.add_argument("--outdir",type=Path)
    parser.add_argument("--rollouts",type=int,default=1024)
    args=parser.parse_args(argv)
    device=torch.device("cuda" if torch.cuda.is_available() else "cpu")
    cfg=Config(rollouts=args.rollouts)
    out=args.outdir or Path(__file__).resolve().parent/"results"/datetime.now().strftime("formal_%Y%m%d_%H%M%S")
    out.mkdir(parents=True,exist_ok=True)
    course=Course(device); records=[]
    for method in args.methods:
        for seed in range(args.seed_offset,args.seed_offset+args.seeds):
            result,trajectory=simulate(method,seed,course,cfg); records.append(result)
            np.savez_compressed(out/f"{method}_seed{seed}.npz",**trajectory)
            (out/f"{method}_seed{seed}.json").write_text(json.dumps(result,indent=2)+"\n")
            print(method,seed,result["success"],f"T={result['flight_time_s']:.2f}",
                  f"clear={result['minimum_dense_clearance_m']:.3f}",
                  f"mppi_p99={result['mppi_latency_ms']['p99']:.2f}ms",flush=True)
    summary={"device":str(device),"provenance":provenance(device),
             "config":asdict(cfg),"summary":summarize(records),"records":records}
    (out/"summary.json").write_text(json.dumps(summary,indent=2)+"\n")
    print(json.dumps(summary["summary"],indent=2))


if __name__=="__main__": main()
