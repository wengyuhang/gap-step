"""Single SLSQP--interval-separation loop for SIP-DynaTOGT."""

from __future__ import annotations
from dataclasses import dataclass
from typing import Any, Callable, Iterable
import numpy as np
from numpy.typing import ArrayLike
from scipy.optimize import OptimizeResult, minimize

from nonconvex_timevarying_window.sc_dynatogt.dynamics import ObjectiveWeights, PenaltyWeights
from nonconvex_timevarying_window.sc_dynatogt.optimizer import ForwardPass, JointTOGTObjective, OptimizationConfig
from nonconvex_timevarying_window.sc_dynatogt.time_mapping import duration_jacobian_diagonal, k_from_durations
from .certificate import certify
from .constraints import initial_witnesses, witness_constraint_values
from .model import CertificateResult, CertificateStatus, ExchangeRecord, PolynomialTrajectory, SIPConfig, SIPProblem, SIPResult, Witness


@dataclass
class _Cache:
    objective:JointTOGTObjective; x:np.ndarray|None=None; forward:ForwardPass|None=None; trajectory:PolynomialTrajectory|None=None
    def evaluate(self,x:ArrayLike)->tuple[ForwardPass,PolynomialTrajectory]:
        values=np.asarray(x,dtype=float)
        if self.x is None or not np.array_equal(values,self.x):
            self.forward=self.objective.forward(values); self.trajectory=PolynomialTrajectory.from_minco(self.forward.trajectory); self.x=values.copy()
        assert self.forward is not None and self.trajectory is not None; return self.forward,self.trajectory


def _objective(problem:SIPProblem,config:SIPConfig)->JointTOGTObjective:
    if problem.track is None: raise ValueError("solve requires SIPProblem.from_track(); replay-only problems can only be certified")
    return JointTOGTObjective(problem.track,OptimizationConfig(initial_speed=config.initial_speed,minimum_initial_duration=config.minimum_initial_duration,max_iterations=config.slsqp_max_iterations,objective_weights=ObjectiveWeights(time=1.0,snap_energy=0.0),penalty_weights=PenaltyWeights(velocity=0.0,collective_thrust=0.0,body_rate=0.0,rotor_thrust=0.0),dynamic_limits=config.dynamic_limits,quadrotor=config.quadrotor))


def _slsqp(problem:SIPProblem,objective:JointTOGTObjective,x0:np.ndarray,active:tuple[Witness,...],config:SIPConfig)->tuple[OptimizeResult,_Cache]:
    cache=_Cache(objective); temporal=objective.temporal_dimension; expected: int|None=None
    def value(x):
        try: return float(np.sum(cache.evaluate(x)[0].durations))
        except Exception: return 1e30
    def jac(x):
        values=np.asarray(x,dtype=float)
        try:
            forward,_=cache.evaluate(values); result=np.zeros_like(values); result[:temporal]=duration_jacobian_diagonal(forward.k); return result
        except Exception: return np.clip(values,-1e6,1e6)*1e12
    def constraints(x):
        nonlocal expected
        try: result=witness_constraint_values(problem,cache.evaluate(x)[1],active,config)
        except Exception:
            if expected is None: expected=len(witness_constraint_values(problem,cache.evaluate(x0)[1],active,config))
            result=np.full(expected,-1e12)
        if expected is None: expected=len(result)
        if len(result)!=expected: raise RuntimeError("constraint vector changed shape")
        return result
    result=minimize(value,np.asarray(x0),method="SLSQP",jac=jac,constraints=({"type":"ineq","fun":constraints},),options={"maxiter":config.slsqp_max_iterations,"ftol":config.slsqp_ftol,"disp":False})
    cache.evaluate(result.x); return result,cache


def _dilate_time(objective:JointTOGTObjective,x:np.ndarray,scale:float)->np.ndarray:
    """Deterministic Phase-I restart seed; waypoints are unchanged."""
    if scale==1.0: return np.asarray(x,dtype=float).copy()
    forward=objective.forward(x); out=np.asarray(x,dtype=float).copy()
    out[:objective.temporal_dimension]=k_from_durations(forward.durations*scale)
    return out


def _restore_feasibility(problem:SIPProblem,objective:JointTOGTObjective,x0:np.ndarray,active:tuple[Witness,...],config:SIPConfig)->tuple[np.ndarray,bool]:
    """Phase I: minimize a common relaxation of the current finite SIP set."""
    best=np.asarray(x0,dtype=float).copy(); best_r=np.inf
    for scale in config.feasibility_time_scales:
        seed=_dilate_time(objective,best,scale); cache=_Cache(objective); expected: int|None=None
        def finite(x:ArrayLike)->np.ndarray:
            nonlocal expected
            try: values=witness_constraint_values(problem,cache.evaluate(x)[1],active,config)
            except Exception:
                if expected is None: expected=len(witness_constraint_values(problem,cache.evaluate(seed)[1],active,config))
                values=np.full(expected,-1e12)
            if expected is None: expected=len(values)
            if len(values)!=expected: raise RuntimeError("constraint vector changed shape")
            return values
        r0=max(0.0,float(-np.min(finite(seed))))+1e-4; z0=np.concatenate((seed,[r0]))
        def objective_r(z:ArrayLike)->float: return float(np.asarray(z,dtype=float)[-1])
        def jac_r(z:ArrayLike)->np.ndarray:
            out=np.zeros_like(np.asarray(z,dtype=float)); out[-1]=1.0; return out
        def constraints_r(z:ArrayLike)->np.ndarray:
            values=np.asarray(z,dtype=float); return np.concatenate((finite(values[:-1])+values[-1],[values[-1]]))
        result=minimize(objective_r,z0,method="SLSQP",jac=jac_r,constraints=({"type":"ineq","fun":constraints_r},),options={"maxiter":config.feasibility_max_iterations,"ftol":config.slsqp_ftol,"disp":False})
        candidate=np.asarray(result.x[:-1],dtype=float); relaxation=max(0.0,float(result.x[-1]))
        if relaxation<best_r: best,best_r=candidate,relaxation
        if bool(result.success) and relaxation<=config.feasibility_tolerance: return candidate,True
    return best,False


def _add(existing:list[Witness],new:tuple[Witness,...])->int:
    keys={w.key() for w in existing}; added=0
    for witness in new:
        if witness.key() not in keys: existing.append(witness); keys.add(witness.key()); added+=1
    return added


def solve(
    problem:SIPProblem,
    config:SIPConfig|None=None,
    *,
    initial_x:ArrayLike|None=None,
    active_witnesses:Iterable[Witness]|None=None,
    certify_initial:bool=True,
    progress:Callable[[ExchangeRecord],None]|None=None,
    _certifier:Callable[[SIPProblem,PolynomialTrajectory,SIPConfig],CertificateResult]=certify,
    _initial_witness_provider:Callable[[SIPProblem,int,SIPConfig],tuple[Witness,...]]=initial_witnesses,
)->SIPResult:
    settings=config or SIPConfig(); objective=_objective(problem,settings); start=objective.initial_guess() if initial_x is None else np.asarray(initial_x,dtype=float); objective.split(start)
    final_forward=objective.forward(start); final_traj=PolynomialTrajectory.from_minco(final_forward.trajectory); active=list(_initial_witness_provider(problem,final_traj.num_segments,settings)); history=[]; final_opt:OptimizeResult|None=None; report:CertificateResult|None=None; iterations=0
    if active_witnesses is not None: _add(active,tuple(active_witnesses))
    initial_report=(
        _certifier(problem,final_traj,settings)
        if certify_initial
        else CertificateResult(
            CertificateStatus.VIOLATED,
            "initial certificate skipped because verified witnesses were supplied",
            0,0,0,None,None,
        )
    )
    incumbent=(np.asarray(start).copy(),final_forward,final_traj,initial_report,False) if initial_report.certified else None
    recovery=(np.asarray(start).copy(),final_forward,final_traj,initial_report,False)
    if initial_report.status is CertificateStatus.VIOLATED:
        _add(active,initial_report.witnesses)
    for outer in range(settings.max_exchange_iterations):
        try:
            final_opt,cache=_slsqp(problem,objective,start,tuple(active),settings); iterations+=int(getattr(final_opt,"nit",0)); final_forward,final_traj=cache.evaluate(final_opt.x); start=np.asarray(final_opt.x)
        except Exception as error:
            report=CertificateResult(CertificateStatus.NUMERICAL_FAILURE,f"SLSQP failed closed: {type(error).__name__}: {error}",0,0,0,None,None); record=ExchangeRecord(outer,False,float(np.sum(final_forward.durations)),len(active),report.status,0); history.append(record)
            if progress is not None: progress(record)
            break
        report=_certifier(problem,final_traj,settings); record=ExchangeRecord(outer,bool(final_opt.success),final_traj.total_time,len(active),report.status,report.checked_cells); history.append(record)
        if progress is not None: progress(record)
        if bool(final_opt.success) and np.isfinite(final_traj.total_time):
            recovery=(np.asarray(start).copy(),final_forward,final_traj,report,True)
        if report.status is CertificateStatus.CERTIFIED_FEASIBLE:
            if incumbent is None or final_traj.total_time < incumbent[2].total_time: incumbent=(np.asarray(start).copy(),final_forward,final_traj,report,bool(final_opt.success))
            break
        if report.status is not CertificateStatus.VIOLATED: break
        added=_add(active,report.witnesses)
        if added==0:
            # Repeating a violated witness is finite-NLP failure, never SIP
            # convergence. Repair the same separated constraint set first.
            seed=np.asarray(recovery[0] if not bool(final_opt.success) else start).copy()
            start,feasible=_restore_feasibility(problem,objective,seed,tuple(active),settings)
            if not feasible: break
            continue
        # A separating witness makes the just-certified candidate infeasible for
        # the next finite NLP.  When a certified incumbent is available, restart
        # SLSQP from that point: every added witness is then feasible and SLSQP
        # does not have to recover from an arbitrarily aggressive time collapse.
        if incumbent is not None:
            start=np.asarray(incumbent[0]).copy()
        elif not bool(final_opt.success):
            # Never propagate an unsuccessful SLSQP point.  Such points can
            # contain useful separating witnesses, but unconstrained TOGT time
            # variables may otherwise explode by many orders of magnitude.
            start=np.asarray(recovery[0]).copy()
    assert report is not None
    selected_optimizer_success=bool(final_opt.success) if final_opt is not None else False
    retained_incumbent = incumbent is not None and (report.status is not CertificateStatus.CERTIFIED_FEASIBLE or incumbent[2].total_time < final_traj.total_time)
    if retained_incumbent:
        start,final_forward,final_traj,report,selected_optimizer_success=incumbent
    elif report.status is not CertificateStatus.CERTIFIED_FEASIBLE and not selected_optimizer_success:
        start,final_forward,final_traj,_,selected_optimizer_success=recovery
        report=_certifier(problem,final_traj,settings)
    messages={CertificateStatus.CERTIFIED_FEASIBLE:("best certified incumbent retained instead of the final local SLSQP candidate" if retained_incumbent else "locally optimized candidate passed the complete interval certificate"),CertificateStatus.VIOLATED:"no certified candidate: final candidate violates a hard constraint",CertificateStatus.UNRESOLVED:"no certified candidate: interval budget was insufficient",CertificateStatus.NUMERICAL_FAILURE:report.reason}
    return SIPResult(report.status,messages[report.status],np.asarray(start),final_traj,np.asarray(final_forward.durations),np.asarray(final_forward.traversal_times),np.asarray(final_forward.waypoints),report,tuple(history),selected_optimizer_success,iterations,tuple(active))


__all__=["solve"]
