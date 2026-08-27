"""Adaptive, fail-closed interval separator for SIP-DynaTOGT."""

from __future__ import annotations
from dataclasses import dataclass
from typing import Any, Callable
import numpy as np

from .constraints import dynamic_residual_values, point_singularity_residual_values, safety_residual_value
from .intervals import FlatnessIndeterminate, IntervalDependencyError, boundary_interval, boundary_parameter_spans, ctx, dynamic_residual_intervals, flatness_interval, global_time_interval, interval_ball, require_flint, safety_residual_from_interval_components, window_state_interval
from .model import CertificateResult, CertificateStatus, PolynomialTrajectory, SIPConfig, SIPProblem, Witness


@dataclass(frozen=True)
class _DynamicCell: segment:int; lo:float; hi:float; depth:int=0
@dataclass(frozen=True)
class _SafetyCell: window:int; boundary:int; segment:int; tlo:float; thi:float; ulo:float; uhi:float; depth:int=0
@dataclass
class _Counters:
    checked:int=0; depth:int=0; safety:float=float("inf"); dynamics:float=float("inf")


def _trajectory(value:Any)->PolynomialTrajectory: return value if isinstance(value,PolynomialTrajectory) else PolynomialTrajectory.from_minco(value)
def _result(status:CertificateStatus,reason:str,bits:int,c:_Counters,w:tuple[Witness,...]=())->CertificateResult:
    return CertificateResult(status,reason,bits,c.checked,c.depth,None if not np.isfinite(c.safety) else c.safety,None if not np.isfinite(c.dynamics) else c.dynamics,w)


def _dynamic_witness(traj:PolynomialTrajectory,segment:int,tau:float,config:SIPConfig,kind:str|None=None,source:str="interval")->Witness:
    try:
        residuals=dynamic_residual_values(traj,segment,tau,config); kind=kind if kind in residuals else max(residuals,key=residuals.__getitem__); residual=residuals[kind]
    except Exception:
        try:
            residuals=point_singularity_residual_values(traj,segment,tau,config)
            if kind not in residuals or residuals[kind]<=config.violation_tolerance: kind=max(residuals,key=residuals.__getitem__)
            residual=residuals[kind]
        except Exception: kind,residual=kind or "specific_force_singularity",float("inf")
    return Witness(kind,segment,tau,float(residual),source=source)


def _dynamic_counterexample(traj:PolynomialTrajectory,segment:int,lo:float,hi:float,config:SIPConfig,kinds:tuple[str,...],source:str)->Witness|None:
    """Check subdivision endpoints as well as its midpoint.

    A violating instant can lie exactly on a dyadic split.  No adjacent closed
    interval is then uniformly violating, so an interval-only violation test
    can remain undecided even though direct evaluation supplies a concrete
    counterexample.
    """
    found=[]
    for tau in dict.fromkeys((lo,(lo+hi)/2,hi)):
        for kind in kinds:
            witness=_dynamic_witness(traj,segment,tau,config,kind,source)
            # Singularity residuals are bounded above by floor**2 (1e-12 by
            # default), so the general 1e-9 witness tolerance would suppress
            # even an exact zero denominator.
            tolerance=0.0 if witness.kind.endswith("singularity") else config.violation_tolerance
            if witness.residual>tolerance: found.append(witness)
    return max(found,key=lambda item:item.residual) if found else None


def _safety_witness(problem:SIPProblem,traj:PolynomialTrajectory,cell:_SafetyCell,config:SIPConfig,source:str)->Witness:
    tau,u=(cell.tlo+cell.thi)/2,(cell.ulo+cell.uhi)/2; provisional=Witness("safety",cell.segment,tau,0.0,cell.window,cell.boundary,u,source)
    try: residual=safety_residual_value(problem,traj,provisional,config)
    except Exception: residual=float("inf")
    return Witness("safety",cell.segment,tau,float(residual),cell.window,cell.boundary,u,source)


def _coarse(problem:SIPProblem,traj:PolynomialTrajectory,config:SIPConfig)->tuple[Witness,...]:
    # These dyadic points are only a counterexample accelerator.  Boundary
    # objects remain exact curves and a candidate with no sampled violation
    # still has to pass the full interval cover below.
    separator_nodes=tuple(
        float(value) for value in np.linspace(0.0,1.0,config.separator_grid_size)
    )
    found=[]
    for segment in range(traj.num_segments):
        for tau in separator_nodes:
            try:
                for kind,residual in dynamic_residual_values(traj,segment,tau,config).items():
                    if residual>config.violation_tolerance: found.append(Witness(kind,segment,tau,residual,source="coarse"))
            except Exception: found.append(_dynamic_witness(traj,segment,tau,config,source="coarse"))
        for wi,window in enumerate(problem.windows):
            for bi,boundary in enumerate(window.boundary):
                for lo,hi in boundary_parameter_spans(boundary):
                    for tau in separator_nodes:
                        for fraction in separator_nodes:
                            u=lo+(hi-lo)*fraction
                            witness=Witness("safety",segment,tau,0.0,wi,bi,u,"coarse")
                            try: residual=safety_residual_value(problem,traj,witness,config)
                            except Exception: continue
                            if residual>config.violation_tolerance: found.append(Witness("safety",segment,tau,residual,wi,bi,u,"coarse"))
    found.sort(key=lambda w:w.residual,reverse=True); output=[]; keys=set()
    for witness in found:
        if witness.key() not in keys: output.append(witness); keys.add(witness.key())
        if len(output)>=config.max_witnesses_per_iteration: break
    return tuple(output)


def _split_dynamic(cell:_DynamicCell)->tuple[_DynamicCell,_DynamicCell]:
    mid=(cell.lo+cell.hi)/2; return _DynamicCell(cell.segment,cell.lo,mid,cell.depth+1),_DynamicCell(cell.segment,mid,cell.hi,cell.depth+1)
def _split_safety(cell:_SafetyCell,*,force_time:bool=False)->tuple[_SafetyCell,_SafetyCell]:
    if force_time or cell.thi-cell.tlo>=cell.uhi-cell.ulo:
        mid=(cell.tlo+cell.thi)/2; return _SafetyCell(cell.window,cell.boundary,cell.segment,cell.tlo,mid,cell.ulo,cell.uhi,cell.depth+1),_SafetyCell(cell.window,cell.boundary,cell.segment,mid,cell.thi,cell.ulo,cell.uhi,cell.depth+1)
    mid=(cell.ulo+cell.uhi)/2; return _SafetyCell(cell.window,cell.boundary,cell.segment,cell.tlo,cell.thi,cell.ulo,mid,cell.depth+1),_SafetyCell(cell.window,cell.boundary,cell.segment,cell.tlo,cell.thi,mid,cell.uhi,cell.depth+1)


SafetySpanProvider = Callable[
    [SIPProblem, PolynomialTrajectory, SIPConfig, int],
    tuple[dict[tuple[int, int], tuple[tuple[float, float], ...]], int, int, str | None],
]


def _at_precision(problem:SIPProblem,traj:PolynomialTrajectory,config:SIPConfig,bits:int,safety_span_provider:SafetySpanProvider|None=None)->CertificateResult:
    old=int(ctx.prec); ctx.prec=bits; c=_Counters(); unresolved_reason=None
    violations=[]; violation_keys=set(); violation_reason=None
    def record_violation(witness:Witness,reason:str)->bool:
        nonlocal violation_reason
        key=witness.key()
        if key not in violation_keys:
            violations.append(witness); violation_keys.add(key)
            violation_reason=violation_reason or reason
        return len(violations)>=config.max_witnesses_per_iteration
    try:
        stack=[_DynamicCell(i,0.0,1.0) for i in reversed(range(traj.num_segments))]
        while stack:
            if c.checked>=config.max_cells:
                if violations: return _result(CertificateStatus.VIOLATED,violation_reason or "dynamic violations found",bits,c,tuple(violations))
                return _result(CertificateStatus.UNRESOLVED,"interval-cell budget exhausted during dynamics proof",bits,c)
            cell=stack.pop(); c.checked+=1; c.depth=max(c.depth,cell.depth); tau=interval_ball(cell.lo,cell.hi)
            try: residuals=dynamic_residual_intervals(flatness_interval(traj,cell.segment,tau,config),config)
            except FlatnessIndeterminate as error:
                if error.residual>0:
                    witness=_dynamic_witness(traj,cell.segment,(cell.lo+cell.hi)/2,config,error.kind,f"arb-{bits}")
                    if record_violation(witness,f"flatness singularity proved: {error.kind}"): return _result(CertificateStatus.VIOLATED,violation_reason or "dynamic violations found",bits,c,tuple(violations))
                    continue
                witness=_dynamic_counterexample(traj,cell.segment,cell.lo,cell.hi,config,(error.kind,),f"arb-{bits}")
                if witness is not None:
                    if record_violation(witness,f"flatness singularity found: {witness.kind}"): return _result(CertificateStatus.VIOLATED,violation_reason or "dynamic violations found",bits,c,tuple(violations))
                    continue
                if cell.depth>=config.max_depth or cell.hi-cell.lo<=config.min_time_width:
                    unresolved_reason=unresolved_reason or f"could not separate {error.kind}"
                    continue
                stack.extend(reversed(_split_dynamic(cell))); continue
            uncertain=[]; proved=None
            for kind,residual in residuals:
                if residual>0:
                    proved=_dynamic_witness(traj,cell.segment,(cell.lo+cell.hi)/2,config,kind,f"arb-{bits}")
                    break
                if residual<=0: c.dynamics=min(c.dynamics,max(0.0,-float(residual.upper())))
                else: uncertain.append(kind)
            if proved is not None:
                if record_violation(proved,f"continuous dynamic violation proved: {proved.kind}"): return _result(CertificateStatus.VIOLATED,violation_reason or "dynamic violations found",bits,c,tuple(violations))
                continue
            if uncertain:
                witness=_dynamic_counterexample(traj,cell.segment,cell.lo,cell.hi,config,tuple(uncertain),f"arb-{bits}")
                if witness is not None:
                    if record_violation(witness,f"dynamic counterexample found: {witness.kind}"): return _result(CertificateStatus.VIOLATED,violation_reason or "dynamic violations found",bits,c,tuple(violations))
                    continue
                if cell.depth>=config.max_depth or cell.hi-cell.lo<=config.min_time_width:
                    unresolved_reason=unresolved_reason or "dynamic interval undecidable at limit"
                    continue
                stack.extend(reversed(_split_dynamic(cell)))

        safety=[]
        if safety_span_provider is None:
            safety_spans={(segment,wi):((0.0,1.0),) for segment in range(traj.num_segments) for wi in range(len(problem.windows))}
        else:
            safety_spans,plane_checked,plane_depth,plane_error=safety_span_provider(problem,traj,config,max(0,config.max_cells-c.checked))
            c.checked+=plane_checked; c.depth=max(c.depth,plane_depth)
            if plane_error is not None:
                return _result(CertificateStatus.UNRESOLVED,plane_error,bits,c)
        for segment in reversed(range(traj.num_segments)):
            for wi in reversed(range(len(problem.windows))):
                window=problem.windows[wi]
                for bi in reversed(range(len(window.boundary))):
                    for tlo,thi in reversed(safety_spans.get((segment,wi),())):
                        for lo,hi in reversed(boundary_parameter_spans(window.boundary[bi])): safety.append(_SafetyCell(wi,bi,segment,tlo,thi,lo,hi))
        safety_flat_cache={}
        safety_window_cache={}
        while safety:
            if c.checked>=config.max_cells:
                if violations: return _result(CertificateStatus.VIOLATED,violation_reason or "whole-body violations found",bits,c,tuple(violations))
                return _result(CertificateStatus.UNRESOLVED,"interval-cell budget exhausted during whole-body proof",bits,c)
            cell=safety.pop(); c.checked+=1; c.depth=max(c.depth,cell.depth); tau,u=interval_ball(cell.tlo,cell.thi),interval_ball(cell.ulo,cell.uhi); window=problem.windows[cell.window]
            time_key=(cell.segment,cell.tlo,cell.thi)
            window_key=(cell.window,*time_key)
            try:
                if time_key not in safety_flat_cache: safety_flat_cache[time_key]=flatness_interval(traj,cell.segment,tau,config)
                if window_key not in safety_window_cache: safety_window_cache[window_key]=window_state_interval(window,global_time_interval(traj,cell.segment,tau))
                q=boundary_interval(window.boundary[cell.boundary],u)
                residual=safety_residual_from_interval_components(safety_flat_cache[time_key],safety_window_cache[window_key],q,config)
            except FlatnessIndeterminate as error:
                if error.residual>0:
                    witness=_dynamic_witness(traj,cell.segment,(cell.tlo+cell.thi)/2,config,error.kind,f"arb-{bits}")
                    if record_violation(witness,f"flatness singularity prevents body pose: {error.kind}"): return _result(CertificateStatus.VIOLATED,violation_reason or "violations found",bits,c,tuple(violations))
                    continue
                witness=_dynamic_counterexample(traj,cell.segment,cell.tlo,cell.thi,config,(error.kind,),f"arb-{bits}")
                if witness is not None:
                    if record_violation(witness,f"flatness singularity found while proving body pose: {witness.kind}"): return _result(CertificateStatus.VIOLATED,violation_reason or "violations found",bits,c,tuple(violations))
                    continue
                if cell.depth>=config.max_depth or cell.thi-cell.tlo<=config.min_time_width:
                    unresolved_reason=unresolved_reason or f"body pose interval crosses {error.kind}"
                    continue
                safety.extend(reversed(_split_safety(cell,force_time=True))); continue
            if residual<=0: c.safety=min(c.safety,max(0.0,-float(residual.upper()))); continue
            if residual>0:
                witness=_safety_witness(problem,traj,cell,config,f"arb-{bits}")
                if record_violation(witness,"whole-body clearance violation proved"): return _result(CertificateStatus.VIOLATED,violation_reason or "whole-body violations found",bits,c,tuple(violations))
                continue
            witness=_safety_witness(problem,traj,cell,config,f"arb-{bits}")
            if witness.residual>config.violation_tolerance:
                if record_violation(witness,"whole-body counterexample found"): return _result(CertificateStatus.VIOLATED,violation_reason or "whole-body violations found",bits,c,tuple(violations))
                continue
            if cell.depth>=config.max_depth or (cell.thi-cell.tlo<=config.min_time_width and cell.uhi-cell.ulo<=config.min_boundary_width):
                unresolved_reason=unresolved_reason or "whole-body interval undecidable at limit"
                continue
            safety.extend(reversed(_split_safety(cell)))
        if violations:
            return _result(CertificateStatus.VIOLATED,violation_reason or "continuous violations found",bits,c,tuple(violations))
        if unresolved_reason is not None:
            return _result(CertificateStatus.UNRESOLVED,unresolved_reason,bits,c)
        return _result(CertificateStatus.CERTIFIED_FEASIBLE,"every hard residual is non-positive on a rigorous finite full-domain cover",bits,c)
    finally: ctx.prec=old


def certify(problem:SIPProblem,trajectory:Any,config:SIPConfig|None=None,*,_safety_span_provider:SafetySpanProvider|None=None)->CertificateResult:
    settings=config or SIPConfig()
    try:
        require_flint(); traj=_trajectory(trajectory)
        if traj.num_segments!=len(problem.windows)+1: raise ValueError("N windows require N+1 trajectory pieces")
    except (IntervalDependencyError,TypeError,ValueError) as error: return CertificateResult(CertificateStatus.NUMERICAL_FAILURE,str(error),0,0,0,None,None)
    try: coarse=_coarse(problem,traj,settings)
    except Exception as error: return CertificateResult(CertificateStatus.NUMERICAL_FAILURE,f"coarse evaluation failed closed: {type(error).__name__}: {error}",0,0,0,None,None)
    if coarse: return CertificateResult(CertificateStatus.VIOLATED,"finite points expose violations; no safety claim is made",0,0,0,None,None,coarse)
    last=None
    for bits in settings.precision_bits:
        try: last=_at_precision(problem,traj,settings,int(bits),_safety_span_provider)
        except Exception as error: last=CertificateResult(CertificateStatus.NUMERICAL_FAILURE,f"interval evaluation failed closed: {type(error).__name__}: {error}",int(bits),0,0,None,None)
        if last.status in (CertificateStatus.CERTIFIED_FEASIBLE,CertificateStatus.VIOLATED): return last
    assert last is not None; return last


__all__=["certify"]
