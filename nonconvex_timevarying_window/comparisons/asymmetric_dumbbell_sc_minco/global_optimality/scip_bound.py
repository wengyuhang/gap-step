"""SCIP lower bound: exact PV-MINCO, moving aperture, selected hard constraints.

This is an explicitly recorded relaxation, not a change of the upper problem.
Every retained inequality is necessary for the original physical problem.
"""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import sys
import time

import casadi as ca
import numpy as np

from convex_timevarying_window.single_gate_stop_racing.planner import Config
from convex_timevarying_window.dynamic_passage_velocity_minco.repaired_planner import flat_force
from convex_timevarying_window.dynamic_passage_velocity_minco.hard_dynamics import flat_actuators
from convex_timevarying_window.dynamic_passage_velocity_minco.allocation_mpc import KF
from convex_timevarying_window.single_gate_stop_racing.geometry import symbolic_constraints
from convex_timevarying_window.togt_minco_mpc.trajectory import basis
from ..spin.scenario import build_case,symbolic_pose
from ..run import save_json
from .model import coefficients,bound_knot_velocity


def tape_values(function,x):
    """Numerically evaluate every instruction for an incumbent consistency test."""
    work=np.zeros(function.sz_w());values=[]
    unary={ca.OP_NEG:lambda a:-a,ca.OP_SQ:lambda a:a*a,ca.OP_SQRT:np.sqrt,
        ca.OP_SIN:np.sin,ca.OP_COS:np.cos,ca.OP_EXP:np.exp,ca.OP_LOG:np.log,
        ca.OP_TWICE:lambda a:2*a,ca.OP_INV:lambda a:1/a,ca.OP_FABS:abs}
    binary={ca.OP_ADD:lambda a,b:a+b,ca.OP_SUB:lambda a,b:a-b,
        ca.OP_MUL:lambda a,b:a*b,ca.OP_DIV:lambda a,b:a/b,
        ca.OP_POW:lambda a,b:a**b,ca.OP_CONSTPOW:lambda a,b:a**b,
        ca.OP_FMIN:min,ca.OP_FMAX:max}
    for k in range(function.n_instructions()):
        op=function.instruction_id(k);ii=function.instruction_input(k);oo=function.instruction_output(k)
        if op==ca.OP_OUTPUT:values.append(float(work[ii[0]]));continue
        if op==ca.OP_INPUT:v=x[ii[1]]
        elif op==ca.OP_CONST:v=float(function.instruction_constant(k))
        elif op in unary:v=unary[op](work[ii[0]])
        elif op in binary:v=binary[op](work[ii[0]],work[ii[1]])
        else:raise NotImplementedError(op)
        work[oo[0]]=v;values.append(float(v))
    return values


def export_sx(model,function,variables,initial=None,solution=None,lifting='all',prefix='main'):
    """Export factorable scalar graph; no approximate surrogate/SC callback."""
    from pyscipopt import sin,cos,sqrt,exp,log
    work=[None]*function.sz_w();outputs=[];aux=0
    initial_values=tape_values(function,initial) if initial is not None else None
    for k in range(function.n_instructions()):
        op=function.instruction_id(k)
        ii=function.instruction_input(k);oo=function.instruction_output(k)
        if op==ca.OP_INPUT:
            work[oo[0]]=variables[ii[1]];continue
        if op==ca.OP_CONST:
            work[oo[0]]=float(function.instruction_constant(k));continue
        if op==ca.OP_OUTPUT:
            outputs.append(work[ii[0]]);continue
        a=work[ii[0]];b=work[ii[1]] if len(ii)>1 else None
        if op==ca.OP_ADD:e=a+b
        elif op==ca.OP_SUB:e=a-b
        elif op==ca.OP_MUL:e=a*b
        elif op==ca.OP_DIV:e=a/b if lifting=='grouped' else None
        elif op==ca.OP_NEG:e=-a
        elif op==ca.OP_SQ:e=a*a
        elif op==ca.OP_SQRT:e=sqrt(a)
        elif op==ca.OP_SIN:e=sin(a)
        elif op==ca.OP_COS:e=cos(a)
        elif op==ca.OP_EXP:e=exp(a)
        elif op==ca.OP_LOG:e=log(a)
        elif op in (ca.OP_POW,ca.OP_CONSTPOW):e=a**b
        elif op==ca.OP_TWICE:e=2*a
        elif op==ca.OP_INV:e=1/a if lifting=='grouped' else None
        elif op==ca.OP_FABS:e=abs(a)
        elif op==ca.OP_FMIN:e=(a+b-abs(a-b))/2
        elif op==ca.OP_FMAX:e=(a+b+abs(a-b))/2
        else:raise NotImplementedError(f'CasADi operation {op}')
        if op in (ca.OP_DIV,ca.OP_INV) and lifting!='grouped':
            # SCIP's reciprocal power operator may impose a positive domain.
            # Cross multiplication retains negative denominators; allowing
            # zero only enlarges this lower-bound relaxation, never excludes
            # an original feasible point (all its denominators are nonzero).
            var=model.addVar(f'aux_{prefix}_{aux}',lb=-model.infinity(),ub=model.infinity())
            model.addCons(var*(b if op==ca.OP_DIV else a)==(a if op==ca.OP_DIV else 1.))
            if solution is not None:model.setSolVal(solution,var,initial_values[k])
            aux+=1;work[oo[0]]=var;continue
        nonlinear=op not in (ca.OP_ADD,ca.OP_SUB,ca.OP_NEG,ca.OP_TWICE)
        if lifting=='grouped':
            if op==ca.OP_MUL and (isinstance(a,(float,int)) or isinstance(b,(float,int))):
                nonlinear=False
            if op==ca.OP_DIV and isinstance(b,(float,int)):nonlinear=False
        if nonlinear and not isinstance(e,(int,float,np.number)):
            lo=0. if op in (ca.OP_SQ,ca.OP_SQRT,ca.OP_FABS) else -model.infinity()
            var=model.addVar(f'aux_{prefix}_{aux}',lb=lo,ub=model.infinity())
            model.addCons(var==e);aux+=1;work[oo[0]]=var
            if solution is not None:model.setSolVal(solution,var,initial_values[k])
        else:work[oo[0]]=e
    return outputs,aux


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--outdir',type=Path,required=True)
    ap.add_argument('--budget-s',type=float,default=300.)
    ap.add_argument('--nodes-per-piece',type=int,default=5)
    ap.add_argument('--upper-plan',type=Path)
    ap.add_argument('--full-actuators',action='store_true')
    ap.add_argument('--gap-s',type=float,default=.01)
    ap.add_argument('--lifting',choices=('all','grouped'),default='grouped')
    args=ap.parse_args();out=args.outdir.resolve();out.mkdir(parents=True,exist_ok=False)
    try:import pyscipopt
    except ImportError:
        sys.path.insert(0,'/tmp/codex_single_gate_global_deps');import pyscipopt
    cfg=Config(compile_objective=False);track=build_case(2.);w=track.windows[0]
    xx=ca.SX.sym('physical_time',7);times=xx[:2]
    ctr,rr=symbolic_pose(w,times[0]);point=ctr+rr[:,:2]@xx[2:4]
    cc=coefficients(times,point,xx[4:],ca.DM(track.start),ca.DM(track.goal))
    radius=cfg.radius+cfg.frame_thickness+cfg.passage_margin
    expr=list(symbolic_constraints(w.aperture,ca.vertcat(xx[2:4],0.),radius))
    for piece in range(2):
        for u in np.linspace(0.,1.,args.nodes_per_piece):
            states=[cc[piece*8:piece*8+8,:].T@basis(float(u),d)/times[piece]**d
                    for d in (1,2,3,4)]
            if args.full_actuators:
                _,rates,rotor=flat_actuators(*states)
                expr.extend(ca.vertsplit(1-(rates/ca.DM([3.8,3.8,3.49]))**2))
                expr.extend(ca.vertsplit((rotor-KF*150**2)/(KF*1000**2)))
                expr.extend(ca.vertsplit(1-rotor/(KF*1000**2)))
            else:
                f=flat_force(states[0],states[1])
                expr.append(1-(ca.sumsqr(f)+1e-12)/(4*KF*1000**2)**2)
    fun=ca.Function('lower_constraints',[xx],[ca.vertcat(*expr)],{'cse':True})
    m=pyscipopt.Model('single_spinning_gate_global_relaxation')
    m.setRealParam('limits/time',args.budget_s)
    m.setRealParam('limits/absgap',args.gap_s)
    m.setRealParam('numerics/feastol',1e-7)
    m.setIntParam('parallel/maxnthreads',1)
    m.setLogfile(str(out/'scip.log'))
    vlo,vhi=bound_knot_velocity(cfg)
    lower=np.r_[[.3,.3],[-1.6,-1.6],vlo]
    upper=np.r_[[15.,15.],[3.2,1.6],vhi]
    variables=[m.addVar(f'x{k}',lb=float(l),ub=float(h)) for k,(l,h) in enumerate(zip(lower,upper))]
    m.setObjective(variables[0]+variables[1],'minimize')
    m.addCons(variables[5]>=.1)
    # Original aperture is exactly three rectangles. The middle one cannot
    # contain a center with radius>.2, hence omit it only after that proof.
    left=m.addVar('left_component',vtype='B')
    # Exact constants from the aperture bounding box; no hidden indicator
    # slacks in the fully specified incumbent validation.
    m.addCons(variables[2]<=-.4+3.6*(1-left))
    m.addCons(variables[3]<=.6+1.*(1-left))
    m.addCons(-variables[3]<=.6+1.*(1-left))
    m.addCons(variables[2]>=-1.6*left)
    initial=None;solution=None
    if args.upper_plan:
        dd=np.load(args.upper_plan);initial=dd['x'].copy();initial[:2]=np.exp(initial[:2])
        solution=m.createSol()
        for v,value in zip(variables,initial):m.setSolVal(solution,v,float(value))
        m.setSolVal(solution,left,float(initial[2]<-.4))
        print(json.dumps(dict(incumbent_constraint_min=float(np.asarray(fun(initial)).min()),
            x=initial.tolist())),flush=True)
        # Independently optimize the relaxation before accepting any dual bound.
        from scipy.optimize import minimize
        jj=ca.Function('lower_jac',[xx],[ca.jacobian(fun(xx),xx)])
        bb=list(zip(lower,upper));bb[2]=(0.,3.2)
        local=minimize(lambda z:(z[0]+z[1],np.array([1.,1.,0.,0.,0.,0.,0.])),
            initial,method='SLSQP',jac=True,bounds=bb,
            constraints={'type':'ineq','fun':lambda z:np.asarray(fun(z)).ravel(),
                'jac':lambda z:np.asarray(jj(z))},options={'maxiter':500,'ftol':1e-10})
        check=dict(success=bool(local.success),status=str(local.message),
            duration_s=float(local.x[:2].sum()),gmin=float(np.asarray(fun(local.x)).min()),
            x=local.x.tolist())
        save_json(out/'local_relaxation_check.json',check)
        print(json.dumps(dict(local_relaxation=check)),flush=True)
        if check['gmin']>=-1e-7 and check['duration_s']<initial[:2].sum():
            initial=local.x.copy()
            for v,value in zip(variables,initial):m.setSolVal(solution,v,float(value))
            m.setSolVal(solution,left,float(initial[2]<-.4))
    began=time.perf_counter();gg,naux=export_sx(m,fun,variables,initial,solution,args.lifting)
    for e in gg:
        if isinstance(e,(int,float,np.number)):
            if e<-1e-8:raise RuntimeError('Inconsistent constant relaxation constraint')
        else:m.addCons(e>=0.)
    export_s=time.perf_counter()-began
    incumbent_validated=None
    if solution is not None:
        incumbent_validated=bool(m.checkSol(solution,printreason=True,completely=True))
        print(json.dumps(dict(incumbent_export_valid=incumbent_validated)),flush=True)
        if not incumbent_validated:raise RuntimeError('Exported SCIP model rejects original feasible incumbent')
        m.addSol(solution)
    m.writeProblem(str(out/'model.cip'))
    manifest=dict(config=asdict(cfg),pyscipopt=pyscipopt.__version__,scip=m.version(),
        budget_s=args.budget_s,target_gap_s=args.gap_s,export_s=export_s,
        retained_constraints='crossing geometry and '+('sampled full actuators' if args.full_actuators else 'sampled total-force necessary bounds'),
        omitted_constraints='between-crossing geometry, '+('workspace nodes' if args.full_actuators else 'individual rotors, rates, workspace nodes'),
        target='lower bound for original two-piece PV-MINCO continuous nominal task',
        dynamics_nodes=np.linspace(0.,1.,args.nodes_per_piece).tolist(),
        velocity_box_derivation='exact degree-seven derivative reconstruction from original workspace nodes',
        variable_lower=lower.tolist(),variable_upper=upper.tolist(),expression_count=fun.n_instructions(),
        auxiliary_variables=naux,constraint_count=len(gg),formal_interval_arithmetic=False,
        incumbent_export_validated=incumbent_validated)
    save_json(out/'manifest.json',manifest)
    print(json.dumps(dict(stage='optimizing',export_s=export_s,aux=naux,expr=fun.n_instructions())),flush=True)
    save_json(out/'status.json',dict(finished=False,stage='optimizing'))
    m.optimize()
    log=(out/'scip.log').read_text(errors='replace')
    warning=any(s in log for s in ['unresolved numerical troubles','unbounded variable','numerical troubles in LP'])
    result=dict(status=str(m.getStatus()),dual_bound_s=float(m.getDualbound()),
        relaxation_primal_s=float(m.getPrimalbound()),gap=float(m.getGap()),
        solutions=int(m.getNSols()),search_nodes=int(m.getNNodes()),solve_s=float(m.getSolvingTime()),
        optimal_for_relaxation=str(m.getStatus())=='optimal',
        original_problem_optimality_proven=False,retained_constraints=manifest['retained_constraints'],
        numerical_warning=warning,
        bound_validated=bool(not warning and (m.getStatus()!='infeasible' or not incumbent_validated)))
    if m.getNSols():result['relaxation_x']=[float(m.getVal(v)) for v in variables]
    save_json(out/'result.json',result)
    save_json(out/'status.json',dict(finished=True,stage='finished',result=result))
    print(json.dumps(result),flush=True)


if __name__=='__main__':main()
