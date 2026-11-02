"""Uncondensed Bernstein lower model, avoiding rational MINCO elimination.

PVAJ endpoints and C5 join define exactly the original PV-MINCO family.
Individual-rotor/rate and between-crossing safety constraints are relaxed here.
"""
import argparse
import json
import math
from pathlib import Path
import sys
import time

import casadi as ca
import numpy as np

from convex_timevarying_window.single_gate_stop_racing.planner import Config
from convex_timevarying_window.dynamic_passage_velocity_minco.repaired_planner import flat_force
from convex_timevarying_window.dynamic_passage_velocity_minco.allocation_mpc import KF
from convex_timevarying_window.single_gate_stop_racing.geometry import symbolic_constraints
from convex_timevarying_window.dynamic_passage_minco.safety import power_to_bernstein
from ..spin.scenario import build_case
from ..run import save_json
from .scip_bound import export_sx


def difference(ctrl,d):
    work=list(ctrl)
    for _ in range(d):work=[b-a for a,b in zip(work[:-1],work[1:])]
    return work


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--outdir',type=Path,required=True)
    ap.add_argument('--upper-plan',type=Path,required=True)
    ap.add_argument('--budget-s',type=float,default=180.)
    ap.add_argument('--nodes-per-piece',type=int,default=5)
    args=ap.parse_args();out=args.outdir.resolve();out.mkdir(parents=True,exist_ok=False)
    sys.path.insert(0,'/tmp/codex_single_gate_global_deps')
    from pyscipopt import Model,sin,cos
    cfg=Config();track=build_case(2.);window=track.windows[0]
    old=np.load(args.upper_plan);xx=old['x'];durations=old['durations']
    coeff=old['coefficients'];incumbent_s=float(durations.sum())
    numerical_controls=np.array([power_to_bernstein(c) for c in coeff])
    m=Model('uncondensed_original_pv_minco_relaxation')
    m.setLogfile(str(out/'scip.log'));m.setRealParam('limits/time',args.budget_s)
    m.setRealParam('limits/absgap',.01);m.setIntParam('parallel/maxnthreads',1)
    m.setIntParam('presolving/maxrounds',0);m.setIntParam('misc/usesymmetry',0)
    m.setBoolParam('misc/allowstrongdualreds',False)
    times=[m.addVar(f'T{k}',lb=.3,ub=incumbent_s-.3+1e-6) for k in range(2)]
    m.addCons(sum(times)<=incumbent_s+1e-6);m.setObjective(sum(times),'minimize')
    q=[m.addVar('q_u',lb=-1.6,ub=3.2),m.addVar('q_v',lb=-1.6,ub=1.6)]
    left=m.addVar('left',vtype='B')
    m.addCons(q[0]<=-.4+3.6*(1-left));m.addCons(q[0]>=-1.6*left)
    m.addCons(q[1]<=.6+1*(1-left));m.addCons(-q[1]<=.6+1*(1-left))
    u=np.linspace(0,1,17)
    evaluation=np.array([[math.comb(7,k)*s**k*(1-s)**(7-k) for k in range(8)] for s in u])
    pinv=np.linalg.pinv(evaluation)
    workspace_lo=np.asarray(cfg.workspace_lower);workspace_hi=np.asarray(cfg.workspace_upper)
    lower=np.maximum(pinv,0)@np.tile(workspace_lo,(17,1))+np.minimum(pinv,0)@np.tile(workspace_hi,(17,1))-1e-4
    upper=np.maximum(pinv,0)@np.tile(workspace_hi,(17,1))+np.minimum(pinv,0)@np.tile(workspace_lo,(17,1))+1e-4
    ctrl=np.empty((2,8,3),dtype=object)
    for piece in range(2):
        for k in range(8):
            for j in range(3):
                fixed=(piece==0 and k<4) or (piece==1 and k>=4)
                value=float((track.start if piece==0 else track.goal)[j])
                ctrl[piece,k,j]=m.addVar(f'b{piece}_{k}_{j}',
                    lb=value if fixed else float(lower[k,j]),
                    ub=value if fixed else float(upper[k,j]))
    # Position and derivative joins, with common factorial factors cancelled.
    for j in range(3):
        m.addCons(ctrl[0,7,j]==ctrl[1,0,j])
        for d in range(1,6):
            a=difference(ctrl[0,:,j],d)[-1];b=difference(ctrl[1,:,j],d)[0]
            m.addCons(a*times[1]**d==b*times[0]**d)
    m.addCons(7*(ctrl[0,7,1]-ctrl[0,6,1])>=.1*times[0])
    theta=4*np.pi*times[0];phase=6*np.pi*times[0]
    m.addCons(ctrl[0,7,0]==2*sin(phase)+cos(theta)*q[0]-sin(theta)*q[1])
    m.addCons(ctrl[0,7,1]==0.)
    m.addCons(ctrl[0,7,2]==3.2+sin(theta)*q[0]+cos(theta)*q[1])
    # These original workspace rows also make the finite Bernstein box valid.
    for piece in range(2):
        for weights in evaluation:
            for j in range(3):
                pos=sum(float(a)*ctrl[piece,k,j] for k,a in enumerate(weights))
                m.addCons(pos>=float(workspace_lo[j]));m.addCons(pos<=float(workspace_hi[j]))
    sol=m.createSol()
    for k,v in enumerate(times):m.setSolVal(sol,v,float(durations[k]))
    for k,v in enumerate(q):m.setSolVal(sol,v,float(xx[2+k]))
    m.setSolVal(sol,left,float(xx[2]<-.4))
    for ix in np.ndindex(ctrl.shape):m.setSolVal(sol,ctrl[ix],float(numerical_controls[ix]))
    z=ca.SX.sym('q',2)
    gf=ca.Function('cross_geometry',[z],[ca.vertcat(*symbolic_constraints(window.aperture,
        ca.vertcat(z,0.),cfg.radius+cfg.frame_thickness+cfg.passage_margin))])
    g,_=export_sx(m,gf,q,xx[2:4],sol,'grouped',prefix='geometry')
    for e in g:
        if not isinstance(e,(float,int)):m.addCons(e>=0.)
    va=ca.SX.sym('va',6);f=flat_force(va[:3],va[3:])
    ff=ca.Function('total_force',[va],[1-(ca.sumsqr(f)+1e-12)/(4*KF*1000**2)**2])
    # Distinct variable names in every exported factorable subgraph.
    total_aux=0
    for piece in range(2):
        for index,s in enumerate(np.linspace(0,1,args.nodes_per_piece)):
            states=[];numeric=[]
            for d in (1,2):
                degree=7-d
                weights=[math.comb(degree,k)*s**k*(1-s)**(degree-k) for k in range(degree+1)]
                factor=math.factorial(7)/math.factorial(7-d)
                for j in range(3):
                    dd=difference(ctrl[piece,:,j],d)
                    numerator=factor*sum(float(a)*b for a,b in zip(weights,dd))
                    # A physical state variable prevents repeated expression-tree expansion.
                    v=m.addVar(f'd{d}_{piece}_{index}_{j}',lb=-1e10,ub=1e10)
                    m.addCons(v*times[piece]**d==numerator)
                    nd=difference(list(numerical_controls[piece,:,j]),d)
                    value=factor*sum(a*b for a,b in zip(weights,nd))/durations[piece]**d
                    m.setSolVal(sol,v,float(value));states.append(v);numeric.append(value)
            gg,na=export_sx(m,ff,states,np.array(numeric),sol,'grouped',prefix=f'f{piece}_{index}');total_aux+=na
            m.addCons(gg[0]>=0.)
    valid=bool(m.checkSol(sol,printreason=True,completely=True))
    save_json(out/'manifest.json',dict(upper_plan=str(args.upper_plan.resolve()),
        upper_time_s=incumbent_s,incumbent_validated=valid,
        trajectory='equivalent degree-seven C5 PV-MINCO via Bernstein controls',
        nodes_per_piece=args.nodes_per_piece,auxiliary_force_variables=total_aux,
        retained='original workspace nodes, knot geometry, sampled total force, forward speed',
        relaxed='individual rotors, rates, all other geometry',
        budget_s=args.budget_s,model='uncondensed bounded, strong dual reductions disabled'))
    if not valid:raise RuntimeError('Uncondensed model does not match the incumbent')
    m.addSol(sol);m.writeProblem(str(out/'model.cip'))
    save_json(out/'status.json',dict(finished=False,stage='optimizing'))
    m.optimize()
    log=(out/'scip.log').read_text(errors='replace')
    bad=any(s in log for s in ['unresolved numerical troubles','unbounded variable','numerical troubles in LP'])
    row=dict(status=str(m.getStatus()),raw_dual_s=float(m.getDualbound()),
        relaxation_primal_s=float(m.getPrimalbound()),gap=float(m.getGap()),
        solve_s=float(m.getSolvingTime()),nodes=int(m.getNNodes()),
        numerical_warning=bad,bound_accepted=not bad,
        original_optimality_proven=False)
    save_json(out/'result.json',row);save_json(out/'status.json',dict(finished=True,result=row))
    print(json.dumps(row),flush=True)


if __name__=='__main__':main()
