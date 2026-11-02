"""Arb enclosure of the stored trajectory's original flatness dynamics.

This verifies a fixed stored polynomial, not all polynomial trajectories or
closed-loop flight. Geometry keeps its independently reported local checker.
"""
import argparse
import json
import math
from pathlib import Path
import time

import casadi as ca
from flint import arb,ctx
import numpy as np

from convex_timevarying_window.dynamic_passage_velocity_minco.hard_dynamics import flat_actuators
from convex_timevarying_window.dynamic_passage_velocity_minco.allocation_mpc import KF
from convex_timevarying_window.single_gate_stop_racing.planner import Config
from .model import PhysicalModel
from ..run import save_json


def split(control,u):
    work=[list(r) for r in control];left=[work[0]];right=[work[-1]]
    for _ in range(len(control)-1):
        work=[[(1-u)*a+u*b for a,b in zip(x,y)] for x,y in zip(work[:-1],work[1:])]
        left.append(work[0]);right.append(work[-1])
    return left,list(reversed(right))


def controls(coefficients,duration,d):
    n=7-d
    power=[[arb(float(coefficients[k+d,j]))*math.factorial(k+d)/math.factorial(k)/arb(float(duration))**d
            for j in range(3)] for k in range(n+1)]
    return [[sum((power[k][j]*math.comb(i,k)/math.comb(n,k) for k in range(i+1)),arb(0))
             for j in range(3)] for i in range(n+1)]


class Tape:
    def __init__(self):
        self.function=flat_actuators.expand()
        f=self.function
        self.code=[(f.instruction_id(i),f.instruction_input(i),f.instruction_output(i),
                    arb(float(f.instruction_constant(i))) if f.instruction_id(i)==ca.OP_CONST else None)
                   for i in range(f.n_instructions())]

    def evaluate(self,inputs):
        work=[arb(0)]*self.function.sz_w();outputs=[[],[],[]]
        for op,ii,oo,constant in self.code:
            if op==ca.OP_INPUT:v=inputs[ii[0]][ii[1]]
            elif op==ca.OP_CONST:v=constant
            elif op==ca.OP_OUTPUT:outputs[oo[0]].append(work[ii[0]]);continue
            else:
                a=work[ii[0]];b=work[ii[1]] if len(ii)>1 else None
                if op==ca.OP_ADD:v=a+b
                elif op==ca.OP_SUB:v=a-b
                elif op==ca.OP_MUL:v=a*b
                elif op==ca.OP_DIV:v=a/b
                elif op==ca.OP_NEG:v=-a
                elif op==ca.OP_SQ:v=a**2
                elif op==ca.OP_SQRT:v=a.sqrt()
                elif op==ca.OP_TWICE:v=2*a
                elif op==ca.OP_INV:v=1/a
                elif op in (ca.OP_POW,ca.OP_CONSTPOW):v=a**b
                else:raise NotImplementedError(op)
            work[oo[0]]=v
        return outputs


def enclosure(control):
    result=[]
    for j in range(3):
        val=control[0][j]
        for row in control[1:]:val=val.union(row[j])
        result.append(val)
    return result


def verify(path,budget_s=120.):
    data=np.load(path);durations=data['durations'];cc=data['coefficients']
    cfg=Config();ctx.prec=128;tape=Tape();began=time.perf_counter()
    queue=[]
    for piece in (0,1):
        queue.append((piece,0.,1.,[controls(cc[piece],durations[piece],d) for d in range(5)]))
    processed=0;accepted=0;max_depth=0
    low=KF*150**2;high=KF*1000**2
    worst_margin=float('inf')
    while queue:
        if time.perf_counter()-began>budget_s:
            return dict(passed=False,status='interval_budget',processed=processed,accepted_boxes=accepted,
                remaining_boxes=len(queue),elapsed_s=time.perf_counter()-began,
                evidence='Arb 128-bit fixed-polynomial dynamics enclosures')
        piece,a,b,ctrl=queue.pop();processed+=1
        bounds=[enclosure(c) for c in ctrl]
        try:
            _,rates,motors=tape.evaluate(bounds[1:])
            margins=[]
            for v,limit in zip(rates,[3.8,3.8,3.49]):
                margins.extend([v+limit,limit-v])
            for f in motors:margins.extend([(f-low)/high,(high-f)/high])
            for j,p in enumerate(bounds[0]):
                margins.extend([(p-cfg.workspace_lower[j])/30,(cfg.workspace_upper[j]-p)/30])
            valid=all(m.is_finite() and m>=0 for m in margins)
        except (ValueError,ZeroDivisionError):valid=False;margins=[]
        if valid:
            accepted+=1
            worst_margin=min(worst_margin,min(float(m.lower()) for m in margins))
            continue
        if (b-a)*durations[piece]<1e-10:
            return dict(passed=False,status='interval_unresolved',piece=piece,u_interval=[a,b],
                processed=processed,accepted_boxes=accepted,elapsed_s=time.perf_counter()-began)
        left=[];right=[]
        for c in ctrl:
            l,r=split(c,arb(.5));left.append(l);right.append(r)
        mid=(a+b)/2
        queue.extend([(piece,mid,b,right),(piece,a,mid,left)])
    return dict(passed=True,status='enclosed',processed=processed,accepted_boxes=accepted,
        minimum_enclosure_margin=worst_margin,elapsed_s=time.perf_counter()-began,
        evidence='Arb 128-bit full-time rotor/rate/workspace enclosures of stored polynomial',
        scope='original nominal flatness model, not PX4/Gazebo tracking, geometry separately checked')


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--plan',type=Path,required=True)
    ap.add_argument('--output',type=Path,required=True);ap.add_argument('--budget-s',type=float,default=120.)
    args=ap.parse_args()
    if args.output.exists():raise FileExistsError(args.output)
    result=verify(args.plan,args.budget_s);save_json(args.output,result);print(json.dumps(result),flush=True)


if __name__=='__main__':main()
