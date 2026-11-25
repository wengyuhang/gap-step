"""Shortest periodic arrival schedule for fixed physical gate PVAJ states.

Only a same-trajectory globalization step. All selected paths receive the
original full-frame verification; continuous states remain free afterwards.
"""
import time,json,argparse
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import casadi as ca
from bootstrap import load,HERE
scene,higher,common,_,_=load()
from repair import Planner
from convex_timevarying_window.global_gate_minco_probe.trajectory import INVERSE
from convex_timevarying_window.comparisons.论文实验.实验2 import safety
from convex_timevarying_window.comparisons.论文实验.实验2.soft_dynamics import kernels
from convex_timevarying_window.global_gate_minco_probe.planner import Plan

def coeff(a,b,T):return INVERSE@np.vstack([a[:,k]*T**k for k in range(4)]+[b[:,k]*T**k for k in range(4)])
def run(source,out):
 out.mkdir(parents=True,exist_ok=False);began=time.perf_counter();raw=common.read(source);tr,_=scene.build_case(scene.Case(raw['case']['level']));p=Planner(tr,True,pose_function=scene.pose);T=np.array(raw['durations']);C=np.array(raw['coefficients']);arr=np.cumsum(T);states=[np.zeros((3,4))];states[0][:,0]=tr.start
 for c,t in zip(C,T):states.append(np.column_stack([np.polynomial.polynomial.polyval(1.,np.polynomial.polynomial.polyder(c,k))/t**k for k in range(4)]))
 end=np.zeros((3,4));end[:,0]=tr.goal;states[-1]=end
 z=ca.MX.sym('segment',25);res=kernels.dynamic_residuals('togt',.95)(z);fun=ca.Function('edge_cost',[z],[kernels.slack_integral('togt',.95)(ca.fmax(res,0),z[-1])])
 radius=p.cfg.radius+p.cfg.frame_thickness+p.cfg.passage_margin;basis=np.polynomial.polynomial.polyvander(np.linspace(0,1,p.cfg.nodes_per_piece+1),7)
 nodes=[np.array([0.])]
 for i,w in enumerate(tr.windows):
  period=1/w.motion.frequency;first=int(np.ceil((.3-arr[i])/period));last=0;nodes.append(arr[i]+np.arange(first,last+1)*period)
 stats=dict(evaluated=0,penalty_pruned=0,workspace_pruned=0,local_pruned=0,local_checks=0);edges=[];bound=raw['objective']
 def evaluate(layer,t0,t1):
  dt=t1-t0
  if not .3-1e-9<=dt<=15+1e-9:return None
  c=coeff(states[layer],states[layer+1],dt);stats['evaluated']+=1
  try:pen=float(fun(np.r_[c.T.ravel(),dt]))
  except RuntimeError:return None
  if not np.isfinite(pen) or dt+pen>=bound:stats['penalty_pruned']+=1;return None
  pos=basis@c
  if np.any(pos<np.array(p.cfg.workspace_lower)-1e-7) or np.any(pos>np.array(p.cfg.workspace_upper)+1e-7):stats['workspace_pruned']+=1;return None
  plan=SimpleNamespace(coefficients=np.array([c]),durations=np.array([dt]),made_at=float(t0))
  gates=([layer-1] if layer>0 else [])+([layer] if layer<7 else [])
  for gate in gates:
   stats['local_checks']+=1;check=safety.passage(plan,tr.windows[gate],radius,0)
   if not check['passed']:stats['local_pruned']+=1;return None
  return dict(cost=dt+pen,penalty=pen,duration=dt,coefficients=c)
 for layer in range(7):
  layer_edges={}
  for a,t0 in enumerate(nodes[layer]):
   for b,t1 in enumerate(nodes[layer+1]):
    r=evaluate(layer,float(t0),float(t1))
    if r is not None:layer_edges[(a,b)]=r
  edges.append(layer_edges);print('LAYER',layer,len(nodes[layer]),len(nodes[layer+1]),len(layer_edges),stats,flush=True)
 # Goal duration has no motion phase. Keep multiple feasible tails, so the
 # subsequent full-frame checker can reject nonlocal return interference.
 tails=[]
 for dt in np.unique(np.r_[np.arange(.3,15.00001,.05),T[-1]]):
  r=evaluate(7,float(arr[6]),float(arr[6]+dt))
  if r is not None:tails.append(r)
 tails=sorted(tails,key=lambda r:r['cost'])[:32];blocked=set();rejected=[];selected=None
 for attempt in range(100):
  distance=np.array([0.]);parents=[]
  for layer,E in enumerate(edges):
   next_d=np.full(len(nodes[layer+1]),np.inf);back=np.full(len(next_d),-1,int)
   for (a,b),e in E.items():
    if (layer,a,b) in blocked:continue
    value=distance[a]+e['cost']
    if value<next_d[b]:next_d[b]=value;back[b]=a
   distance=next_d;parents.append(back)
  opts=[(cost+tail['cost'],i,k) for i,cost in enumerate(distance) if np.isfinite(cost) for k,tail in enumerate(tails) if (7,i,k) not in blocked]
  if not opts:break
  value,i,k=min(opts)
  if value>=bound-1e-7:break
  path=[i]
  for back in reversed(parents):path.append(int(back[path[-1]]))
  path=path[::-1];chosen=[edges[l][(path[l],path[l+1])] for l in range(7)]+[tails[k]];ts=np.array([e['duration'] for e in chosen]);cs=np.array([e['coefficients'] for e in chosen]);plan=Plan(0.,0,ts,cs,np.array(raw['variables']),0.,0.,'phase graph',7)
  checks,passed=p.verify(plan)
  if passed:
   selected=dict(objective=value,nominal_time_s=float(ts.sum()),durations=ts.tolist(),coefficients=cs.tolist(),crossing_times_s=np.cumsum(ts)[:-1].tolist(),path=path,tail_index=k);break
  bad=[c for c in checks if not c['passed']];rejected.append(dict(path=path,tail_index=k,bad=[dict(gate=c['gate'],piece=c['piece'],outcome=c['outcome']) for c in bad]))
  for b in bad:
   layer=b['piece'];blocked.add((layer,path[layer],path[layer+1]) if layer<7 else (7,path[7],k))
 result=dict(found=selected is not None,source=str(source),original_time_s=raw['nominal_time_s'],original_objective=raw['objective'],selected=selected,stats=stats,rejected_paths=rejected,wall_s=time.perf_counter()-began,node_counts=[len(n) for n in nodes],feasible_edge_counts=[len(e) for e in edges],terminal_choices=len(tails),same_fixed_gate_states=True,all_selected_frames_checked=True)
 (out/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n');print('DP_RESULT',result['found'],selected['nominal_time_s'] if selected else None,result['wall_s'],flush=True)
if __name__=='__main__':
 a=argparse.ArgumentParser();a.add_argument('--source',type=Path,required=True);a.add_argument('--outdir',type=Path,required=True);q=a.parse_args();run(q.source,q.outdir)
