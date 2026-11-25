"""Released TOGT MINCO/L-BFGS/x500 objective with p_i=c_i(t_i)+R_i q_i.

Only moving-gate geometry and its full chain rule are adapted. No PVAJ
interior variables, no added trajectory safety filter or safety objective.
"""
import argparse,copy,json,time
from pathlib import Path
import casadi as ca
import numpy as np
from convex_dynamic_seven_window_gazebo.x500_togt.native_backend import AnalyticTOGTCore as X500Core
from convex_timevarying_window.togt.native_backend import AnalyticTOGTCore as ReleasedSolver
from nonconvex_timevarying_window.sc_dynatogt.time_mapping import durations_from_k,k_from_durations,duration_jacobian_diagonal
from nonconvex_timevarying_window.sc_dynatogt.minco import MincoSnap,BoundaryState
from convex_timevarying_window.single_gate_stop_racing import point_mapping
from convex_timevarying_window.comparisons.single_gate_constant_velocity.run import save
from .scenario import Case,build_case,pose,manifest

INSTANT_INSET=.38594472+.1

class Objective:
    def __init__(self,track,center=False):
        self.track=track;self.center=center;self.n=8;self.core=X500Core();self.solver=ReleasedSolver()
        self.head=np.column_stack([track.start,np.zeros((3,3))]);self.tail=np.column_stack([track.goal,np.zeros((3,3))])
        self.apertures=[];self.slices=[];cursor=8
        for w in track.windows:
            ap=copy.copy(w.aperture)
            if ap.kind=='circle':ap.radius-=INSTANT_INSET
            else:ap.physical_vertices=ap.physical_vertices-np.sign(ap.physical_vertices)*INSTANT_INSET
            if ap.kind=='circle' and ap.radius<=0:raise ValueError('no instantaneous body opening')
            self.apertures.append(ap);dim=0 if center else point_mapping.dimension(ap);self.slices.append(slice(cursor,cursor+dim));cursor+=dim
        self.dimension=cursor
        x=ca.MX.sym('dyn_togt_x',cursor);k=x[:8]
        T=ca.vertcat(*[ca.if_else(q>0,(.5*q+1)*q+1,1/((.5*q-1)*q+1)) for q in ca.vertsplit(k)])
        crossings=ca.cumsum(T);P=[]
        for i,w in enumerate(track.windows):
            c,R=pose(w,crossings[i]);q=ca.DM.zeros(2) if center else point_mapping.symbolic(self.apertures[i],x[self.slices[i]])
            P.append(c+R[:,:2]@q)
        P=ca.horzcat(*P)
        self.geometry=ca.Function('dynamic_togt_geometry',[x],[T,P,ca.jacobian(ca.vec(P),x)])
    def value_and_gradient(self,x):
        x=np.asarray(x,float);T,P,JP=self.geometry(x);T=np.array(T).ravel();P=np.array(P).T
        if not np.isfinite(T).all() or not np.isfinite(P).all() or np.min(T)<1e-5 or np.max(T)>1000:
            raise ValueError('invalid numerical trajectory duration')
        cost,gp,gt=self.core.value_and_gradient(self.head,self.tail,P,T)
        gradient=np.array(JP).T@gp.ravel();gradient[:8]+=duration_jacobian_diagonal(x[:8])*gt
        if not np.isfinite(cost) or not np.isfinite(gradient).all():raise ValueError('invalid objective/gradient')
        return cost,gradient
    def seeds(self):
        nodes=np.vstack([self.track.start,*[w.center0 for w in self.track.windows],self.track.goal]);T=np.maximum(.8,np.linalg.norm(np.diff(nodes,axis=0),axis=1)/6.5);T[0]+=.5
        distance=np.linalg.norm(nodes[-1]-nodes[-2]);T[-1]=max(.8,np.sqrt(6*distance/8),(60*distance/25)**(1/3))
        spatial=np.concatenate([point_mapping.initial(ap) for ap in self.apertures]) if not self.center else np.empty(0)
        return [np.r_[k_from_durations(T*s),spatial] for s in (1.,.8,1.2,1.5)]
    def solve(self,seconds_per_seed=180.,progress=lambda r:None):
        began=time.perf_counter();rows=[];best=None
        for index,seed in enumerate(self.seeds()):
            start=time.perf_counter();deadline=start+seconds_per_seed;invalid=0;expired=False
            def fun(x):
                nonlocal invalid,expired
                if time.perf_counter()>deadline:expired=True;raise RuntimeError('seed_wall_budget')
                try:return self.value_and_gradient(x)
                except (ValueError,RuntimeError,OverflowError):invalid+=1;raise
            result=self.solver.released_lbfgs(seed,fun,memory_size=256,past_iterations=32,min_step=1e-32,
                max_line_search_steps=64,max_iterations=0,function_tolerance=1e-5,gradient_tolerance=0.)
            x=result.pop('x');admitted=result['status']>=0 and not expired and result['cost']<1e29 and np.isfinite(x).all()
            row=dict(seed=index,seed_variables=seed.tolist(),variables=x.tolist(),elapsed_s=time.perf_counter()-start,
                optimizer_converged=bool(admitted),invalid_trials=invalid,wall_budget_expired=expired,**result)
            if admitted:
                value,_=self.value_and_gradient(x);T,P,_=self.geometry(x);T=np.array(T).ravel();P=np.array(P).T
                trajectory=MincoSnap(BoundaryState(self.track.start),BoundaryState(self.track.goal),P,T)
                C=trajectory.coefficients*T[:,None,None]**np.arange(8)[None,:,None]
                if not np.isfinite(C).all():admitted=False;row['optimizer_converged']=False
                else:
                    row['objective']=value;row['nominal_time_s']=float(T.sum())
                    if best is None or value<best[0]:best=(value,T,C,P,x,index)
            rows.append(row);progress(dict(stage='seed_finished',**{k:v for k,v in row.items() if k not in ('seed_variables','variables')}))
        out=dict(found=best is not None,solve_s=time.perf_counter()-began,candidates=rows,selected_seed=best[-1] if best else None,
            all_seven_joint=True,replanning=False,intermediate_stop_points=False,method_family='dynamic_TOGT_x500',
            interior_state_variables=False,continuity_order=6,objective='released x500 TOGT time + native rotor/body-rate penalties',
            optimizer='released C++ Lewis-Overton L-BFGS',safety_filter_added=False,safety_objective_added=False,
            instantaneous_aperture_inset_m=INSTANT_INSET,time_state_geometry_joint=True,includes_all_gate_time_chain_gradients=True,
            selection_rule='lowest original objective among converged four time starts',formal_certificate=False)
        if best:
            _,T,C,P,x,_=best;out.update(durations=T.tolist(),coefficients=C.tolist(),variables=x.tolist(),crossing_points=P.tolist(),
                crossing_times_s=np.cumsum(T)[:7].tolist(),nominal_time_s=float(T.sum()),made_at=0.)
        return out

def main():
    p=argparse.ArgumentParser();p.add_argument('--layout',required=True);p.add_argument('--level',required=True);p.add_argument('--method',choices=('dynamic_togt','dynamic_togt_center'),required=True);p.add_argument('--outdir',type=Path,required=True);a=p.parse_args()
    out=a.outdir.resolve();out.mkdir(parents=True,exist_ok=False);case=Case(a.layout,a.level);track,_=build_case(case)
    save(out/'protocol.json',dict(case=vars(case),method=a.method,scene=manifest(case)))
    began=time.perf_counter();objective=Objective(track,a.method.endswith('_center'));build_s=time.perf_counter()-began
    with (out/'progress.jsonl').open('w') as f:
        def progress(r):f.write(json.dumps(r)+'\n');f.flush();print(json.dumps(r),flush=True)
        result=objective.solve(progress=progress)
    result.update(method=a.method,case=vars(case),build_s=build_s)
    save(out/'result.json',result)
    if result['found']:np.savez_compressed(out/'trajectory.npz',durations=result['durations'],coefficients=result['coefficients'],variables=result['variables'])
    print('TOGT_RESULT',result['found'],result.get('nominal_time_s'),result['solve_s'],flush=True)
if __name__=='__main__':main()
