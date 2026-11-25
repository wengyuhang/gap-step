"""Local asymmetric-gate test with unchanged sphere, frame and planning margins."""
from pathlib import Path
import sys,json,time
import numpy as np,trimesh
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from bootstrap import load,HERE
scene,*_=load()
from convex_timevarying_window.comparisons.论文实验.实验2.safety import frame_aperture,passage
from convex_timevarying_window.single_gate_stop_racing.geometry import halfplanes
from convex_timevarying_window.comparisons.论文实验.实验2.basic_group.trajectory_only.collision import MeshDistance,CheckConfig,_piece
from convex_timevarying_window.global_gate_minco_probe.planner import Plan
from types import SimpleNamespace

out=HERE/'results/triangle_motion_diagnostic_v1_20261114';out.mkdir(exist_ok=False);R=np.array([[0,0,1],[0,-1,0],[1,0,0]],float);native=json.loads((HERE/'results/cases_v1/n10_compact.json').read_text());aprow=next(r['aperture'] for r in native['windows'] if r['shape']=='triangle');w=scene.Window(dict(index=0,name='local_triangle',shape='triangle',aperture=aprow,center=[0,0,5.5],R=R.tolist(),frequency_hz=1.,amplitude_m=0.,phase=0.,motion_kind='translation',direction_local=[1,0,0]));mesh=MeshDistance(trimesh.load_mesh(scene.ASSETS/'meshes/template_triangle.stl'));normals,bounds=halfplanes(frame_aperture(w.aperture));radius=.62594472;normal_speed=13.;T=.05;rows=[]
class Linear:
 def __init__(self,speed):self.speed=speed
 def local(self,t):return np.array([self.speed*(t-.05),0.,0.]),np.array([self.speed,0.,0.])
 def local_bounds(self,a,b):return np.array([self.speed*(a-.05),0.,0.]),np.array([self.speed*(b-.05),0.,0.])
 def translation(self,t):p,v=self.local(t);return R@p,R@v
for ratio in [2.4,2.5,2.6,2.7,2.8]:
 speed=normal_speed*ratio;w.motion=Linear(speed);required=radius*np.sqrt(1+(normals[:,0]*ratio)**2);rhs=bounds-required;a=normals[:,0];low=max(rhs[a<0]/a[a<0],default=-np.inf);high=min(rhs[a>0]/a[a>0],default=np.inf);feasible=bool(low<=high and np.all(rhs[abs(a)<1e-12]>=0));delta=float((low+high)/2) if feasible else None;record=dict(relative_speed_ratio=ratio,gate_speed_mps=speed,normal_speed_mps=normal_speed,current_finite_radius_m=radius,feasible_offset_interval_m=[float(low),float(high)],linear_inside_constraints_feasible=feasible,actions=[])
 for method,offset in [('center',0.)]+([('free',delta)] if feasible else []):
  C=np.zeros((2,8,3));C[0,0]=w.center0+R@np.array([offset,0.,-normal_speed*T]);C[0,1]=R@np.array([0.,0.,normal_speed*T]);C[1,0]=w.center0+R@np.array([offset,0.,0.]);C[1,1]=C[0,1];plan=Plan(0.,0,np.array([T,T]),C,np.array([]),0.,0.,'local-asymmetric-fixed-state',1);checks=[passage(plan,w,radius,piece) for piece in [0,1]];independent=[_piece(C[p],T,p*T,w,mesh,CheckConfig(),time.perf_counter()+120) for p in [0,1]];record['actions'].append(dict(method=method,offset_m=offset,original_planning_margin_passed=all(r['passed'] for r in checks),planning_checks=checks,stl_outcomes=[r['outcome'] for r in independent],stl_minimum_sample_clearance_m=min([r['minimum_sample_clearance_m'] for r in independent if r.get('minimum_sample_clearance_m') is not None],default=None)))
 rows.append(record);print(ratio,feasible,[(a['method'],a['offset_m'],a['original_planning_margin_passed'],a['stl_outcomes']) for a in record['actions']],flush=True)
(out/'RESULTS.json').write_text(json.dumps(dict(scope='local actions with fixed passing time and velocity, exact same triangular opening and current planning radius; not whole center NLP infeasibility or Gazebo',rows=rows),indent=2)+'\n')
