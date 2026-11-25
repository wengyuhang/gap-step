"""Freeze six larger courses before any optimizer results are inspected."""
from pathlib import Path
import json,hashlib,sys,math
import numpy as np
import trimesh
from bootstrap import load,HERE
scene,*_=load()
from repair import explicit_signs
from convex_timevarying_window.comparisons.论文实验.实验2.safety import frame_aperture

SHAPES=['rectangle','circle','triangle','pentagon','hexagon','heptagon','octagon']
def save(p,r):p.write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n')
def shape(name):
 if name=='circle':return dict(kind='circle',radius=1.35)
 if name=='rectangle':v=np.array([[-1.45,-1.15],[1.45,-1.15],[1.45,1.15],[-1.45,1.15]])
 else:
  n=dict(triangle=3,pentagon=5,hexagon=6,heptagon=7,octagon=8)[name];r=(1.5 if n==5 else 1.45 if n==6 else 1.25/np.cos(np.pi/n));a=np.arange(n)*2*np.pi/n;v=r*np.c_[np.cos(a),np.sin(a)]
 return dict(kind='polygon',vertices=v.tolist())
def intersects(a,b,lo,hi):
 low,high=0.,1.
 for k in range(3):
  d=b[k]-a[k]
  if abs(d)<1e-12:
   if a[k]<lo[k] or a[k]>hi[k]:return False
  else:
   u,v=sorted(((lo[k]-a[k])/d,(hi[k]-a[k])/d));low=max(low,u);high=min(high,v)
   if low>high:return False
 return True

def make(n,rng,scale):
 counts=[n//4+(i<n%4) for i in range(4)];width=(max(counts)-1)*12.;points=[]
 for row,count in enumerate(counts):
  x=np.linspace(0,width,count)
  if row%2:x=x[::-1]
  for j,t in enumerate(x):points.append([t,row*12.,5.5+.5*np.sin((len(points)+1)*.73)])
 points=np.array(points);jitter=rng.uniform(-.65,.65,(n,2));points[:,:2]+=jitter
 # Geometry is generated once then scaled; this function's random state is cloned for each paired spacing.
 points[:,:2]*=scale;start=np.array([-18.*scale,-12.*scale,5.5]);nodes=np.vstack([start,points,start]);chord=np.diff(nodes,axis=0);unit=chord/np.linalg.norm(chord,axis=1)[:,None];directions=unit[:-1]+unit[1:];shapes=(SHAPES*((n+6)//7))[:n];rng.shuffle(shapes);swing=set(rng.choice(n,n//2,replace=False).tolist());windows=[]
 for i,(p,d,name) in enumerate(zip(points,directions,shapes)):
  yaw=rng.uniform(-12,12)*np.pi/180;d=d[:2];d/=np.linalg.norm(d);normal=np.r_[np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]])@d,0.];up=np.array([0.,0.,1.]);R=np.column_stack([up,np.cross(normal,up),normal]);angle=rng.uniform(0,2*np.pi);kind='swing' if i in swing else 'translation';windows.append(dict(index=i,shape=name,aperture=shape(name),center=p.tolist(),R=R.tolist(),motion_kind=kind,direction_local=[float(np.cos(angle)),float(np.sin(angle)),0.],amplitude_m=float(rng.uniform(.9,1.4)),frequency_hz=float(rng.uniform(.9,1.8)),phase=float(rng.uniform(0,2*np.pi))))
 return dict(start=start.tolist(),goal=start.tolist(),windows=windows)

def screen(data):
 boxes=[];issues=[]
 for r in data['windows']:
  w=scene.Window(dict(r,name='screen'));lo,hi=w.motion.local_bounds(0,10.);v=w.aperture.boundary_points(512);extent=np.max(abs(v@w.R[:,:2].T),axis=0)+.15;c=w.center0;motion_center=w.R@((lo+hi)/2);motion_half=abs(w.R)@((hi-lo)/2);a=c+motion_center-motion_half-extent;b=c+motion_center+motion_half+extent
  if a[2]<=.38594472:issues.append('ground_envelope')
  boxes.append((a,b));r['envelope_lo']=a.tolist();r['envelope_hi']=b.tolist()
 for i,(lo,hi) in enumerate(boxes):
  for j in range(i):
   if np.all(hi>=boxes[j][0]) and np.all(boxes[j][1]>=lo):issues.append('overlapping_envelopes')
 nodes=np.array([data['start']]+[r['center'] for r in data['windows']]+[data['goal']])
 for i,(a,b) in enumerate(zip(nodes[:-1],nodes[1:])):
  for j,(lo,hi) in enumerate(boxes):
   if j in [i-1,i]:continue
   if intersects(a,b,lo-.38594472,hi+.38594472):issues.append('non_target_gate_in_connector')
 return sorted(set(issues))

def tube(points,radius=.085,sections=12):
 values=np.c_[points,np.zeros(len(points))];vertices=[];faces=[];angles=np.arange(sections)*2*np.pi/sections
 for a,b in zip(values,np.roll(values,-1,axis=0)):
  t=(b-a)/np.linalg.norm(b-a);u=np.cross(t,[0.,0.,1.]);u/=np.linalg.norm(u);v=np.cross(t,u);ra=a+radius*(np.cos(angles)[:,None]*u+np.sin(angles)[:,None]*v);rb=b+radius*(np.cos(angles)[:,None]*u+np.sin(angles)[:,None]*v);offset=len(vertices);vertices.extend(ra);vertices.extend(rb);vertices.extend([a,b]);ca,cb=offset+2*sections,offset+2*sections+1
  for k in range(sections):
   q=(k+1)%sections;ai,aj=offset+k,offset+q;bi,bj=offset+sections+k,offset+sections+q;faces.extend([(ai,bi,bj),(ai,bj,aj),(ca,aj,ai),(cb,bi,bj)])
 return trimesh.Trimesh(np.array(vertices),np.array(faces),process=False)

def outward(v,offset=.065):
 e=np.roll(v,-1,axis=0)-v;t=e/np.linalg.norm(e,axis=1)[:,None];normal=np.c_[t[:,1],-t[:,0]];out=[]
 for i,p in enumerate(v):
  d=normal[i-1]+normal[i];d/=np.linalg.norm(d);out.append(p+d*offset/max(.5,float(d@normal[i])))
 return np.array(out)

def main():
 out=HERE/'results/cases_v1';out.mkdir(parents=True,exist_ok=False);meshes=scene.ASSETS/'meshes';meshes.mkdir(parents=True,exist_ok=False);rows=[];all_data=[]
 for name in SHAPES:
  ap=scene.aperture(shape(name));points=ap.boundary_points(512);points=points*((ap.radius+.065)/ap.radius) if ap.kind=='circle' else outward(points);tube(points).export(meshes/f'template_{name}.stl')
 for n in [10,14,18]:
  seed=2026111400+n;rejected=[]
  for trial in range(5000):
   data=[]
   for label,scale in [('compact',1.),('long',1.5)]:
    d=make(n,np.random.default_rng(seed+trial*1000),scale);issues=screen(d);data.append((label,scale,d,issues))
   if not any(a[3] for a in data):break
   rejected.append([a[3] for a in data])
  else:raise RuntimeError(f'geometry screening failed n={n}')
  for label,scale,d,issues in data:
   level=f'n{n}_{label}'
   for r in d['windows']:
    r['name']=f'{level}_{r["index"]+1:02d}_{r["shape"]}';(meshes/f'gate_{r["name"]}_sleeve.stl').symlink_to(f'template_{r["shape"]}.stl')
   nodes=np.array([d['start']]+[r['center'] for r in d['windows']]+[d['goal']]);length=float(np.linalg.norm(np.diff(nodes,axis=0),axis=1).sum());d.update(level=level,gate_count=n,seed=seed,accepted_trial=trial,horizontal_scale=scale,reference_polyline_length_m=length,geometry_rule='four alternating-direction straight rows with connecting hairpins; jitter ±0.65m and gate yaw ±12deg; heights vary; start/finish outside left of all gates',geometry_screening=issues,return_to_start=True,shape_names=SHAPES,collision_model='same .3859447197m sphere and .085m sleeve around .065m outward offset, 12 cross sections; circular outline 512 segments',trajectory_feasibility_claim=False)
   save(out/f'{level}.json',d);all_data.append(d);rows.append(dict(level=level,n=n,length_m=length,mean_leg_m=length/(n+1),accepted_trial=trial,translation_gates=sum(r['motion_kind']=='translation' for r in d['windows']),swing_gates=sum(r['motion_kind']=='swing' for r in d['windows']),geometry_rejections=len(rejected)))
 save(out/'SUMMARY.json',rows);save(out/'SHA256.json',{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in out.glob('n*.json')});save(out/'MESH_SHA256.json',{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in meshes.glob('template_*.stl')});print(rows,flush=True)
 import matplotlib;matplotlib.use('Agg');import matplotlib.pyplot as plt
 fig,axs=plt.subplots(2,3,figsize=(14,8),constrained_layout=True)
 for ax,d in zip(axs.flat,sorted(all_data,key=lambda d:(d['horizontal_scale'],d['gate_count']))):
  P=np.array([d['start']]+[r['center'] for r in d['windows']]+[d['goal']]);ax.plot(P[:,0],P[:,1],color='.65',lw=1)
  for r in d['windows']:
   R=np.array(r['R']);p=np.array(r['center']);a=scene.aperture(r['aperture']);b=p+a.boundary_points(100)@R[:,:2].T;ax.plot(b[:,0],b[:,1],color='#2171b5' if r['motion_kind']=='translation' else '#d95f02',lw=2);ax.text(*p[:2],str(r['index']+1),fontsize=8)
  ax.plot(*P[0,:2],'ks');ax.set_aspect('equal');ax.grid(alpha=.2);ax.set_title(f"{d['level']}: {d['reference_polyline_length_m']:.1f}m")
 fig.savefig(out/'tracks.png',dpi=160)
if __name__=='__main__':main()
