"""Independent single-gate geometry: both sides of sphere passage, no racing NLP."""
from pathlib import Path
import json
import numpy as np
from scipy.optimize import brentq,minimize_scalar
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

HERE=Path(__file__).resolve().parents[1]
BODY=.3859447197013662
TUBE=.085
R=1.35+.065
r=BODY+TUBE
vn=13.

def tube_clearance(delta,relative):
 # Exact circle-centerline distance minus Minkowski-inflated body radius.
 grid=np.linspace(-1,1,20001);t=grid*r/vn;q=delta+relative(t);values=np.hypot(abs(q)-R,vn*t)-r
 indices=np.flatnonzero((values[1:-1]<=values[:-2])&(values[1:-1]<=values[2:]))+1
 best=float(min(values[0],values[-1]))
 for k in indices:
  result=minimize_scalar(lambda s:float(np.hypot(abs(delta+relative(s*r/vn))-R,r*s)-r),bounds=(grid[k-1],grid[k+1]),method='bounded',options={'xatol':1e-14})
  best=min(best,float(result.fun))
 return best

def main():
 out=HERE/'results/relative_motion_diagnostic_v1_20261114';out.mkdir(exist_ok=True);assert not (out/'RESULTS.json').exists()
 velocity=[]
 for w in [0.,10.,20.,40.]:
  for delta in [-.6,0.,.6]:
   required=abs(delta)+r*np.sqrt(1+(w/vn)**2);clear=tube_clearance(delta,lambda t:w*t);velocity.append(dict(relative_speed_mps=w,offset_m=delta,required_opening_radius_m=required,analytic_safe=bool(required<=R),minimum_sphere_tube_clearance_m=clear))
 # These profiles are fixed diagnostics, not new formal race conditions.
 profiles=[('moderate',1.4,1.8),('amplitude4_f3',4.,3.),('amplitude4_f3p5',4.,3.5),('amplitude4_f4',4.,4.)];acceleration=[];curves=[]
 for label,A,f in profiles:
  b=2*np.pi*f*r/vn;assert b<np.pi/2
  if A*b*b<=r:s=0.
  else:s=brentq(lambda z:A*b*np.sin(b*z)-r*z/np.sqrt(1-z*z),1e-8,1-1e-12)
  M=A*(1-np.cos(b*s))+r*np.sqrt(1-s*s);offset=float(np.clip((r-M)/2,-(R-r),R-r));worst_free=max(M+offset,r-offset)
  relative=lambda t:A*(1-np.cos(2*np.pi*f*t));cc=tube_clearance(0.,relative);fc=tube_clearance(offset,relative);row=dict(label=label,amplitude_m=A,frequency_hz=f,normal_speed_mps=vn,center_required_radius_m=float(M),best_offset_m=offset,free_required_radius_m=float(worst_free),center_clearance_m=cc,free_clearance_m=fc,center_collision=cc<0,free_collision=fc<0);acceleration.append(row);print(row,flush=True)
  grid=np.linspace(-1,1,20001);t=grid*r/vn;curves.append((label,t,abs(relative(t))+r*np.sqrt(1-grid*grid),abs(offset+relative(t))+r*np.sqrt(1-grid*grid)))
 record=dict(purpose='analytic geometry diagnostic, not full planner feasibility or Gazebo',body_radius_m=BODY,tube_radius_m=TUBE,frame_centerline_radius_m=R,minkowski_radius_m=r,normal_speed_mps=vn,constant_velocity=velocity,sinusoid_at_turning_point=acceleration)
 (out/'RESULTS.json').write_text(json.dumps(record,indent=2)+'\n')
 fig,axes=plt.subplots(1,2,figsize=(11,4.1),constrained_layout=True)
 x=np.linspace(-.65,.65,401)
 for w in [10.,40.]:axes[0].plot(x,abs(x)+r*np.sqrt(1+(w/vn)**2),label=f'Relative speed {w:g} m/s')
 axes[0].axhline(R,color='k',ls='--',lw=1,label='Frame radius');axes[0].set_xlabel('Offset at plane crossing (m)');axes[0].set_ylabel('Required opening radius (m)');axes[0].set_title('Constant relative velocity: center minimizes sweep');axes[0].legend(fontsize=8);axes[0].grid(alpha=.2)
 label,t,c,f=curves[-1];axes[1].plot(1000*t,c,label='Center crossing');axes[1].plot(1000*t,f,label='Free offset');axes[1].axhline(R,color='k',ls='--',lw=1,label='Frame radius');axes[1].set_xlabel('Time relative to center-plane crossing (ms)');axes[1].set_ylabel('Tangential displacement + body section (m)');axes[1].set_title('Accelerating gate: asymmetric sweep can favor offset');axes[1].legend(fontsize=8);axes[1].grid(alpha=.2)
 fig.savefig(out/'relative_motion.png',dpi=180)
if __name__=='__main__':main()
