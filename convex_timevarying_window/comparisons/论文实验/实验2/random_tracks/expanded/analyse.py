"""Post-hoc geometry diagnostics, without modifying any optimizer results."""
import argparse,json
from pathlib import Path
import numpy as np
from bootstrap import load,HERE
scene,*_=load()

def main():
 a=argparse.ArgumentParser();a.add_argument('--outdir',type=Path,required=True);q=a.parse_args();summary=json.loads((q.outdir/'summary.json').read_text());diagnoses=[]
 for pair in summary['pairs']:
  if any(pair['methods'][m]['completed']!=8 or pair['methods'][m]['best'] is None for m in ['ours','center']):continue
  tr,_=scene.build_case(scene.Case(pair['level']));P=np.vstack([tr.start,*[w.center0 for w in tr.windows],tr.goal]);chord=np.diff(P,axis=0);unit=chord/np.linalg.norm(chord,axis=1)[:,None];angles=np.degrees(np.arccos(np.clip(np.sum(unit[:-1]*unit[1:],axis=1),-1.,1.)))
  def polyline(points):return float(np.linalg.norm(np.diff(np.vstack([tr.start,points,tr.goal]),axis=0),axis=1).sum())
  result=dict(level=pair['level'],gate_count=len(tr.windows),reference_length_m=pair['length_m'],mean_turn_deg=float(angles.mean()),nearly_straight_gate_count=int(sum(angles<15)),total_turn_deg=float(angles.sum()),gain_percent=pair['free_time_gain_percent'],methods={})
  for method in ['ours','center']:
   best=pair['methods'][method]['best'];raw=json.loads(Path(best['source']).read_text());points=np.array(raw['crossing_points']);times=np.cumsum(raw['durations'])[:-1];centers=np.array([w.state_at(t)[0] for w,t in zip(tr.windows,times)]);offset=np.array([w.R.T@(p-c) for w,p,c in zip(tr.windows,points,centers)]);norms=np.linalg.norm(offset[:,:2],axis=1);m=len(tr.windows);start=len(raw['variables'])-9*m;V=np.array(raw['variables'][start:start+3*m]).reshape(m,3);relative=np.array([w.R.T@(v-w.state_at(t)[2]) for w,v,t in zip(tr.windows,V,times)])
   if method=='center':assert norms.max()<1e-7
   g=json.loads(Path(best['source']).with_name('assessment.json').read_text());assert len(g['checks'])==m*(m+1) and len(g['opening_events'])==m
   result['methods'][method]=dict(time_s=raw['nominal_time_s'],distance_m=raw['trajectory_distance_m'],dynamic_penalty=raw['dynamic_soft_integral'],mean_offset_m=float(norms.mean()),max_offset_m=float(norms.max()),mean_vertical_offset_m=float(np.mean(abs(offset[:,0]))),mean_horizontal_offset_m=float(np.mean(abs(offset[:,1]))),point_polyline_m=polyline(points),same_times_center_polyline_m=polyline(centers),mean_crossing_normal_speed_mps=float(np.mean(abs(relative[:,2]))),per_gate=[dict(index=i+1,shape=w.shape_name,point_local=off[:2].tolist(),relative_velocity_local=rv.tolist(),window_speed_mps=float(np.linalg.norm(w.state_at(t)[2]))) for i,(w,off,rv,t) in enumerate(zip(tr.windows,offset,relative,times))])
  diagnoses.append(result)
 (q.outdir/'GEOMETRY_DIAGNOSIS.json').write_text(json.dumps(diagnoses,ensure_ascii=False,indent=2)+'\n');print([(r['level'],r['gain_percent'],{k:round(v['mean_offset_m'],3) for k,v in r['methods'].items()},r['mean_turn_deg']) for r in diagnoses])
if __name__=='__main__':main()
