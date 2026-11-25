"""Reproducible algorithm-independent random-track geometry screening."""
from pathlib import Path
import json, hashlib
from collections import Counter
import numpy as np
from scipy.spatial.transform import Rotation
from ..higher_swing import scenario as base
from ..basic_group.whole_course_difficulty.scenario import horizontal_width, BASE_FREQUENCIES

HERE=Path(__file__).resolve().parent
RADIUS=.3859447197013662
LEVELS={'low':(.25,1.),'medium':(.5,3.),'high':(.75,6.)}

def intersects(a,b,lo,hi):
    low,high=0.,1.
    for k in range(3):
        d=b[k]-a[k]
        if abs(d)<1e-12:
            if a[k]<lo[k] or a[k]>hi[k]:return False
        else:
            s,t=sorted(((lo[k]-a[k])/d,(hi[k]-a[k])/d))
            low=max(low,s);high=min(high,t)
            if low>high:return False
    return True

def make(rng):
    tr,_=base.build_case(base.Case(level='static'))
    p=np.array([w.center0 for w in tr.windows]);route=np.vstack([tr.start,p,tr.goal]);dist=np.linalg.norm(np.diff(route,axis=0),axis=1)
    swing=set(rng.choice(7,3,replace=False).tolist());windows=[]
    for i,w in enumerate(tr.windows):
        offset=rng.normal(size=3);offset/=np.linalg.norm(offset);offset*=rng.random()**(1/3)*.1*min(dist[i],dist[i+1])
        yaw=rng.uniform(-10,10);R=Rotation.from_euler('z',yaw,degrees=True).as_matrix()@w.R
        angle=rng.uniform(0,2*np.pi);direction=R[:,0]*np.cos(angle)+R[:,1]*np.sin(angle)
        # Aperture envelope with 0.15 m conservative frame padding.
        verts=w.aperture.boundary_points(256) if hasattr(w.aperture,'boundary_points') else (np.c_[np.cos(np.linspace(0,2*np.pi,257)),np.sin(np.linspace(0,2*np.pi,257))]*w.aperture.radius if w.aperture.kind=='circle' else w.aperture.physical_vertices)
        extent=(w.aperture.radius*np.linalg.norm(R[:,:2],axis=1) if w.aperture.kind=='circle' else np.max(np.abs(np.asarray(verts)@R[:,:2].T),axis=0))+.15
        windows.append(dict(center=(p[i]+offset).tolist(),R=R.tolist(),kind='swing' if i in swing else 'translation',direction=direction.tolist(),phase=float(rng.uniform(0,2*np.pi)),amplitude_jitter=float(rng.uniform(.85,1.15)),frequency_jitter=float(rng.uniform(.85,1.15)),width=horizontal_width(w),base_frequency=BASE_FREQUENCIES[i],frame_extent=extent.tolist(),shape=w.aperture.kind))
    return dict(start=tr.start.tolist(),goal=tr.goal.tolist(),windows=windows)

def evaluate(layout,level):
    rho,factor=LEVELS[level];boxes=[];windows=[];reasons=[]
    for i,w in enumerate(layout['windows']):
        a=rho*w['width']*w['amplitude_jitter'];f=factor*w['base_frequency']*w['frequency_jitter'];R=np.array(w['R']);c=np.array(w['center']);extent=np.array(w['frame_extent'])
        if w['kind']=='swing':
            if a>=2.2:reasons.append('swing_amplitude_exceeds_length');continue
            rise=2.2-np.sqrt(2.2**2-a*a);d0=-a*np.abs(R[:,1])+np.minimum(0,rise*R[:,0]);d1=a*np.abs(R[:,1])+np.maximum(0,rise*R[:,0])
        else:d1=a*np.abs(w['direction']);d0=-d1
        lo=c+d0-extent;hi=c+d1+extent
        if lo[2]<=0:reasons.append('ground_envelope')
        boxes.append((lo,hi));windows.append(dict(w,amplitude_m=a,frequency_hz=f,swing_length_m=2.2,envelope_lo=lo.tolist(),envelope_hi=hi.tolist()))
    if len(boxes)!=7:return None,reasons
    for i,(lo,hi) in enumerate(boxes):
        for j in range(i):
            if np.all(hi>=boxes[j][0]) and np.all(boxes[j][1]>=lo):reasons.append('overlapping_motion_envelopes')
        for point in [layout['start'],layout['goal']]:
            if np.all(np.array(point)>=lo-RADIUS) and np.all(np.array(point)<=hi+RADIUS):reasons.append('endpoint_in_envelope')
    route=np.array([layout['start']]+[w['center'] for w in windows]+[layout['goal']])
    for s,(a,b) in enumerate(zip(route[:-1],route[1:])):
        for j,(lo,hi) in enumerate(boxes):
            if j in [s-1,s]:continue
            if intersects(a,b,lo-RADIUS,hi+RADIUS):reasons.append('non_target_gate_in_connector')
    return dict(level=level,start=layout['start'],goal=layout['goal'],windows=windows,reference_polyline_length_m=float(np.linalg.norm(np.diff(route,axis=0),axis=1).sum())),reasons

def main():
    out=HERE/'results/v2';out.mkdir(parents=True,exist_ok=False);summary=[]
    for seed in [2026111301,2026111302,2026111303]:
        rng=np.random.default_rng(seed);failures=Counter();accepted=None
        for trial in range(5000):
            layout=make(rng);tracks={};bad=[]
            for level in LEVELS:
                track,issues=evaluate(layout,level);tracks[level]=track;bad.extend(issues)
            if bad:failures.update(set(bad));continue
            accepted=tracks;break
        row=dict(seed=seed,attempts=trial+1,accepted=accepted is not None,rejection_counts=dict(failures))
        if accepted:
            for level,track in accepted.items():
                track.update(seed=seed,accepted_trial=trial,screening='conservative all-phase AABB, not a trajectory feasibility certificate')
                p=out/f'{seed}_{level}.json';p.write_text(json.dumps(track,indent=2)+'\n')
        summary.append(row)
    (out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary,indent=2))
if __name__=='__main__':main()
