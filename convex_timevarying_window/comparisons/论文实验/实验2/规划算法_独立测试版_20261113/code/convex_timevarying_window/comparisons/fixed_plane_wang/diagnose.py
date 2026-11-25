"""Read-only explanation of the held-out pilot; never runs an optimizer."""
import argparse
from collections import Counter
from dataclasses import replace
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from .scenario import Case,build_case
from .report import truth_trace,LABELS,COLORS
from convex_timevarying_window.comparisons.checked_soft_racing.run import save
from convex_timevarying_window.wang_free_state_mpc.planner import Planner,Config
from convex_timevarying_window.wang_free_state_mpc.trajectory import Plan
from convex_timevarying_window.continuation_racing.guard import frame_clearances


def plan_for(s):
    d = np.array(s['durations'])
    return Plan(s['made_at'],s.get('candidate_first_gate',s['gate']),d,np.array(s['coefficients']),
                np.array(s['variables']),0.,s['violation'],s['status'],len(d)//2-1)


def replay_checks(planner,plan):
    cfg = planner.cfg
    tt = np.linspace(plan.made_at,plan.end_time,max(2,int(np.ceil((plan.end_time-plan.made_at)/.01))+1))
    p,v,a,j = [plan.sample_many(tt,d) for d in range(4)]
    demand = a+cfg.response_time*j;f = demand+np.array([0.,0.,9.8066]);tests = {}
    for name,values,limit in [('speed',v,cfg.speed),('acceleration',a,cfg.acceleration),
                               ('jerk',j,cfg.jerk),('legacy_lead_acceleration',demand,cfg.acceleration)]:
        norm = np.linalg.norm(values,axis=1);index = int(norm.argmax())
        tests[name] = dict(peak=float(norm[index]),limit=limit,ratio=float(norm[index]/limit),
                           at=float(tt[index]),violated=bool(norm[index]>limit*1.006))
    norms = np.linalg.norm(f,axis=1);limit=4*8.54858/2.0643076923*.9
    tests['thrust'] = dict(peak=float(norms.max()),limit=limit,violated=bool(norms.max()>limit*1.006 or f[:,2].min()<=0))
    tilt_ratio = np.linalg.norm(f[:,:2],axis=1)/f[:,2]/np.tan(np.radians(42))
    tests['lead_tilt'] = dict(ratio=float(tilt_ratio.max()),at=float(tt[tilt_ratio.argmax()]),violated=bool(tilt_ratio.max()>1.006))
    clearance = frame_clearances(planner.frames,p,tt,cfg.radius,cfg.frame_thickness)
    tests['frame_clearance'] = dict(minimum=float(clearance.min()),at=float(tt[clearance.argmin()]),
                                   required=cfg.tracking_margin,violated=bool(clearance.min()<cfg.tracking_margin))
    room = min(float((p-np.array(cfg.workspace_lower)).min()),float((np.array(cfg.workspace_upper)-p).min()))
    tests['workspace'] = dict(minimum=room,violated=room<cfg.tracking_margin)
    hold = min(np.linalg.norm(p[-1]-center)-radius for center,radius in planner.swept_balls())
    tests['terminal_hold'] = dict(minimum=float(hold),violated=bool(hold< -1e-8))
    return tests


def main():
    parser = argparse.ArgumentParser();parser.add_argument('pilot',type=Path);a=parser.parse_args()
    batch=a.pilot.resolve();out=batch/'diagnosis';out.mkdir(exist_ok=True)
    result={};traces={};rejections=[];transitions=[];reference={}
    for method in ('ours','wang'):
        run=batch/f'case00_{method}_attempt1';raw=json.loads((run/'result.json').read_text())
        times,positions,speed=truth_trace(dict(run=str(run)));traces[method]=(times,positions,speed)
        events=json.loads((run/'passage_audit.json').read_text())['aperture_events']
        cuts=[0.]+[e['time'] for e in events]+[raw['flight_time_s']];segments=[]
        telemetry=np.genfromtxt(run/'telemetry.csv',delimiter=',',names=True)
        accepted=[s for s in raw['solves'] if s['accepted']]
        for i,(start,end) in enumerate(zip(cuts[:-1],cuts[1:])):
            tt=np.unique(np.r_[start,times[(times>start)&(times<end)],end])
            p=np.column_stack([np.interp(tt,times,positions[:,j]) for j in range(3)])
            s=np.interp(tt,times,speed);length=float(np.linalg.norm(np.diff(p,axis=0),axis=1).sum())
            segments.append(dict(segment=i,start=start,end=end,duration=end-start,path_m=length,
                average_speed_mps=length/(end-start),minimum_speed_mps=float(s.min()),
                time_below_2mps=float(np.trapz((s<2).astype(float),tt))))
        result[method]=dict(flight_time_s=raw['flight_time_s'],segments=segments,
            accepted=len(accepted),requests=len(raw['solves']),
            tracking_error_rms_m=float(np.sqrt(np.mean(telemetry['error']**2))),
            tracking_error_max_m=float(telemetry['error'].max()),
            planning_max_ms=max(s['cycle_ms'] for s in raw['solves']),
            logged_rejections=dict(Counter(s.get('rejection_reason','unspecified') for s in raw['solves'] if not s['accepted'])))
        if method=='ours':
            case=Case(**raw['scenario']['case']);track,_=build_case(case)
            cfg=replace(Config(),workspace_lower=tuple(raw['config']['workspace_lower']),workspace_upper=tuple(raw['config']['workspace_upper']))
            planner=Planner(track,cfg,build_indices=[])
            plans=[plan_for(s) for s in accepted]
            if raw['pending_discarded']: raise ValueError('Reference replay needs discarded-plan handling')
            rs=np.zeros(len(times));rp=np.tile(track.start,(len(times),1))
            for plan in plans:
                mask=times>=plan.made_at
                rp[mask]=plan.sample_many(times[mask])
                rs[mask]=np.linalg.norm(plan.sample_many(times[mask],1),axis=1)
                rs[times>=plan.end_time]=0.
            reference=dict(times=times.tolist(),positions=rp.tolist(),speed=rs.tolist())
            for previous,current,record in zip(plans[:-1],plans[1:],accepted[1:]):
                old_ids=list(range(previous.first_gate,previous.first_gate+previous.gate_count))
                if previous.first_gate!=current.first_gate and current.first_gate in old_ids:
                    previous_time=previous.crossing_times[old_ids.index(current.first_gate)]
                    transitions.append(dict(received_at=record['time'],new_head_at=current.made_at,
                        target_gate=current.first_gate+1,old_crossing_s=float(previous_time),
                        new_crossing_s=float(current.crossing_times[0]),delay_s=float(current.crossing_times[0]-previous_time),
                        old_nominal_check=replay_checks(planner,previous),new_nominal_check=replay_checks(planner,current)))
            for s in raw['solves']:
                if s.get('rejection_reason')=='geometry_or_dynamics' and s.get('coefficients'):
                    checks=replay_checks(planner,plan_for(s))
                    rejections.append(dict(time=s['time'],gate=s['gate']+1,
                        violations=[k for k,v in checks.items() if v['violated']],checks=checks))
            for i,segment in enumerate(result[method]['segments']):
                mask=(times>=segment['start'])&(times<=segment['end'])
                segment['reference_speed_min']=float(rs[mask].min())
                segment['reference_speed_mean']=float(rs[mask].mean())
                segment['reference_tracking_distance_max']=float(np.linalg.norm(positions[mask]-rp[mask],axis=1).max())
    counts=Counter(name for rejection in rejections for name in rejection['violations'])
    result.update(horizon_transition_acceptances=transitions,replayed_geometry_dynamics_rejections=rejections,
        rejection_constraint_counts=dict(counts),reference_replay=reference,
        evidence='Read-only replay of saved plans and ground-truth flight traces; no optimization, no counterfactual flight')
    save(out/'diagnosis.json',result)
    fig,axes=plt.subplots(1,2,figsize=(12,4.8),layout='constrained')
    for method,(t,p,speed) in traces.items():
        axes[0].plot(t[2:-2],speed[2:-2],color=COLORS[method],label=LABELS[method])
        axes[1].plot(p[:,0],p[:,1],color=COLORS[method],label=LABELS[method])
    axes[0].plot(reference['times'],reference['speed'],'--',lw=1,color='black',alpha=.6,label='Current planned speed')
    for i in (3,5):
        segment=result['ours']['segments'][i]
        axes[0].axvspan(segment['start'],segment['end'],color='gray',alpha=.12)
    for i,c in enumerate(track.windows):axes[1].text(*c.center0[:2],f'W{i+1}',fontsize=8)
    axes[0].set(xlabel='Race time [s]',ylabel='Speed [m/s]',title='Pilot: actual vs planned speed')
    axes[1].set(xlabel='World x [m]',ylabel='World y [m]',title='Pilot: actual body-center paths',aspect='equal')
    for ax in axes:ax.grid(alpha=.2);ax.legend(fontsize=8)
    fig.savefig(out/'speed_and_path.png',dpi=170);plt.close(fig)
    compact={k:{kk:vv for kk,vv in v.items() if kk!='segments'} for k,v in result.items() if k in ('ours','wang')}
    compact['transitions']=[{k:v for k,v in r.items() if 'check' not in k} for r in transitions]
    compact['rejection_constraint_counts']=dict(counts)
    compact['segments']={m:result[m]['segments'] for m in ('ours','wang')}
    print(json.dumps(compact,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
