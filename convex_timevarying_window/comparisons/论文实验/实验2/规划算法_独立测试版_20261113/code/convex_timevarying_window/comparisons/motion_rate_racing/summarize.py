"""Rebuild metrics from saved evidence, optionally compare independent baseline."""
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import contextlib
import io
import re


def summarize(ours,gp=None):
    roots=[Path(ours)]+([Path(gp)] if gp else [])
    rows=[]
    for root in roots:
        data=json.loads((root/'summary.json').read_text())
        known={Path(r['run']).resolve() for r in data}
        for path in sorted(root.iterdir()):
            if not path.is_dir() or not (path/'result.json').exists() or path.resolve() in known:continue
            match=re.fullmatch(r'(ours|gp)_rate([0-9.]+)_phase([0-9]+)_attempt([0-9]+)',path.name)
            if not match:continue
            from convex_timevarying_window.receding_gate_racing.analyze import analyze
            from convex_timevarying_window.receding_gate_racing.localization_audit import audit as truth_audit
            d=json.loads((path/'result.json').read_text())
            with contextlib.redirect_stdout(io.StringIO()):a=analyze(path);b=truth_audit(path)
            geo=b.get('room_clearance_passed',False) and all(g['minimum_sampled_sphere_sleeve_clearance_m']>=0 for g in b.get('groundtruth_sampled_sphere_sleeve_clearance',[]))
            physical=a.get('completed',False) and a.get('contact_free',False) and a.get('ordered_passages_passed',False) and geo
            data.append(dict(method=match[1],rate=float(match[2]),phase_id=int(match[3]),phase=d['phase'],attempt=int(match[4]),
                run=str(path.resolve()),infrastructure_failure=False,physical_success=physical,realtime_success=physical and a['deadline_passed'],
                flight_time_s=d['flight_time_s'],cycle_ms=a.get('cycle_ms'),collision=d['collision_detected'],failure=d['failure'],
                recovered_after_batch_interruption=True))
        for r in data:
            if r.get('infrastructure_failure'):continue
            path=Path(r['run']);result=json.loads((path/'result.json').read_text())
            # Audit ordered is a seven-slot list, which can contain None.
            audit=json.loads((path/'audit.json').read_text()) if (path/'audit.json').exists() else {}
            r['gates']=sum(g is not None for g in audit.get('ordered',[]))
            a=np.genfromtxt(path/'telemetry.csv',delimiter=',',names=True)
            if a.size>1:
                speed=np.sqrt(a['vx']**2+a['vy']**2+a['vz']**2)
                r['mean_speed_mps']=float(np.trapz(speed,a['t'])/(a['t'][-1]-a['t'][0]))
                r['max_speed_mps']=float(speed.max())
                r['path_length_m']=float(np.trapz(speed,a['t']))
            r['accepted_cycles']=sum(s['accepted'] for s in result['solves'])
            r['total_cycles']=len(result['solves']);rows.append(r)
        (root/'summary.json').write_text(json.dumps(data,indent=2)+'\n')
    out=roots[0];(out/'comparison.json').write_text(json.dumps(rows,indent=2)+'\n')
    grouped=[]
    for method in sorted({r['method'] for r in rows}):
        for rate in sorted({r['rate'] for r in rows}):
            rr=[r for r in rows if r['method']==method and r['rate']==rate]
            if not rr:continue
            wins=[r for r in rr if r.get('physical_success')]
            grouped.append(dict(method=method,rate=rate,physical_successes=len(wins),n=len(rr),
                realtime_successes=sum(r.get('realtime_success',False) for r in rr),
                mean_flight_time_s=float(np.mean([r['flight_time_s'] for r in wins])) if wins else None,
                mean_speed_success_mps=float(np.mean([r['mean_speed_mps'] for r in wins])) if wins else None,
                max_cycle_ms=max([r['cycle_ms']['max'] for r in rr if r.get('cycle_ms') and r['cycle_ms']['max'] is not None],default=None),
                max_passed_gates=max(r['gates'] for r in rr),
                collisions=sum(r.get('collision',False) for r in rr)))
    (out/'grouped.json').write_text(json.dumps(grouped,indent=2)+'\n')
    if gp:
        fig,axes=plt.subplots(1,3,figsize=(13,4),layout='constrained')
        for method,label,color in [('ours','Our frozen online planner','tab:blue'),('gp','Independent GP-MPPI / PX4 adaptation','tab:orange')]:
            g=[r for r in grouped if r['method']==method]
            x=[r['rate'] for r in g]
            axes[0].plot(x,[100*r['physical_successes']/r['n'] for r in g],'o-',color=color,label=label)
            axes[1].plot(x,[np.nan if r['mean_flight_time_s'] is None else r['mean_flight_time_s'] for r in g],'o-',color=color)
            axes[2].plot(x,[r['max_passed_gates'] for r in g],'o-',color=color)
            for r in g:axes[0].annotate(f"{r['physical_successes']}/{r['n']}",(r['rate'],100*r['physical_successes']/r['n']),xytext=(0,7 if method=='ours' else -14),textcoords='offset points',ha='center',color=color)
        axes[0].set(ylabel='Physical success (%)',ylim=(-12,112));axes[0].legend(fontsize=8,loc='center right')
        axes[1].set(ylabel='Mean successful flight time (s)',title='Missing points mean no successful finish')
        axes[2].set(ylabel='Maximum ordered gates passed',ylim=(-.3,7.5),yticks=range(8))
        for ax in axes:ax.set_xlabel('Window frequency multiplier');ax.set_xticks([1,3,6,10]);ax.grid(alpha=.2)
        fig.suptitle('INTEGRATION DIAGNOSTICS ONLY — GP-MPPI adaptation is not validated')
        fig.savefig(out/'independent_comparison.png',dpi=180);plt.close(fig)
        fig,axes=plt.subplots(2,2,figsize=(12,7),layout='constrained')
        for ax,rate in zip(axes.flat,[1,3,6,10]):
            for method,color in [('ours','tab:blue'),('gp','tab:orange')]:
                rr=[r for r in rows if r['method']==method and r['rate']==rate and r['phase_id']==0]
                if not rr:continue
                a=np.genfromtxt(Path(rr[0]['run'])/'telemetry.csv',delimiter=',',names=True)
                if a.size<2:continue
                speed=np.sqrt(a['vx']**2+a['vy']**2+a['vz']**2)
                ax.plot(a['t'],speed,color=color,label=f"{method}: {'success' if rr[0].get('physical_success') else 'failed'}")
            ax.set(title=f'{rate}x, fixed phase 0',xlabel='Time (s)',ylabel='Actual speed (m/s)');ax.grid(alpha=.2);ax.legend()
        fig.suptitle('INTEGRATION DIAGNOSTICS — failed traces stop at actual termination')
        fig.savefig(out/'comparison_speed_profiles.png',dpi=180);plt.close(fig)
    print(json.dumps(grouped,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('ours');p.add_argument('--gp');a=p.parse_args();summarize(a.ours,a.gp)
