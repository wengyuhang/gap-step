"""Read-only attribution of saved failures; no retuning or rerun selection."""
import argparse
from collections import Counter
import json
from pathlib import Path
import numpy as np


def main(root):
    root=Path(root);rows=json.loads((root/'summary.json').read_text());out=[]
    for row in rows:
        path=Path(row['run'])
        if not (path/'result.json').exists():continue
        r=json.loads((path/'result.json').read_text());solves=r['solves']
        first=next((s['time'] for s in solves if s['accepted']),None)
        rejected=[s for s in solves if not s['accepted']]
        reasons=[]
        if not r['completed']:reasons.append('not_completed')
        if row.get('gates',0)<7:reasons.append('ordered_crossings')
        if row.get('collision'):reasons.append('gazebo_gate_contact')
        if row.get('min_clearance_m',0)<0:reasons.append('sampled_sphere_clearance')
        if not row.get('room_passed',False):reasons.append('configured_workspace_envelope')
        item=dict(run=path.name,success=row.get('physical_success',False),failure=row.get('failure'),
            audit_failure_reasons=reasons,
            ordered_gates=row.get('gates'),first_accepted_plan_time_s=first,
            accepted=len(solves)-len(rejected),total=len(solves),
            solver_status_counts=dict(Counter(s['status'] for s in solves)),
            rejected_constraint_violation=sum(s['violation'] is not None and s['violation']>=.003 for s in rejected),
            rejected_short_prefix=sum(s.get('prefix_clearance') is not None and s['prefix_clearance']<r['config']['tracking_margin'] for s in rejected),
            rejected_deadline=sum(s['cycle_ms']>=100 for s in rejected),
            rejected_without_candidate=sum(s['violation'] is None for s in rejected),
            note='Rejection causes can overlap; missing timing/state detail is not inferred.')
        contacts=[c for c in r['contacts'] if c['contact_pair_count']]
        if contacts:
            stamp=min(c['first_contact_sim_time_s'] for c in contacts)-30.
            trace=np.genfromtxt(path/'telemetry.csv',delimiter=',',names=True)
            near=int(np.argmin(abs(trace['t']-stamp)));before=trace[trace['t']<stamp]
            item['first_contact_race_time_s']=stamp
            item['nearest_contact_sample_time_s']=float(trace['t'][near])
            item['contact_after_flight_end_s']=stamp-r['last_time_s']
            item['tracking_error_at_nearest_contact_sample_m']=float(trace['error'][near])
            item['tracking_margin_m']=r['config']['tracking_margin']
            item['pre_contact_max_speed_mps']=float(np.sqrt(before['vx']**2+before['vy']**2+before['vz']**2).max()) if before.size else None
            item['contact_topics']=[c['raw_log'] for c in contacts]
        out.append(item)
    (root/'diagnostics.json').write_text(json.dumps(out,indent=2)+'\n')
    for item in out:print(item['run'],'success',item['success'],'gates',item['ordered_gates'],'accepted',item['accepted'],'/',item['total'],'failure',item['failure'],item['audit_failure_reasons'])


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('root');main(p.parse_args().root)
