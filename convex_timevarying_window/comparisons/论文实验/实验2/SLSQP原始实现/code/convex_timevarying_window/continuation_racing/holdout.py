"""Two predeclared layout/phase checks, with the frozen algorithm unchanged."""
import argparse
import contextlib
import hashlib
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import numpy as np
from convex_timevarying_window.receding_gate_racing.analyze import analyze
from convex_timevarying_window.receding_gate_racing.localization_audit import audit


def main():
    ap=argparse.ArgumentParser();ap.add_argument('frozen_root',type=Path);ap.add_argument('outdir',type=Path)
    ap.add_argument('--supplement-missing-evidence',action='store_true');args=ap.parse_args()
    frozen=json.loads((args.frozen_root/'protocol.json').read_text())
    args.outdir.mkdir(parents=True,exist_ok=False)
    conditions=[dict(label='layout_0',purpose='layout_holdout',layout=1.2,phase=.8,rate=10,translation=4,rotation=4),
                dict(label='layout_1',purpose='layout_holdout',layout=-.8,phase=1.1,rate=10,translation=4,rotation=4)]
    if args.supplement_missing_evidence:
        conditions.append(dict(label='evidence_supplement',purpose='missing_groundtruth_supplement',layout=0.,
                               phase=2*np.pi/3,rate=1,translation=1,rotation=1))
    (args.outdir/'protocol.json').write_text(json.dumps(dict(conditions=conditions,
        reference_protocol=str(args.frozen_root/'protocol.json')),indent=2)+'\n')
    subprocess.Popen([sys.executable,'-u','-m','convex_timevarying_window.continuation_racing.observe','--until-pid',str(os.getpid())])
    rows=[]
    for i,case in enumerate(conditions):
        for attempt in (1,2):
            source=Path('convex_timevarying_window/receding_gate_racing')
            assert frozen['source_sha256']=={f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in source.glob('*.py')}
            assert frozen['continuation_source_sha256']==hashlib.sha256(Path('convex_timevarying_window/continuation_racing/planner.py').read_bytes()).hexdigest()
            if frozen.get('checked'):assert frozen['guard_source_sha256']==hashlib.sha256(Path('convex_timevarying_window/continuation_racing/guard.py').read_bytes()).hexdigest()
            path=args.outdir/f"{case['label']}_attempt{attempt}"
            print('START',path,flush=True)
            command=[sys.executable,'-u','-m','convex_timevarying_window.receding_gate_racing.px4_experiment','--launch','--continuation',
                     '--motion-rate',str(case['rate']),'--translation-scale',str(case['translation']),
                     '--rotation-scale',str(case['rotation']),'--layout',str(case['layout']),
                     '--phase',str(case['phase']),'--outdir',str(path)]
            command.extend(['--tracking-preview',str(frozen.get('tracking_preview_s',0.))])
            if frozen.get('checked'):command.append('--checked')
            if frozen.get('anticipatory'):command.append('--anticipatory')
            process=subprocess.run(command,capture_output=True,text=True,timeout=240)
            path.mkdir(exist_ok=True);(path/'console.log').write_text(process.stdout+'\n'+process.stderr)
            r=json.loads((path/'result.json').read_text());infra=r['failure']=="RuntimeError('arm failed')"
            row=dict(**case,run=str(path.resolve()),infrastructure_failure=infra,attempt=attempt)
            if not infra:
                with contextlib.redirect_stdout(io.StringIO()) as log:
                    a=analyze(path);b=audit(path)
                (path/'analysis.log').write_text(log.getvalue())
                room=bool(np.all(np.array(b['world_body_position_min'])>=np.array(r['config']['workspace_lower'])-.01) and
                          np.all(np.array(b['world_body_position_max'])<=np.array(r['config']['workspace_upper'])+.01))
                clearance=min(g['minimum_sampled_sphere_sleeve_clearance_m'] for g in b['groundtruth_sampled_sphere_sleeve_clearance'])
                row.update(physical_success=a['completed'] and a['contact_free'] and a['ordered_passages_passed'] and room and clearance>=0,
                           realtime_passed=a['deadline_passed'],room_passed=room,clearance_m=clearance,
                           flight_time_s=r['flight_time_s'],failure=r['failure'],collision=r['collision_detected'],cycle_ms=a['cycle_ms'])
            rows.append(row);(args.outdir/'summary.json').write_text(json.dumps(rows,indent=2)+'\n')
            print('RESULT',row,flush=True)
            if not infra:break


if __name__=='__main__':main()
