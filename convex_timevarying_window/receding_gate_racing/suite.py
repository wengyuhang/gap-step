"""Sequential frozen-parameter PX4 repetitions and held-out course variants."""
import argparse
import json
from pathlib import Path
import subprocess
import sys

from .analyze import analyze
from .localization_audit import audit as localization_audit


def main():
    parser=argparse.ArgumentParser(); parser.add_argument('--prefix',required=True)
    parser.add_argument('--race',action='store_true')
    parser.add_argument('--nested',action='store_true')
    parser.add_argument('--cases',nargs='+',choices=['repeat','phase','layout','center'],default=['repeat','phase','layout','center'])
    args=parser.parse_args()
    root=Path(__file__).parent/'results'
    cases=[('repeat',[]),('phase',['--phase','0.8']),('layout',['--layout','1.2']),('center',['--center'])]
    cases=[item for item in cases if item[0] in args.cases]
    rows=[]
    for name,extra in cases:
        for attempt in range(2):
            outdir=root/f'{args.prefix}_{name}_{attempt+1}'
            command=[sys.executable,'-u','-m','convex_timevarying_window.receding_gate_racing.px4_experiment',
                     '--launch','--outdir',str(outdir),*extra]
            if args.race: command.append('--race')
            if args.nested: command.append('--nested')
            print('START',name,'attempt',attempt+1,flush=True)
            result=subprocess.run(command,text=True,capture_output=True)
            if outdir.exists(): (outdir/'console.log').write_text(result.stdout+'\n'+result.stderr)
            if not (outdir/'result.json').exists():
                print('adapter error',result.stdout,result.stderr,flush=True)
                rows.append({'case':name,'attempt':attempt+1,'run':str(outdir),'adapter_error':True})
                break
            data=json.loads((outdir/'result.json').read_text())
            # Missing simulated sensors is an infrastructure failure, retained
            # separately. Only this pre-flight condition gets an automatic retry.
            if data['failure']=="RuntimeError('arm failed')":
                rows.append({'case':name,'attempt':attempt+1,'run':str(outdir),'infrastructure_failure':'arm failed'})
                print('arm failure retained',flush=True)
                continue
            audit=analyze(outdir)
            true_state=localization_audit(outdir)
            physical_pass=(true_state.get('room_clearance_passed',False) and
                           all(g['minimum_sampled_sphere_sleeve_clearance_m']>=0
                               for g in true_state.get('groundtruth_sampled_sphere_sleeve_clearance',[])))
            row={'case':name,'attempt':attempt+1,'run':str(outdir),
                 'passed':audit.get('passed',False) and physical_pass,'flight_time_s':data['flight_time_s'],
                 'failure':data['failure'],'collision':data['collision_detected'],
                 'cycle_ms':audit.get('cycle_ms'),'planner_sha256':data['source_sha256']['planner.py'],
                 'groundtruth_geometry_passed':physical_pass}
            rows.append(row); print('RESULT',json.dumps(row),flush=True)
            break
        (root/f'{args.prefix}_suite.json').write_text(json.dumps(rows,indent=2)+'\n')
    print('SUITE',json.dumps(rows,indent=2),flush=True)


if __name__=='__main__': main()
