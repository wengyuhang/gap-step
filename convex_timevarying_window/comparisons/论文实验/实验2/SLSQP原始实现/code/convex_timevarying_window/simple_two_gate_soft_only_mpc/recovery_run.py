"""Only free points: repeat the seven frozen v3 conditions once each."""
import argparse
import json
import shutil
from pathlib import Path
from .run import sources, execute, evaluate, save, ROOT, Trial


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--outdir', type=Path, required=True)
    p.add_argument('--resume', action='store_true')
    a = p.parse_args(); out = a.outdir.resolve(); out.mkdir(parents=True, exist_ok=a.resume)
    cases = [(1.,2.), (1.5,2.5), (2.,3.), (1.5,2.), (2.,2.), (1.,2.5), (1.,3.)]
    hashes = sources()
    protocol = dict(cases=cases, method='ours', phase=0., repeats=1,
         source_sha256=hashes, change='tracking-only MPC: state/input reference tracking and input smoothing; physical input box only. Existing >0.35m actual-state recovery unchanged; initial J not constrained by MINCO.',
         primary_metrics=['complete start-seven-gates-return time','physical frame contacts'])
    if a.resume:
        original=json.loads((out/'protocol.json').read_text())['source_sha256']
        changed=[k for k in original if original[k]!=hashes[k]]
        assert set(changed)<={'convex_timevarying_window/simple_two_gate_soft_only_mpc/recovery_run.py'}
        save(out/'reporting_revision.json',dict(changed=changed,source_sha256=hashes,
             reason='Preserve incomplete flights with missing truth tail as incomplete/evidence-limited; continue remaining conditions without repeats'))
        rows=json.loads((out/'summary.json').read_text())
    else:
        save(out/'protocol.json',protocol)
        for rel in hashes:
            dst = out/'source_snapshot'/rel; dst.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(ROOT/rel, dst)
        rows=[]
    for f, amplitude in cases:
        if any(r['trial']['frequency']==f and r['trial']['amplitude']==amplitude for r in rows):continue
        assert sources() == hashes, 'Frozen source changed'
        trial=Trial('recovery','ours',f,amplitude,0.,0)
        path=out/trial.name
        save(out/'status.json', dict(state='running', case=[f,amplitude], completed=len(rows)))
        print('START', trial.name, flush=True)
        row=evaluate(execute(trial,path,out/(trial.name+'.log'),100))
        rows.append(row);save(out/'summary.json',rows)
        print('RESULT', trial.name, row.get('simulation_outcome'), row.get('simulation_race_time_s'), flush=True)
        if row.get('infrastructure_failure') or (row.get('audit_error') and row.get('completed')):
            save(out/'status.json',dict(state='evidence_or_infrastructure_failure',directory=str(path)))
            raise RuntimeError(f'Unresolved evidence: {path}')
    save(out/'status.json',dict(state='complete',trials=len(rows)))


if __name__ == '__main__': main()
