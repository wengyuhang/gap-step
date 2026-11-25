"""Matched free-point/center Wang MINCO PX4 probes; preserve every attempt."""
import argparse
from dataclasses import asdict, replace
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time

import numpy as np

from convex_timevarying_window.comparisons.checked_soft_mpc_racing.run import inspect, source_hashes as parent_hashes
from convex_timevarying_window.comparisons.checked_soft_racing.run import save, ROOT
from .planner import Config as PlanningConfig
from .passage_audit import audit_run

CASES = [(1,1,1,0),(10,1,1,1)]


def source_hashes():
    hashes=parent_hashes()
    for path in (ROOT/'convex_timevarying_window/checked_soft_mpc_isolated').rglob('*.py'):
        if 'results' not in path.parts: hashes[str(path.relative_to(ROOT))]=hashlib.sha256(path.read_bytes()).hexdigest()
    for path in Path(__file__).parent.glob('*.py'):
        hashes[str(path.relative_to(ROOT))]=hashlib.sha256(path.read_bytes()).hexdigest()
    for directory in ('convex_timevarying_window/togt_minco_mpc','复现/polynomial_dynamic_racing_2023'):
        for path in (ROOT/directory).glob('*.py'):
            hashes[str(path.relative_to(ROOT))]=hashlib.sha256(path.read_bytes()).hexdigest()
    for path in (Path(__file__).parent/'fast').glob('*.py'):
        hashes[str(path.relative_to(ROOT))]=hashlib.sha256(path.read_bytes()).hexdigest()
    return hashes


def main():
    p=argparse.ArgumentParser();p.add_argument('--outdir',type=Path,required=True)
    p.add_argument('--center-only',action='store_true');p.add_argument('--condensed',action='store_true',default=True);a=p.parse_args()
    out=a.outdir.resolve();out.mkdir(parents=True,exist_ok=False)
    hashes=source_hashes()
    save(out/'protocol.json',dict(cases=CASES,source_sha256=hashes,created=time.time(),condensed=a.condensed,
        planning_config=asdict(replace(PlanningConfig(),free_points=not a.center_only,
            workspace_lower=(-28.,-30.,.65),workspace_upper=(28.,22.,12.))),trajectory='Wang degree-five time-uniform MINCO-s3; C4 inside each leg, C2 at gates; arrival jerk replaces free midpoint',
        changes='Single-passage event check; body-exit progress on fresh measurement timestamps; committed-crossing guard; coherent future reference; reject stale/regressive replacements; unchanged curve, objective and MPC',
        acceptance='Complete return, contacts/truth clearance and <100 ms checks, PLUS truth exactly-once ordered passage and body exit; 12/8/42 exceedances diagnostic only',
        max_attempts=2,retry='Only independently confirmed preflight infrastructure failure'))
    for relative in hashes:
        dest=out/'source_snapshot'/relative;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(ROOT/relative,dest)
    rows=[]
    for case,(rate,tr,rot,phase_id) in enumerate(CASES):
        for attempt in (1,2):
            if source_hashes()!=hashes:raise RuntimeError('Frozen optimization experiment sources changed')
            path=out/f'r{rate}_t{tr}_a{rot}_p{phase_id}_attempt{attempt}'
            save(out/'status.json',dict(state='running',case=case+1,total=len(CASES),run=str(path),pid=os.getpid()))
            print('START '+path.name,flush=True)
            command=[sys.executable,'-u','-m','convex_timevarying_window.wang_free_state_mpc.px4',
                     '--outdir',str(path),'--motion-rate',str(rate),'--translation-scale',str(tr),
                     '--rotation-scale',str(rot),'--phase',str(phase_id*2*np.pi/3)]
            if a.condensed:command.append('--condensed')
            if a.center_only:command.append('--center-only')
            began=time.perf_counter()
            try:
                p=subprocess.run(command,cwd=ROOT,capture_output=True,text=True,timeout=900)
                console=p.stdout+'\n'+p.stderr;code=p.returncode
            except subprocess.TimeoutExpired as exc:
                console=repr(exc);code=None
            path.mkdir(parents=True,exist_ok=True);(path/'console.log').write_text(console)
            row=dict(method='wang_center_state_mpc' if a.center_only else 'wang_free_state_mpc',condition_id=[rate,tr,rot,phase_id],attempt=attempt,
                     run=str(path),wall_s=time.perf_counter()-began,returncode=code)
            if (path/'result.json').exists():
                row.update(inspect(path))
                if not row['infrastructure_failure']:
                    try: passage = audit_run(path)
                    except Exception as exc: passage = dict(passed=False, violations=['audit_error'], error=repr(exc))
                    save(path/'passage_audit.json', passage)
                    row['legacy_physical_success'] = row['physical_success']
                    row['legacy_realtime_success'] = row['realtime_success']
                    row['single_passage_success'] = passage['passed']
                    row['passage_violations'] = passage['violations']
                    row['physical_success'] &= passage['passed']
                    row['realtime_success'] &= passage['passed']
                    if 'strict_dynamics_success' in row:
                        row['strict_dynamics_success'] &= passage['passed']
            else:row.update(realtime_success=False,physical_success=False,infrastructure_failure=False,failure='missing_result')
            rows.append(row);save(out/'summary.json',rows)
            print('RESULT '+json.dumps(row,ensure_ascii=False),flush=True)
            if not row['infrastructure_failure']:break
    save(out/'status.json',dict(state='complete',conditions=len(CASES),attempts=len(rows)))


if __name__=='__main__':main()
