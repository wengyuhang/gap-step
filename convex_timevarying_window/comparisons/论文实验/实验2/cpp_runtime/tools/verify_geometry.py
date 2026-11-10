"""Read-only parity against every previously selected reference trajectory."""
import argparse
from pathlib import Path
from types import SimpleNamespace
import json
import subprocess
import time
import numpy as np
from convex_timevarying_window.comparisons.论文实验.实验2.higher_swing import scenario
from convex_timevarying_window.comparisons.论文实验.实验2.safety import passage

parser=argparse.ArgumentParser()
parser.add_argument('--binary',type=Path,required=True)
parser.add_argument('--outdir',type=Path,required=True)
args=parser.parse_args();out=args.outdir.resolve();out.mkdir(parents=True,exist_ok=False)
base=scenario.ROOT/'convex_timevarying_window/comparisons/论文实验/实验2/results/higher_swing_10x5_full_v1_20261109'
rows=json.loads((base/'PLANNING.json').read_text());reports=[]
for row in rows:
    raw=json.loads(Path(row['source_file']).read_text())
    track,_=scenario.build_case(scenario.Case(level=row['level']))
    plan=SimpleNamespace(coefficients=np.array(raw['coefficients']),durations=np.array(raw['durations']),made_at=0.)
    folder=out/(row['level']+'_'+row['variant']);folder.mkdir()
    windows=[]
    for window in track.windows:
        circle=window.aperture.kind=='circle'
        windows.append(dict(center=window.center0.tolist(),rotation=window.R.tolist(),circle=circle,
            swing=window.motion.kind=='swing',frequency=window.motion.frequency,amplitude=window.motion.amplitude,
            length=window.motion.length,phase=window.motion.phase,opening_radius=window.aperture.radius if circle else None,
            vertices=None if circle else window.aperture.physical_vertices.tolist()))
    data=dict(windows=windows,coefficients=plan.coefficients.tolist(),durations=plan.durations.tolist(),radius=.62094472)
    (folder/'input.json').write_text(json.dumps(data)+'\n')
    began=time.perf_counter()
    expected=[dict(gate=gate,piece=piece,**passage(plan,window,.62094472,piece))
        for gate,window in enumerate(track.windows) for piece in range(len(plan.durations))]
    python_s=time.perf_counter()-began
    (folder/'python.json').write_text(json.dumps(expected)+'\n')
    began=time.perf_counter()
    subprocess.run([str(args.binary.resolve()),str(folder/'input.json'),str(folder/'native.json')],check=True)
    native_s=time.perf_counter()-began
    native=json.loads((folder/'native.json').read_text())['checks']
    differences=[]
    for original,actual in zip(expected,native):
        if original['passed']!=actual['passed'] or original['outcome']!=actual['outcome']:
            differences.append(dict(gate=original['gate'],piece=original['piece'],python=original['outcome'],cpp=actual['outcome']))
    report=dict(level=row['level'],method=row['variant'],checks=len(expected),passed=not differences,
        differences=differences,python_s=python_s,native_process_s=native_s)
    reports.append(report)
    (out/'validation.json').write_text(json.dumps(dict(passed=all(r['passed'] for r in reports),results=reports),indent=2)+'\n')
    print('GEOMETRY_PARITY',row['level'],row['variant'],report['passed'],len(expected),flush=True)
