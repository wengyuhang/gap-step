"""Independent read-only STL outcome parity for the previous full matrix."""
import argparse,json,subprocess
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
import numpy as np
from convex_timevarying_window.comparisons.论文实验.实验2.higher_swing import scenario
from convex_timevarying_window.comparisons.论文实验.实验2.basic_group.trajectory_only.collision import evaluate
p=argparse.ArgumentParser();p.add_argument('--outdir',type=Path,required=True);a=p.parse_args();a.outdir.mkdir(parents=True,exist_ok=False)
base=scenario.ROOT/'convex_timevarying_window/comparisons/论文实验/实验2'
rows=json.loads((base/'results/higher_swing_10x5_full_v1_20261109/PLANNING.json').read_text())
def check(row):
 raw=json.loads(Path(row['source_file']).read_text());track,_=scenario.build_case(scenario.Case(level=row['level']))
 expected=evaluate(raw['coefficients'],raw['durations'],track,scenario.ASSETS/'meshes')
 folder=a.outdir/(row['level']+'_'+row['variant']);folder.mkdir()
 (folder/'python.json').write_text(json.dumps(expected)+'\n')
 (folder/'trajectory.json').write_text(json.dumps({key:raw[key] for key in ('durations','coefficients')})+'\n')
 subprocess.run(['/tmp/window_cpp_runtime_build/mesh_check',str(folder/'trajectory.json'),str(base/'cpp_runtime/results/assets_v1_20261110'/row['level']/'scene.json'),str(folder/'native.json')],check=True)
 actual=json.loads((folder/'native.json').read_text());differences=[]
 for k,(old,new) in enumerate(zip(expected['checks'],actual['checks'])):
  if old['outcome']!=new['outcome']:differences.append(dict(check=k,python=old['outcome'],cpp=new['outcome']))
 for field in ('ordered_exactly_once','endpoints_passed'):
  if expected[field]!=actual[field]:differences.append(dict(field=field,python=expected[field],cpp=actual[field]))
 result=dict(level=row['level'],method=row['variant'],passed=not differences,differences=differences,
  python_check_s=expected['check_s'],native_check_s=actual['check_s'])
 print('MESH_PARITY',result,flush=True);return result
with ThreadPoolExecutor(max_workers=4) as pool:results=list(pool.map(check,rows))
(a.outdir/'validation.json').write_text(json.dumps(dict(passed=all(r['passed'] for r in results),results=results),indent=2)+'\n')
