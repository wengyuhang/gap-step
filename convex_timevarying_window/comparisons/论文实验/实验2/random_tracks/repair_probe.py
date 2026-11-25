from pathlib import Path
import sys,json,time
from bootstrap import load,HERE
scene,*_=load()
from repair import Planner
import numpy as np
out=HERE/'results/repair_initials_v3_20261113';out.mkdir(exist_ok=True)
began=time.perf_counter();rows=[]
for level in ['2026111301_low','2026111301_medium','2026111301_high']:
 tr,_=scene.build_case(scene.Case(level));p=Planner(tr,True,pose_function=scene.pose);seeds=p.seeds();audits=[]
 for i,x in enumerate(seeds):
  g,_,C=p.evaluate(x,p.witnesses());audits.append(dict(p.initial_audits[i],minimum_finite_constraint=float(g.min()),variables=x.tolist(),durations=np.exp(x[:p.n]).tolist()))
 data=dict(level=level,explicit_signs=p.signs,reference_normal_rates=p.geometric_reference_rates.tolist(),seeds=audits)
 # numpy directions are serialized explicitly.
 data['explicit_signs']=list(map(float,p.signs));(out/f'{level}.json').write_text(json.dumps(data,indent=2)+'\n');rows.append(dict(level=level,initials=[(a['seed'],round(sum(a['durations']),3),round(a['restored_dynamic_soft_integral'],6),round(a['minimum_finite_constraint'],6)) for a in audits]));print(rows[-1],flush=True)
print('elapsed',time.perf_counter()-began)
