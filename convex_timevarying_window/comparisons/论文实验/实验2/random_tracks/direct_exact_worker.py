"""Same repaired model and independent seeds, exact sparse NLP Hessian."""
import sys,json
from pathlib import Path
from bootstrap import load
load()
from repair import Planner as Base
import finite_direct_exact
import repair_worker
class Planner(Base):
    def constrained_minimize(self,fun,initial,**kwargs):return finite_direct_exact.minimize(self,fun,initial,**kwargs)
repair_worker.Planner=Planner
if __name__=='__main__':
    repair_worker.main()
    import argparse
    p=argparse.ArgumentParser();p.add_argument('--outdir',type=Path);p.add_argument('--job');a=p.parse_args();path=a.outdir/'plans'/a.job/'result.json';r=json.loads(path.read_text());r['hessian_approximation']='exact';r['finite_state_coordinates']='direct physical VAJ' ;path.write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n')
