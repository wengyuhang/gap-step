"""Frozen direct-CTBR matrix, sequential GPU use and no failure replacement."""
import argparse
import json
import math
from pathlib import Path
from .experiment import parser,run

def main():
    p=argparse.ArgumentParser();p.add_argument('--outdir',type=Path,required=True);a=p.parse_args();a.outdir.mkdir(parents=True,exist_ok=False)
    cases=[]
    for rate in (1,10):
        for offset in (0.,.9):
            for speed in (3,8):cases.append((f'single_r{rate}_o{offset}_v{speed}',True,rate,0,offset,speed))
    for rate in (1,10):
        for phase in (0,120):cases.append((f'seven_r{rate}_p{phase}',False,rate,phase,0,3))
    (a.outdir/'protocol.json').write_text(json.dumps(dict(cases=cases,case_phase_units='degrees; converted to radians for scenario',model='17-state CTBR',samples=16384,steps=100,seed=0,retries=0,blas_threads=1),indent=2)+'\n')
    results=[]
    for name,single,rate,phase,offset,speed in cases:
        print('CASE '+name,flush=True)
        cli=['--outdir',str(a.outdir/name),'--motion-rate',str(rate),'--phase',str(math.radians(phase)),'--offset',str(offset),'--speed',str(speed),'--duration','15' if single else '60']
        if single:cli+=['--single']
        result=run(parser().parse_args(cli));results.append(result)
        (a.outdir/'status.json').write_text(json.dumps(dict(finished=len(results),total=len(cases),completed=[r['completed'] for r in results]),indent=2)+'\n')
    (a.outdir/'results.json').write_text(json.dumps(results,indent=2)+'\n')

if __name__=='__main__':main()
