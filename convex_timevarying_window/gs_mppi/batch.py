"""Small predeclared matrix; preserve every failed condition without retries."""
import argparse
import json
from pathlib import Path
from .experiment import parser,run

def main():
    p=argparse.ArgumentParser();p.add_argument('--outdir',type=Path,required=True);args=p.parse_args()
    args.outdir.mkdir(parents=True,exist_ok=False)
    cases=[]
    for rate in (1,10):
        for offset in (0.,.9):
            for speed in (3,8):
                cases.append((f'single_r{rate}_o{offset}_v{speed}',True,rate,0,offset,speed))
    for rate in (1,10):
        for phase in (0,120):cases.append((f'seven_r{rate}_p{phase}',False,rate,phase,0.,3))
    (args.outdir/'protocol.json').write_text(json.dumps(dict(cases=cases,samples=16384,steps=100,dt=.02,
        seed=0,nominal_only=True,single_limit=15,seven_limit=60,retries=0),indent=2)+'\n')
    results=[]
    for name,single,rate,phase,offset,speed in cases:
        print('CASE '+name,flush=True)
        cli=['--outdir',str(args.outdir/name),'--motion-rate',str(rate),'--phase',str(phase),
             '--offset',str(offset),'--speed',str(speed),'--duration','15' if single else '60']
        if single:cli+=['--single']
        result=run(parser().parse_args(cli));results.append(result)
        (args.outdir/'status.json').write_text(json.dumps(dict(finished=len(results),total=len(cases),
            completed=[r['completed'] for r in results]),indent=2)+'\n')
    (args.outdir/'results.json').write_text(json.dumps(results,indent=2)+'\n')

if __name__=='__main__':main()
