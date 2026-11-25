"""Scientific plots from frozen raw audits; unsuccessful tasks get no time."""
import argparse
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--outdir',type=Path,required=True)
    out=parser.parse_args().outdir.resolve()
    rows=json.loads((out/'summary.json').read_text())
    if json.loads((out/'status.json').read_text())['state']!='complete':
        raise RuntimeError('batch not complete')
    fig,axes=plt.subplots(2,2,figsize=(10,7),sharex='col')
    aggregate=dict(scored=len(rows),conditions=len(rows)//2,variants={},paired=[])
    for variant,label,color,marker in [('ours','Free point','#167d9a','o'),('center','Fixed centre','#d67125','s')]:
        selected=[r for r in rows if r['variant']==variant]
        aggregate['variants'][variant]=dict(completed_no_collision=sum(bool(r.get('passed')) for r in selected),
            collided=sum(bool(r.get('collision_detected')) for r in selected),
            incomplete_no_collision=sum(not r.get('passed') and not r.get('collision_detected') for r in selected))
        for j,amplitude in enumerate((2.,4.)):
            data=sorted([r for r in selected if r['amplitude_m']==amplitude],key=lambda r:r['frequency_hz'])
            frequency=np.array([r['frequency_hz'] for r in data])
            axes[0,j].plot(frequency,[r['flight_time_s'] if r.get('passed') else np.nan for r in data],color=color,marker=marker,label=label)
            axes[1,j].scatter(frequency+(-.045 if variant=='ours' else .045),
                [2 if r.get('passed') else 0 if r.get('collision_detected') else 1 for r in data],color=color,marker=marker,s=45)
    for frequency,amplitude in sorted({(r['frequency_hz'],r['amplitude_m']) for r in rows},key=lambda p:(p[1],p[0])):
        pair={r['variant']:r for r in rows if r['frequency_hz']==frequency and r['amplitude_m']==amplitude}
        if all(pair.get(v,{}).get('passed') for v in ('ours','center')):
            a,b=[pair[v]['flight_time_s'] for v in ('ours','center')]
            aggregate['paired'].append(dict(frequency_hz=frequency,amplitude_m=amplitude,free_time_s=a,
                center_time_s=b,saving_s=b-a,saving_percent=100*(b-a)/b))
    for j,amplitude in enumerate((2.,4.)):
        axes[0,j].set_title(f'Amplitude {amplitude:g} m')
        axes[0,j].set_ylabel('Full task time (s), completed only')
        axes[1,j].set_xlabel('Frequency (Hz)')
        axes[1,j].set_yticks([0,1,2],['Collision','Incomplete','Completed'])
        axes[1,j].set_ylim(-.35,2.35)
        for ax in axes[:,j]: ax.grid(alpha=.25)
    axes[0,0].legend()
    fig.tight_layout()
    fig.savefig(out/'paired_results.png',dpi=200)
    fig.savefig(out/'paired_results.pdf')
    plt.close(fig)
    (out/'analysis.json').write_text(json.dumps(aggregate,indent=2)+'\n')
    print(json.dumps(aggregate,indent=2))


if __name__=='__main__':main()
