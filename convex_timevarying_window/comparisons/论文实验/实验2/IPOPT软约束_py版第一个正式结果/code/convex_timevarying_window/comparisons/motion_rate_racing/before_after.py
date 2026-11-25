"""Compare old/new crossing constraints, never masquerading as paper comparison."""
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    p=argparse.ArgumentParser();p.add_argument('old',type=Path);p.add_argument('new',type=Path);a=p.parse_args()
    fig,axes=plt.subplots(1,2,figsize=(10,4),layout='constrained')
    for root,label,color in [(a.old,'Old worst-case speed constraint','tab:gray'),(a.new,'Time-local crossing constraint','tab:blue')]:
        rows=json.loads((root/'summary.json').read_text());rates=[1,3,6,10];times=[];counts=[]
        for rate in rates:
            rr=[r for r in rows if r['rate']==rate and not r.get('infrastructure_failure')]
            win=[r for r in rr if r['physical_success']];counts.append(len(win))
            times.append(np.mean([r['flight_time_s'] for r in win]) if win else np.nan)
        axes[0].plot(rates,counts,'o-',label=label,color=color)
        axes[1].plot(rates,times,'o-',label=label,color=color)
    axes[0].set(ylabel='Successful runs out of 3',ylim=(-.15,3.45),yticks=[0,1,2,3]);axes[0].legend(fontsize=8)
    axes[1].set(ylabel='Mean successful finish time (s)',title='Missing point = no successful finish')
    for ax in axes:ax.set(xlabel='Window frequency multiplier',xticks=[1,3,6,10]);ax.grid(alpha=.2)
    fig.suptitle('Same track / amplitudes / x500 / margins: crossing-constraint change')
    fig.savefig(a.new/'before_after.png',dpi=180)


if __name__=='__main__':main()
