"""Identify one end-to-end tracking delay without using any gate geometry.

Fits measured position(t) to reference(t-delay). This is an empirical model,
not a bound on residual error or a claim that all dynamics are pure delay.
"""
import argparse
import json
from pathlib import Path
import numpy as np


def identify(run):
    run=Path(run);data=np.genfromtxt(run/'telemetry.csv',delimiter=',',names=True)
    t=data['t'];position=np.c_[data['x'],data['y'],data['z']]
    reference=np.c_[data['rx'],data['ry'],data['rz']]
    delay=np.linspace(0.,.2,81);mask=(t>=t[0]+.2)&(t<=t[-1]-.2);loss=[]
    for tau in delay:
        shifted=np.column_stack([np.interp(t-tau,t,reference[:,axis]) for axis in range(3)])
        loss.append(float(np.mean(np.sum((position[mask]-shifted[mask])**2,axis=1))))
    best=int(np.argmin(loss))
    return dict(source_run=str(run.resolve()),delay_s=float(delay[best]),unshifted_rmse_m=float(np.sqrt(loss[0])),
                fitted_rmse_m=float(np.sqrt(loss[best])),search_delay_s=delay.tolist(),mse=loss,
                interpretation='End-to-end estimator/reference tracking fit. No gate input; not a certified delay or error bound.')


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run');p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    result=identify(a.run);a.output.write_text(json.dumps(result,indent=2)+'\n')
    print({k:v for k,v in result.items() if k not in ('search_delay_s','mse')})
