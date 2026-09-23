"""Offline empirical rate-response fit and trace-separated validation."""
import argparse
import json
from pathlib import Path
import numpy as np
from scipy.optimize import minimize_scalar


def rate_errors(data,axis,tau):
    errors=[]
    for i in range(0,len(data)-12,4):
        end=np.searchsorted(data[:,0],data[i,0]+.1)
        if end>=len(data):continue
        rate=data[i,11+axis]
        for k in range(i,end):
            cmd=data[k,19+axis]
            rate=cmd+(rate-cmd)*np.exp(-(data[k+1,0]-data[k,0])/tau)
        errors.append(rate-data[end,11+axis])
    return np.asarray(errors)


def load(path):
    data=np.load(path/'execution.npz')['rows']
    _,ids=np.unique(data[:,0],return_index=True)
    return data[ids]


def main():
    p=argparse.ArgumentParser();p.add_argument('--train',type=Path,required=True)
    p.add_argument('--validation',type=Path,required=True);p.add_argument('--outdir',type=Path,required=True)
    a=p.parse_args();a.outdir.mkdir(parents=True,exist_ok=False)
    train=load(a.train);val=load(a.validation);result=[]
    for axis in range(3):
        fit=minimize_scalar(lambda tau:np.mean(rate_errors(train,axis,tau)**2),bounds=(.015,2.),method='bounded')
        step=.05 if axis==2 else .01;tau=np.ceil(fit.x/step)*step
        row=dict(axis=['roll','pitch','yaw'][axis],fitted_tau_s=float(fit.x),rounded_tau_s=float(tau))
        for name,data in [('train',train),('validation',val)]:
            row[name]={}
            for label,value in [('old_50ms',.05),('rounded_fit',tau)]:
                e=rate_errors(data,axis,value)
                row[name][label]=dict(rmse_rad_s=float(np.sqrt(np.mean(e*e))),max_abs_rad_s=float(np.max(np.abs(e))),count=len(e))
        result.append(row)
    report=dict(train=str(a.train.resolve()),validation=str(a.validation.resolve()),axes=result,
                scope='100 ms body-rate replay; effective first-order approximation, not robust certification or full attitude validation')
    (a.outdir/'identification.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
