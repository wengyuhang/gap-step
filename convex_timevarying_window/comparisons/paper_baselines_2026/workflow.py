"""Serial experiment queue: no learning load during timed PX4 comparisons.

Runs both requested papers. If the trained RL policy fails its independent
PX4/static admission test, its dynamic tests are diagnostics, never paper
performance rankings. All phases and logs survive the interactive session.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time


def main():
    p=argparse.ArgumentParser();p.add_argument('--outdir',type=Path,required=True)
    p.add_argument('--resume',type=Path,required=True);p.add_argument('--training-steps',type=int,default=500_000_000)
    a=p.parse_args();a.outdir.mkdir(parents=True,exist_ok=False)
    began=time.time();events=[];state={'pid':os.getpid(),'started_unix':began,'state':'running','events':events,
         'papers':['arXiv:2306.14461','arXiv:2606.27348'],'training_target_steps':a.training_steps,
         'resume':str(a.resume),'resume_sha256':hashlib.sha256(a.resume.read_bytes()).hexdigest(),
         'schedule':['polynomial_16_conditions','rl_full_budget_training','rl_static_px4_check','rl_dynamic_diagnostics_or_matrix'],
         'ours_reference':'convex_timevarying_window/continuation_racing/results/frozen_checked_20260919/summary.json',
         'note':'Our existing frozen 16-condition results are historical, not newly run; preserve the missing-truth case. Learning terminal task is not yet aligned.'}
    def status():
        state['elapsed_s']=time.time()-began
        (a.outdir/'status.json').write_text(json.dumps(state,indent=2)+'\n')
    def execute(name,module,arguments):
        state['active_phase']=name;event={'phase':name,'started_unix':time.time(),'state':'running'};events.append(event);status()
        cmd=[sys.executable,'-u','-m',module,*arguments];event['command']=cmd;print('START',name,flush=True)
        env=os.environ.copy();env.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
        with (a.outdir/f'{name}.log').open('w') as log:
            child=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,env=env)
            state['child_pid']=child.pid;status();code=child.wait()
        event.update(state='finished' if code==0 else 'process_failed',returncode=code,wall_s=time.time()-event['started_unix'])
        status();print('END',name,code,flush=True)
        if code:raise RuntimeError(f'{name} failed with return code {code}; see phase log')
    try:
        runner='convex_timevarying_window.comparisons.paper_baselines_2026.run'
        execute('polynomial_matrix',runner,['--method','polynomial','--stage','matrix','--outdir',str(a.outdir/'polynomial_matrix')])
        training=Path('复现/agile_generalist_rl_2026/runs')/(a.outdir.name+'_full_training')
        state['rl_training_directory']=str(training);status()
        execute('rl_full_training','复现.agile_generalist_rl_2026.train',
            ['--resume',str(a.resume),'--steps',str(a.training_steps),'--outdir',str(training)])
        checkpoint=training/'best_development.pt'
        if not checkpoint.exists():checkpoint=training/'final.pt'
        static=a.outdir/'rl_static_check'
        execute('rl_static_check',runner,['--method','rl','--stage','static','--checkpoint',str(checkpoint),'--outdir',str(static)])
        valid=all(row.get('realtime_success',False) for row in json.loads((static/'summary.json').read_text()))
        state['rl_full_task_px4_validation_passed']=valid
        stage='matrix' if valid else 'development'
        execute('rl_'+stage,runner,['--method','rl','--stage',stage,'--checkpoint',str(checkpoint),'--outdir',str(a.outdir/('rl_'+stage))])
        state['state']='finished' if valid else 'finished_with_rl_transfer_not_validated'
        state['active_phase']=None;status()
    except Exception as exc:
        state['state']='stopped_on_process_error';state['error']=repr(exc);status();raise


if __name__=='__main__':main()
