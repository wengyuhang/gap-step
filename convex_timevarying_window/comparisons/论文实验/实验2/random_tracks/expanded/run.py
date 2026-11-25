"""Six frozen courses, free/center each eight cold starts, sixteen concurrent."""
import argparse,json,hashlib,os,subprocess,sys,time
from pathlib import Path
import numpy as np
from bootstrap import load,HERE,PARENT
scene,higher,common,_,_=load()
from repair import Planner,explicit_signs

def save(p,r):p.write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n')

def report(out,rows,case_rows,elapsed,state):
 pairs=[]
 for case in case_rows:
  selected={}
  for method in ['ours','center']:
   rr=[r for r in rows if r['level']==case['level'] and r['method']==method];valid=[r for r in rr if r['qualified']];best=min(valid,key=lambda r:r['objective'],default=None);selected[method]=dict(completed=len(rr),qualified=len(valid),best=best)
  a,b=selected['ours']['best'],selected['center']['best'];pairs.append(dict(**case,methods=selected,free_time_gain_percent=None if not (a and b) else 100*(1-a['time_s']/b['time_s'])))
 save(out/'summary.json',dict(state=state,elapsed_s=elapsed,complete_candidates=len(rows),pairs=pairs))
 lines=['# 增加窗数、种类与长度：自由点/中心配对规划比较','',f'状态：{state}，完成{len(rows)}/96候选，当前墙钟{elapsed:.1f}秒。','', '冻结六条赛道，10/14/18窗各一对水平间距。每种方法8个独立初值，点初值在中心，相同时间/PVAJ生成规则；不使用其他方法优化结果作初值。原目标、安全半径、动力学95%软界、QP初始化恢复、有界有限求解恢复保持。无额外相位开发搜索。','', '时间均包含全部窗口后返航。选原目标最小的合格候选。仅双方合格时比较时间；未找到不是已证明不可行。几何与独立STL/任务验收为名义数值证据，未实飞。','', '|赛道|窗数|参考路程/m|自由点合格|自由点时间/s|中心合格|中心时间/s|时间优势/%|','|---|---|---|---|---|---|---|---|']
 for p in pairs:
  values=[]
  for method in ['ours','center']:
   m=p['methods'][method];best=m['best'];rr=[r for r in rows if r['level']==p['level'] and r['method']==method]
   if best:value=f"{best['time_s']:.3f}"
   elif m['completed']<8:value='待完成'
   elif any(r['outcome']=='collision' for r in rr):value='碰撞'
   elif any(r['outcome']=='runtime_error' for r in rr):value='运行异常'
   elif any(r['outcome']=='task_invalid' for r in rr):value='任务未通过'
   else:value='未找到'
   values.extend([f"{m['qualified']}/{m['completed']}",value])
  gain='—' if p['free_time_gain_percent'] is None else f"{p['free_time_gain_percent']:.2f}"
  lines.append(f"|{p['level']}|{p['n']}|{p['length_m']:.1f}|{'|'.join(values)}|{gain}|")
 lines+=['','每初值、失败原因、求解与独立复查时间见candidates.json；每赛道的初始化与16路联合墙钟见case_timing.json。完整原始记录在plans。', '', '旧七窗的最佳自由点平均偏移0.78–0.84m，平均门距22–23m；时间提升3.8–5.0%。这说明收益已存在，但不能预设增加总长度会增大百分比收益。新数据同时检验转弯切点、门距和窗数的影响。']
 (out/'REPORT.md').write_text('\n'.join(lines)+'\n')

def main():
 cli=argparse.ArgumentParser();cli.add_argument('--outdir',type=Path,required=True);cli.add_argument('--levels',nargs='+',default=list(scene.LEVELS));cli.add_argument('--preflight-only',action='store_true');args=cli.parse_args();out=args.outdir;out.mkdir(parents=True,exist_ok=False)
 for s in ['jobs','logs','initials','source_snapshot']:(out/s).mkdir()
 case_rows=json.loads((HERE/'results/cases_v1/SUMMARY.json').read_text());case_rows=[c for c in case_rows if c['level'] in args.levels];began=time.perf_counter();rows=[];timings=[];hashes={}
 for src in list(HERE.glob('*.py'))+[PARENT/n for n in ['repair.py','finite_recovery.py','finite_direct.py','finite_direct_exact.py']]:
  key=('expanded_' if src.parent==HERE else '')+src.name;hashes[str(src)]=hashlib.sha256(src.read_bytes()).hexdigest();(out/'source_snapshot'/key).write_bytes(src.read_bytes())
 save(out/'protocol.json',dict(case_manifests_sha256=json.loads((HERE/'results/cases_v1/SHA256.json').read_text()),sources_sha256=hashes,methods=['ours','center'],starts=8,concurrency=16,release_center_branch=False,algorithm='repaired original soft dynamics SIP, independent cold starts, bounded exact recovery',finite_limit=5000,original_objective=True,phase_globalization=False,flights=False,track_filtering='geometry only before optimization',different_n_generic_checks=True))
 preflight=[]
 for case in case_rows:
  case_start=time.perf_counter();track,_=scene.build_case(scene.Case(case['level']));p=Planner(track,True,pose_function=scene.pose);seeds=p.seeds();init_s=time.perf_counter()-case_start;center=Planner(track,False,pose_function=scene.pose);audits=[]
  for i,x in enumerate(seeds):
   z=np.r_[x[:p.n],x[-9*p.m:]];a=np.asarray(p.curve_values(x));b=np.asarray(center.curve_values(z));err=float(abs(a-b).max());assert err<1e-7;g,_,_=p.evaluate(x,p.witnesses());offset=np.cumsum(np.exp(x[:p.n]))[:-1]
   for w,t in zip(track.windows,offset):
    pos,R=scene.pose(w,float(t));np.testing.assert_allclose(np.asarray(pos).ravel(),w.state_at(float(t))[0],atol=1e-10)
   audits.append(dict(seed=i,paired_curve_max_error=err,minimum_finite=float(g.min()),dynamic_penalty=p.objective(x)[2]))
  assert len({hashlib.sha256(np.asarray(x).tobytes()).hexdigest() for x in seeds})==8
  initial_file=out/'initials'/f"{case['level']}.json";save(initial_file,dict(explicit_signs=list(map(float,explicit_signs(track)[0])),seeds=[dict(variables=x.tolist()) for x in seeds],generation_s=init_s,audits=audits,restoration=p.initial_audits));preflight.append(dict(level=case['level'],gate_count=p.m,pieces=p.n,free_variables=p.nvar,center_variables=center.nvar,initial_generation_s=init_s,all_initial_finite_feasible=all(a['minimum_finite']>=-1e-5 for a in audits),all_paired_curves_equal=True));save(out/'PREFLIGHT.json',preflight)
  if args.preflight_only:continue
  active=[];solve_began=time.perf_counter()
  for method in ['ours','center']:
   for i in range(8):
    job=dict(id=f"{case['level']}_{method}_{i}",level=case['level'],method=method,seed=i);save(out/'jobs'/f"{job['id']}.json",job);log=(out/'logs'/f"{job['id']}.log").open('w');worker=subprocess.Popen([sys.executable,'-u',str(HERE/'worker.py'),'--outdir',str(out),'--job',job['id'],'--initial',str(initial_file)],stdout=log,stderr=subprocess.STDOUT,env=os.environ.copy());active.append(dict(job,process=worker,log=log,started=time.perf_counter()))
  while active:
   for item in list(active):
    code=item['process'].poll()
    if code is None:continue
    item['log'].close();active.remove(item);source=out/'plans'/item['id']/'result.json';raw=common.read(source) if source.exists() else {};qualified=bool(code==0 and raw.get('qualified'));verdict=raw.get('independent_verdict');outcome='qualified' if qualified else 'runtime_error' if code else 'collision' if verdict=='nominal_collision' else 'task_invalid' if verdict=='nominal_task_invalid' else 'unresolved' if verdict=='nominal_unresolved' else 'not_found';row=dict(job=item['id'],level=case['level'],method=item['method'],seed=item['seed'],qualified=qualified,outcome=outcome,exit_code=code,time_s=raw.get('nominal_time_s'),objective=raw.get('objective'),solve_s=raw.get('solve_s'),build_s=raw.get('build_s'),check_s=raw.get('independent_check_s'),observed_process_wall_s=time.perf_counter()-item['started'],distance_m=raw.get('trajectory_distance_m'),finite_statuses=[c['status'] for c in raw.get('optimizer_calls',[])],source=str(source));rows.append(row);save(out/'candidates.json',rows);print(row,flush=True);report(out,rows,case_rows,time.perf_counter()-began,'running')
   save(out/'status.json',dict(state='running',current_case=case['level'],complete=len(rows),total=16*len(case_rows),elapsed_s=time.perf_counter()-began,active=[dict(job=r['id'],pid=r['process'].pid) for r in active]));time.sleep(2)
  timings.append(dict(level=case['level'],initial_generation_s=init_s,paired_16_process_wall_s=time.perf_counter()-solve_began,case_total_s=time.perf_counter()-case_start));save(out/'case_timing.json',timings)
 state='preflight_complete' if args.preflight_only else 'complete';save(out/'status.json',dict(state=state,complete=len(rows),elapsed_s=time.perf_counter()-began,active=[]));report(out,rows,case_rows,time.perf_counter()-began,state)
if __name__=='__main__':main()
