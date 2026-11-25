"""One locked planning-only five-method experiment with immutable candidates."""
from pathlib import Path
from collections import deque
import subprocess,os,sys,time,json,hashlib,fcntl,traceback,shutil
from bootstrap import load,HERE,FROZEN
METHODS=('ours','center','dynamic_togt','dynamic_togt_center','wang2023')
LABELS=dict(zip(METHODS,['本文自由点','本文中心','动态TOGT自由点','动态TOGT中心','Wang']))
OUT=HERE/'results/comparison_v2_20261113'
def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def save(p,obj):
    p.parent.mkdir(parents=True,exist_ok=True);tmp=p.with_suffix(p.suffix+'.new');tmp.write_text(json.dumps(obj,ensure_ascii=False,indent=2)+'\n');tmp.replace(p)
def export(rows,state):
    save(OUT/'summary.json',rows)
    lines=['# 随机七窗五方法规划对比','',f'状态 {state}，已完成 {len(rows)}/45 项。时间覆盖七窗与返航。','', '单元格为完整轨迹时间秒。碰撞、未找到、未完成、证据不足分别标注。仅无窗框碰撞且任务完整的轨迹参与最快值比较。地面另记。','', '|布局|难度|'+'|'.join(LABELS.values())+'|','|---|---|'+'|'.join(['---']*5)+'|']
    lookup={(r['level'],r['method']):r for r in rows}
    for level in scene.LEVELS:
        available=[lookup.get((level,m)) for m in METHODS];valid=[r['nominal_time_s'] for r in available if r and r['outcome']=='clear'];fast=min(valid,default=None);cells=[]
        for row in available:
            if row is None:cell='待运行'
            elif row['outcome']=='clear':
                cell=f"{row['nominal_time_s']:.3f}"
                if row['nominal_time_s']==fast:cell='**'+cell+'**'
            else:cell={'collision':'碰撞','planning_not_found':'未找到','task_invalid':'未完成','unresolved':'证据不足','execution_error':'运行异常'}.get(row['outcome'],row['outcome'])
            cells.append(cell)
        seed,diff=level.split('_');lines.append('|'+seed[-2:]+'|'+diff+'|'+'|'.join(cells)+'|')
    lines+=['','## 计算时间','', '墙钟为该方法首候选启动至最后候选完成，包含进程启动及候选复查。自由点计入中心分支和释放依赖。多任务共享16个工作槽，均保留实际时间戳；不把CPU累计时间当并行墙钟。','', '|赛道|方法|计算墙钟/s|独立几何复查/s|合格候选/候选数|','|---|---|---|---|---|']
    for r in rows:lines.append(f"|{r['level']}|{LABELS[r['method']]}|{r['wall_s']:.3f}|{r.get('geometry_s',0):.3f}|{r['qualified_starts']}/{r['candidate_count']}|")
    (OUT/'REPORT.md').write_text('\n'.join(lines)+'\n')
def publish(level,method,items):
    chosen=min([x for x in items if x.get('passed')],key=lambda x:x.get('objective') if x.get('objective') is not None else 0.,default=None)
    target=OUT/'plans'/f'{level}_{method}';target.mkdir(parents=True,exist_ok=False)
    raw=common.read(Path(chosen['source'])) if chosen else dict(found=False)
    raw.update(method=method,case=dict(layout='random_v2',level=level),candidate_records=items,selected_candidate_job=chosen['job'] if chosen else None)
    save(target/'result.json',raw)
    timing_items=items+([x for x in candidates if x['level']==level and x['method']=='center'] if method=='ours' else [])
    row=dict(level=level,method=method,outcome='planning_not_found',nominal_time_s=raw.get('nominal_time_s'),qualified_starts=sum(bool(x.get('passed')) for x in items),candidate_count=len(items),source=str(target/'result.json'),wall_s=max(x['finished_s'] for x in timing_items)-min(x['started_s'] for x in timing_items),geometry_s=0.)
    if chosen:
        track,_=scene.build_case(scene.Case(level));began=time.perf_counter()
        from convex_timevarying_window.comparisons.论文实验.实验2.basic_group.trajectory_only.collision import evaluate
        geometry=evaluate(raw['coefficients'],raw['durations'],track,scene.ASSETS/'meshes');save(target/'assessment.json',geometry)
        collision=any(c['outcome']=='collision' for c in geometry['checks']);clear=all(c['outcome']=='clear' for c in geometry['checks']);task=geometry['ordered_exactly_once'] and geometry['endpoints_passed']
        row.update(outcome='collision' if collision else 'clear' if clear and task else 'task_invalid' if not task else 'unresolved',geometry_s=time.perf_counter()-began,ground=geometry['ground'],trajectory_distance_m=geometry['trajectory_distance_m'])
    if not chosen and any(x.get('process_error') for x in items):row['outcome']='execution_error'
    save(target/'record.json',row);print('RESULT',level,method,row['outcome'],row.get('nominal_time_s'),flush=True);return row

def main():
    global scene,common,candidates
    scene,higher,common,best,rollout=load()
    assert common.read(HERE/'results/v2/PREFLIGHT.json')['passed']
    OUT.mkdir(exist_ok=False);(OUT/'logs').mkdir()
    frozen_expected=common.read(FROZEN/'convex_timevarying_window/comparisons/论文实验/实验2/results/higher_swing_10x5_full_v1_20261109/protocol.json')['source_sha256']
    assert all(digest(FROZEN/p)==sha for p,sha in frozen_expected.items())
    sources={str(p):digest(p) for p in HERE.glob('*.py')};tracks=common.read(HERE/'results/v2/FROZEN_SHA256.json')
    save(OUT/'protocol.json',dict(methods=METHODS,levels=scene.LEVELS,concurrency=16,starts=8,maximum_candidates=306,planning_only=True,frozen_source_root=str(FROZEN),frozen_source_sha256=frozen_expected,adapter_sha256=sources,track_sha256=tracks,selection='original objective among admissible candidates; baselines not collision-filtered',geometry='original full-trajectory floating-point Bernstein/analytic-motion plus STL distance',source_language='frozen Python/CasADi/IPOPT and native C++ kernels',finite_max_iterations=5000,exchange_rounds=24,modified_algorithm_parameters=False))
    candidates=[];rows=[];queue=deque();active=[];released=set();published=set();began=time.perf_counter()
    for level in scene.LEVELS:
        folder=OUT/'candidates'/level
        for method in METHODS:
            for seed in range(1 if method=='wang2023' else 8):
                job=dict(id=f'{method}_{seed}',method=method,seed=seed,level=level);save(folder/'jobs'/f"{job['id']}.json",job);queue.append(job)
    export(rows,'running')
    while queue or active or len(rows)<45:
        for job in list(active):
            code=job['process'].poll()
            if code is None:continue
            job['log'].close();active.remove(job);finished=time.perf_counter()-began
            result=OUT/'candidates'/job['level']/'plans'/job['id']/'candidate.json'
            if code==0 and result.exists():r=common.read(result)
            else:r=dict(job=job['id'],method=job['method'],seed=job['seed'],level=job['level'],passed=False,found=False,process_error=code,log=str(OUT/'logs'/f"{job['level']}_{job['id']}.log"))
            r.update(started_s=job['started_s'],finished_s=finished,observed_process_wall_s=finished-job['started_s']);candidates.append(r);save(OUT/'candidate_summary.json',candidates)
            print('CANDIDATE',len(candidates),job['level'],job['id'],r.get('passed'),code,flush=True)
        for level in scene.LEVELS:
            current=[r for r in candidates if r['level']==level]
            if level not in released and sum(r['method'] in ('ours','center') for r in current)==16:
                released.add(level);eligible=[r for r in current if r['method']=='center' and r.get('passed')]
                if eligible:
                    chosen=min(eligible,key=lambda r:r['objective']);shutil.copyfile(chosen['source'],OUT/'candidates'/level/'best_center_result.json');job=dict(id='release_0',method='release',seed=0,level=level);save(OUT/'candidates'/level/'jobs/release_0.json',job);queue.appendleft(job)
            for method in METHODS:
                if (level,method) in published:continue
                eligible=[r for r in current if r['method']==method or method=='ours' and r['method']=='release'];needed=1 if method=='wang2023' else 8
                if method=='ours':
                    if level not in released:continue
                    has_center=any(r['method']=='center' and r.get('passed') for r in current)
                    needed+=int(has_center)
                if len(eligible)!=needed:continue
                rows.append(publish(level,method,eligible));published.add((level,method));export(rows,'running')
        while queue and len(active)<16:
            job=queue.popleft();log=(OUT/'logs'/f"{job['level']}_{job['id']}.log").open('w');process=subprocess.Popen([sys.executable,'-u',str(HERE/'worker.py'),'--outdir',str(OUT/'candidates'/job['level']),'--job',job['id']],cwd=FROZEN,stdout=log,stderr=subprocess.STDOUT,env=os.environ.copy());active.append(dict(job,process=process,log=log,started_s=time.perf_counter()-began))
        save(OUT/'status.json',dict(state='running',completed_candidates=len(candidates),completed_method_conditions=len(rows),queued=len(queue),active=[dict(pid=x['process'].pid,level=x['level'],job=x['id']) for x in active],elapsed_s=time.perf_counter()-began))
        time.sleep(1.)
    assert all(digest(Path(p))==sha for p,sha in sources.items())
    assert all(digest(HERE/'results/v2'/p)==sha for p,sha in tracks.items())
    save(OUT/'final_validation.json',dict(passed=True,method_conditions=len(rows),candidates=len(candidates),sources_preserved=True,tracks_preserved=True,process_errors=sum('process_error' in c for c in candidates)))
    export(rows,'complete');save(OUT/'status.json',dict(state='complete',completed_candidates=len(candidates),completed_method_conditions=len(rows),elapsed_s=time.perf_counter()-began))
if __name__=='__main__':
    with open('/tmp/random_tracks_comparison_v1.lock','w') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        try:main()
        except Exception:
            if OUT.exists():save(OUT/'coordinator_error.json',dict(traceback=traceback.format_exc()))
            raise
