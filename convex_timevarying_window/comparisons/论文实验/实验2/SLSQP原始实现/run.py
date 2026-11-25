from pathlib import Path
import argparse,json,os,subprocess,sys,time

PACKAGE=Path(__file__).resolve().parent
LEVELS=['a25_f6','a50_f6','a75_f6','static','mixed_base','a25_f3','a50_f1','a50_f3','a75_f1','a75_f3']
ORDER=['static','mixed_base','a25_f3','a25_f6','a50_f1','a50_f3','a50_f6','a75_f1','a75_f3','a75_f6']
CPUS=[0,2,4,6,8,10,12,14]

def save(path,data):
    temporary=path.with_suffix(path.suffix+'.writing');temporary.write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n');temporary.replace(path)

def report(out,rows):
    text=['# 原始硬约束SLSQP八初值实验','','冻结源为independent_full_matrix_v2_20261107。原始SLSQP、同问题恢复、硬节点模型与验收设置保持，接入当前高起伏十档场景和原生MINCO包的八个物理初值。',
        '每方法8路同时运行，全部候选结束后按合格最短完整七窗返航时间择优。耗时按实际并行墙钟。','',
        '| 条件 | 方法 | 结果 | 完整参考／s | 八路计算墙钟／s | 合格初值 |',
        '|---|---|---|---:|---:|---:|']
    for level in ORDER:
        for row in rows:
            if row['level']!=level:continue
            duration='—' if row['time_s'] is None else f"{row['time_s']:.3f}"
            name='自由点' if row['method']=='ours' else '中心'
            text.append(f"| {level} | {name} | {row['outcome']} | {duration} | {row['compute_wall_s']:.3f} | {row['qualified']}/8 |")
    text+=['','原始300秒/初值预算、SLSQP归一化ftol1e-5、2000次上限及原恢复流程保持。当前八个初始T和实际VAJ逐值相同，窗内参数按原映射表示中心，物理轨迹等价检查逐候选保存。',
           '原动力学90%推力比例、north tilt/heading pi/2模型、原硬节点和几何容差采用原冻结版本。窗口及地面证据保留原字段。结果为规划及名义几何评估。']
    (out/'REPORT.md').write_text('\n'.join(text)+'\n')

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--cases',type=Path,required=True);parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--levels',nargs='+',default=LEVELS);args=parser.parse_args()
    if args.out.exists():raise RuntimeError('Use a new output directory')
    args.out.mkdir(parents=True);(args.out/'groups').mkdir()
    env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1',NUMBA_CACHE_DIR='/tmp/slsqp_original_numba_cache_20261112')
    save(args.out/'PROTOCOL.json',dict(original_source='independent_full_matrix_v2_20261107',solver_class='independent_free_solve.planner.Planner',
        original_code_changed=False,scene='current higher_swing length2.20',initials='current native MINCO eight frozen physical initial states',
        methods=['ours','center'],starts_per_method=8,workers=8,cpu_pool=CPUS,center_release=False,
        seconds_per_seed=300.,slsqp_ftol=1e-5,max_iterations=2000,original_restoration=True,
        parameters='historical hard dynamics and margins unchanged',levels=args.levels,gazebo_executed=False))
    rows=[];begin=time.perf_counter()
    for level in args.levels:
        for method in ['ours','center']:
            group=args.out/'groups'/level/method;group.mkdir(parents=True)
            start=time.perf_counter();active={};logs=[];finished=[]
            for seed,cpu in enumerate(CPUS):
                destination=group/f'seed_{seed}';log=(group/f'seed_{seed}.log').open('w');logs.append(log)
                command=[sys.executable,'-u',str(PACKAGE/'worker.py'),'--cases',str(args.cases),'--level',level,'--method',method,'--seed',str(seed),'--out',str(destination)]
                process=subprocess.Popen(command,cwd=PACKAGE,env=env,stdout=log,stderr=subprocess.STDOUT,preexec_fn=lambda cpu=cpu:os.sched_setaffinity(0,{cpu}))
                active[seed]=process
            while active:
                for seed,process in list(active.items()):
                    code=process.poll()
                    if code is None:continue
                    path=group/f'seed_{seed}'/'result.json'
                    result=json.loads(path.read_text()) if code==0 and path.exists() else None
                    record=dict(seed=seed,exit_code=code,result_path=str(path),result=result)
                    finished.append(record);del active[seed]
                    save(group/'candidate_summary.json',finished)
                    print('CANDIDATE',level,method,seed,code,result['passed'] if result else 'error',flush=True)
                save(args.out/'status.json',dict(state='running',level=level,method=method,finished_methods=len(rows),
                    completed=len(finished),active=[dict(seed=seed,pid=p.pid) for seed,p in active.items()],pid=os.getpid(),elapsed_s=time.perf_counter()-begin))
                time.sleep(.1)
            for log in logs:log.close()
            good=[r for r in finished if r['result'] and r['result']['passed']]
            best=min(good,key=lambda r:r['result']['nominal_time_s']) if good else None
            if best:save(group/'BEST_RESULT.json',best['result'])
            errors=sum(item['exit_code']!=0 for item in finished)
            row=dict(level=level,method=method,outcome='窗口复查通过' if best else '运行错误' if errors else '未找到',
                time_s=best['result']['nominal_time_s'] if best else None,qualified=len(good),errors=errors,
                best_seed=best['seed'] if best else None,compute_wall_s=time.perf_counter()-start,source=str(group))
            rows.append(row);save(args.out/'SUMMARY.json',rows);report(args.out,rows)
            print('METHOD_COMPLETE',json.dumps(row,ensure_ascii=False),flush=True)
            if errors:raise RuntimeError('Original implementation worker failed; inspect original logs')
    save(args.out/'status.json',dict(state='complete',finished_methods=len(rows),elapsed_s=time.perf_counter()-begin,pid=os.getpid()))

if __name__=='__main__':main()
