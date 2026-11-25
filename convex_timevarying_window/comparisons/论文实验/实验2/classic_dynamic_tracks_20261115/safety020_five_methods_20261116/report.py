from pathlib import Path
import json
p=Path(__file__).resolve().parent
ours=json.loads((p/'results/ours/SUMMARY.json').read_text())['conditions']
baselines=json.loads((p/'results/baselines/BASELINE_SUMMARY.json').read_text())
levels=json.loads((p/'config/protocol.json').read_text())['levels']
rows=[]
for level in levels:
 pair=next((x for x in ours if x['level']==level),None)
 for method in ['ours','center']:
  if pair is None:rows.append(dict(level=level,method=method,outcome='待完成',time_s=None,compute_s=None));continue
  x=pair['methods'][method]
  rows.append(dict(level=level,method=method,outcome='通过' if x['found'] else '未找到',time_s=x.get('nominal_time_s'),compute_s=x['elapsed_to_final_selection_s'],successful_starts=x['successful_candidates']))
 for method in ['dynamic_togt','dynamic_togt_center','wang2023']:
  x=next(v for v in baselines if v['level']==level and v['method']==method)
  rows.append(dict(level=level,method=method,outcome=x['outcome'],time_s=x['trajectory_time_s'],compute_s=x['compute_wall_s']))
(p/'PLANNING.json').write_text(json.dumps(rows,ensure_ascii=False,indent=2)+'\n')
text='# 三赛道四档五方法规划比较\n\n2026-11-16。12条件、60项，只规划。新增freq3/freq4保持高档幅度，各窗频率乘1.5和2。实际频率分别2.55～3.45 Hz、3.40～4.60 Hz。本文使用8目标时间初值、对应PVAJ恢复和16路配对，两个TOGT各作者单初值，Wang独立复现N2/M4。本文为最新SX局部几何求导修复版。\n\n|条件|本文自由点|本文中心|动态TOGT自由点|动态TOGT中心|Wang|\n|---|---|---|---|---|---|\n'
for level in levels:
 selected=[r for r in rows if r['level']==level];valid=[r['time_s'] for r in selected if r['outcome']=='通过'];best=min(valid) if valid else None
 values=[]
 for r in selected:
  if r['outcome']=='通过':
   value=f"{r['time_s']:.3f}"
   if r['time_s']==best:value='**'+value+'**'
  elif r['time_s'] is not None:value=f"{r['outcome']}（{r['time_s']:.3f}）"
  else:value=r['outcome']
  values.append(value)
 text+='|'+level+'|'+'|'.join(values)+'|\n'
text+='\n单位秒，包含最后一窗至既定终点，终点与原三赛道适配协议一致。碰撞和未完成括号中保留候选轨迹时间，不参与最快比较。未找到不表示已证明物理不可行。动力学采用各原方法约定，本文为软惩罚，合格结果属于名义规划数值证据。\n\n|条件|本文自由计算|本文中心计算|TOGT自由计算|TOGT中心计算|Wang计算|\n|---|---|---|---|---|---|\n'
for level in levels:
 text+='|'+level+'|'+'|'.join('待完成' if r['compute_s'] is None else f"{r['compute_s']:.3f}" for r in rows if r['level']==level)+'|\n'
text+='\n计算为墙钟秒。本文包含共享并行初值生成、准备及本方法8候选全部求解和检查的实际等待时间，不累加worker耗时。基线包含进程和独立复查墙钟。所有方法此次新算，旧结果保留。场景、模型与初始化见PROTOCOL.md、FREEZE.json、INPUT_AUDIT.json（inputs下）。原始结果在results/ours与results/baselines。\n'
(p/'PLANNING.md').write_text(text)
print('reported',len(ours),'of',len(levels),'conditions')
