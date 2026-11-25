"""Read-only report for the ongoing independent eight-start development run."""
from pathlib import Path
import json
HERE=Path(__file__).resolve().parent
OUT=HERE/'results/repair_phase_eight_v2_20261113'
def main():
 coord=json.loads((OUT/'status.json').read_text());rows=[]
 for seed in range(8):
  p=OUT/f'ours_{seed}';s=json.loads((p/'status.json').read_text()) if (p/'status.json').exists() else {};attempts=json.loads((p/'attempts.json').read_text()) if (p/'attempts.json').exists() else [];r=json.loads((p/'best_result.json').read_text()) if (p/'best_result.json').exists() else {};rows.append(dict(seed=seed,state=s.get('state','running_first_attempt'),qualified=bool(r.get('found') and r.get('independent_frame_clear') and r.get('independent_task_passed')),time_s=r.get('nominal_time_s'),completed_attempts=len(attempts),failed_attempts=sum(not a['qualified'] for a in attempts),elapsed_at_last_completed_attempt_s=s.get('elapsed_s'),finite_failed_calls=sum(st!='Solve_Succeeded' for a in attempts for st in a.get('finite_statuses',[]))))
 lines=['# 高难统一八起点开发验证','','原始初值均不同。最终轨迹时间包含七窗和返航。所有接纳结果均为优化后全窗数值复查与任务检查通过，没有实飞。','',f"协调状态 {coord['state']}，当前墙钟 {coord['elapsed_s']:.1f} 秒。仍在运行时，下表为当前最好结果，不是最终批次成绩。",'', '|原始初值|状态|当前合格时间/s|已结束尝试|失败尝试|非正常有限调用|','|---|---|---|---|---|---|']
 for r in rows:
  t='未得到' if r['time_s'] is None else f"{r['time_s']:.6f}";lines.append(f"|{r['seed']}|{r['state']}|{t}|{r['completed_attempts']}|{r['failed_attempts']}|{r['finite_failed_calls']}|")
 lines+=['','算法先求解原始安全初值，再作整周期提前到达，失败时减小跳转幅度。轨迹点和完整PVAJ随后按原目标联合优化。所有失败和时间都保留，没有借用其他初值或其他方法结果。','', '部分阶段有限求解曾有上限/恢复失败等非正常终止，不能宣称每个有限问题均收敛。此处与低中难79个有限问题全部正常的结果分别报告。','', '32条已完整验证低中难结果见 ../repair_verified_low_medium_v1_20261113/REPORT.md。九赛道五方法正式对比保持暂停。']
 (OUT/'REPORT.md').write_text('\n'.join(lines)+'\n');(OUT/'CURRENT_RESULTS.json').write_text(json.dumps(rows,ensure_ascii=False,indent=2)+'\n');print([(r['seed'],r['qualified'],r['time_s'],r['state']) for r in rows])
if __name__=='__main__':main()
