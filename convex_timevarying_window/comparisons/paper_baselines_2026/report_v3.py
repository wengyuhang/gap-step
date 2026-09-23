"""Evidence-labelled comparison from saved runs; never reruns controllers."""
import argparse
from collections import Counter
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from convex_dynamic_seven_window_gazebo.x500_togt.experiment import build_track
from convex_timevarying_window.receding_gate_racing.scenario import vary_track


def main():
    p=argparse.ArgumentParser();p.add_argument("run",type=Path);a=p.parse_args()
    status=json.loads((a.run/"status.json").read_text())
    if status["state"]!="finished":raise RuntimeError("Do not report an incomplete batch as completed")
    rows=json.loads((a.run/"summary.json").read_text())
    previous=Path(__file__).parent/"results/two_papers_20260919"
    polynomial=json.loads((previous/"polynomial_matrix/summary.json").read_text())
    strict=lambda r:bool(r.get("physical_success") and r.get("realtime_success"))
    successes=sum(map(strict,rows));contacts=sum(bool(r.get("collision")) for r in rows)
    ms=[];cadence=[];speeds=[];updates=[];clearances=[];plane_events=[]
    fig,axes=plt.subplots(2,2,figsize=(11,8),constrained_layout=True)
    x=np.arange(16)
    axes[0,0].bar(x-.18,[r.get("gates",0) for r in polynomial],width=.36,label="2023 polynomial (old runs)")
    axes[0,0].bar(x+.18,[r.get("gates",0) for r in rows],width=.36,label="V3 (new runs)")
    axes[0,0].set(xlabel="Matched case index",ylabel="Completed gates",ylim=(0,7.5))
    axes[0,0].legend(fontsize=8)
    axes[0,1].bar(x,[r.get("last_time_s",0) or 0 for r in rows],color="#c67932")
    axes[0,1].set(xlabel="Case index",ylabel="Time before termination (s)",title="Not successful lap times")
    for i,r in enumerate(rows):
        path=Path(r["run"])
        if not (path/"result.json").exists():continue
        result=json.loads((path/"result.json").read_text())
        controls=[s for s in result["solves"] if s.get("status")=="direct_motor_mppi"]
        values=[s["cycle_ms"] for s in controls];ms.extend(values)
        cadence.extend((np.diff([s["time"] for s in controls])*1000).tolist())
        updates.extend(result.get("reference_updates",[]))
        data=np.atleast_1d(np.genfromtxt(path/"telemetry.csv",names=True,delimiter=","))
        if len(data):
            speed=np.sqrt(data['vx']**2+data['vy']**2+data['vz']**2);speeds.extend(speed.tolist())
            if i in (0,6,10,14):axes[1,0].plot(data['t'],speed,label=f"case {i}",linewidth=.8)
            track,_=build_track()
            vary_track(track,r['phase'],0.,r['motion_rate'],r['translation_scale'],r['rotation_scale'])
            positions=np.column_stack((data['x'],data['y'],data['z']))
            for gate,window in enumerate(track.windows):
                center,rotation,*_=window.state_at(0.)
                before=track.start if gate==0 else track.windows[gate-1].state_at(0.)[0]
                sign=-np.sign(rotation[:,2]@(before-center))
                local=np.array([np.r_[*window.world_to_local(position,float(t))]
                                for position,t in zip(positions,data['t'])])
                signed=local[:,2]*sign
                indices=np.flatnonzero((signed[:-1]<0)&(signed[1:]>=0)&(data['gate'][:-1]==gate))
                for j in indices:
                    alpha=-signed[j]/(signed[j+1]-signed[j])
                    t=float(data['t'][j]+alpha*(data['t'][j+1]-data['t'][j]))
                    position=(1-alpha)*positions[j]+alpha*positions[j+1]
                    q,z=window.world_to_local(position,t)
                    plane_events.append({'case':i,'gate':gate,'time':t,'q':q.tolist(),
                        'inside_physical_aperture':window.aperture.contains(q),
                        'distance_to_boundary':float(window.aperture.boundary_distance(q)),
                        'evidence':'interpolated estimated centre-plane event, NOT full-body completion'})
        clear=r.get("min_clearance_m")
        if clear is not None:clearances.append(clear)
    axes[1,0].set(xlabel="Race clock (s)",ylabel="Speed (m/s)",title="Selected cases, failures retained")
    axes[1,0].legend(fontsize=8)
    if ms:axes[1,1].hist(ms,bins=50,color="#348c97")
    axes[1,1].axvline(20,color="orange",linestyle="--",label="20 ms target")
    axes[1,1].axvline(100,color="red",linestyle=":",label="100 ms limit")
    axes[1,1].set(xlabel="Full MPPI computation (ms)",ylabel="Cycles")
    axes[1,1].legend(fontsize=8)
    for ax in axes.flat:ax.grid(alpha=.15)
    fig.suptitle("Frozen PX4 integration evaluation — independent implementations\nRL transfer not validated: no invented 0/16 RL score")
    fig.savefig(a.run/"comparison.png",dpi=170);plt.close(fig)
    (a.run/"plane_events.json").write_text(json.dumps(plane_events,indent=2)+"\n")
    fig,axes=plt.subplots(1,2,figsize=(10,5),constrained_layout=True)
    track,_=build_track()
    for gate,ax in enumerate(axes):
        boundary=track.windows[gate].aperture.boundary_points()
        boundary=np.vstack((boundary,boundary[0]))
        ax.plot(boundary[:,0],boundary[:,1],color='black',label='Physical opening')
        events=[e for e in plane_events if e['gate']==gate]
        for inside,color,label in ((True,'#218c65','Inside centre-plane event'),(False,'#c84635','Missed opening')):
            selected=[e for e in events if e['inside_physical_aperture']==inside]
            if selected:
                q=np.array([e['q'] for e in selected]);ax.scatter(q[:,0],q[:,1],color=color,s=14,label=label,alpha=.7)
        ax.set(title=f'Gate {gate+1}: all cases',xlabel='Local q1 (m)',ylabel='Local q2 (m)',aspect='equal')
        ax.grid(alpha=.2);ax.legend(fontsize=7)
    fig.suptitle('Moving-frame crossing locations — centre crossing alone is NOT success')
    fig.savefig(a.run/'crossing_positions.png',dpi=170);plt.close(fig)
    statistics={"cases":len(rows),"strict_successes":successes,"contact_cases":contacts,
        "failure_counts":dict(Counter(str(r.get("failure")) for r in rows)),
        "mppi_cycles":len(ms),"mppi_ms_median":float(np.median(ms)) if ms else None,
        "mppi_ms_max":max(ms) if ms else None,"mppi_ms_p99":float(np.percentile(ms,99)) if ms else None,
        "mppi_over_20ms":sum(v>20 for v in ms),
        "mppi_over_100ms":sum(v>100 for v in ms),
        "control_start_interval_ms_median":float(np.median(cadence)) if cadence else None,
        "max_speed_mps":max(speeds) if speeds else None,"minimum_truth_sampled_clearance_m":min(clearances) if clearances else None,
        "centre_plane_events":len(plane_events),
        "centre_plane_events_outside_aperture":sum(not e['inside_physical_aperture'] for e in plane_events),
        "reference_results":len(updates),"accepted_reference_results":sum(bool(r["accepted"]) for r in updates),
        "polynomial_strict_successes":sum(map(strict,polynomial)),
        "rl":"5e8-step independent training completed, PX4 transfer not validated; no formal 16-case score"}
    (a.run/"statistics.json").write_text(json.dumps(statistics,indent=2)+"\n")
    lines=["# V3与两篇旧baseline：同条件完整评估", "", "本次仅运行V3，16条件全部结束；旧两篇结果未改动或重跑。",
        "这是当前独立实现的PX4集成评估，不是作者官方性能比较，也不意味着文档算法已经成功。", "",
        f"V3严格成功 **{successes}/16**；发生门套接触的试飞 **{contacts}/16**；失败不填写成功圈时。",
        "2023多项式旧实现严格8/16（另有1次解锁失败）；2026学习方法未通过PX4迁移，不能写成0/16。", "",
        "|编号|频率|平移幅度|摆角幅度|相位°|V3过门|V3终止原因|V3严格成功|2023严格成功|",
        "|---:|---:|---:|---:|---:|---:|---|---|---|"]
    for i,r in enumerate(rows):
        lines.append(f"|{i}|{r['motion_rate']}|{r['translation_scale']}|{r['rotation_scale']}|{np.rad2deg(r['phase']):.0f}|{r.get('gates',0)}|{r.get('failure')}|{strict(r)}|{strict(polynomial[i])}|")
    lines += ["", "## 耗时与证据边界", "", "```json",json.dumps(statistics,indent=2,ensure_ascii=False),"```", "",
        "[曲线图](comparison.png)、[门平面穿越位置](crossing_positions.png)。后者是估计中心的插值过平面事件，不等于完整合法出门。",
        "净空来自旧协议的10 ms真值球模型，实际接触单独统计；不是连续安全证书。",
        "40 ms是控制生效前瞻，不是20 ms目标周期；实际启动间隔和超期必须一起读，不能只看中位计算耗时。",
        "PX4直控与旧基线PVA/CTBR的执行层不同，训练信息也不对齐，不能据此作普遍算法优劣结论。",
        "本轮PX4先建立已检查的控制缓存，MPPI采样优先延续缓存；数值开发中的初始TOGT推力种子不是本轮PX4实际基准输入。",
        "中心过平面事件可能包含门平面自身运动；不能视为主动穿窗次数。孔外事件说明存在绕门现象，具体代价/采样原因尚未通过消融验证。",
        "协议见上级V3_PROTOCOL.md，完成核对见本目录AUDIT.md。",
        "没有共同成功的新方法圈速时，不计算‘快了多少’。失败后的位置接管是仿真恢复，不是算法安全后备。"]
    (a.run/"REPORT.md").write_text("\n".join(lines)+"\n")
    print(json.dumps(statistics,indent=2))


if __name__=="__main__":main()
