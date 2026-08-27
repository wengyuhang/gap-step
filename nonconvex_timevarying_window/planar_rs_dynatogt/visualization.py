"""Generate an interactive replay of a certified Planar-RS trajectory.

The generated HTML is deliberately a diagnostic replay, not a replacement for
the Arb certificate.  It draws the original boundary primitives, the cuboid
centre trajectory, and a dense whole-body point-to-boundary clearance profile.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

import numpy as np
from scipy.optimize import differential_evolution

from nonconvex_timevarying_window.sc_dynatogt.collision import (
    point_to_oriented_cuboid_distance_squared,
)
from nonconvex_timevarying_window.sip_dynatogt.constraints import point_flatness
from nonconvex_timevarying_window.sip_dynatogt.model import (
    PolynomialTrajectory,
    SIPConfig,
)

from .scenario import build_benchmark


DEFAULT_RESULT_DIRECTORY = Path(__file__).with_name("results") / "hard_unlimited_20260826"
DEFAULT_OUTPUT = Path("/home/jack/wyh/visualizations/planar-rs-hard-track.html")


def _locate(trajectory: PolynomialTrajectory, time: float) -> tuple[int, float]:
    cumulative = np.concatenate(([0.0], np.cumsum(trajectory.durations)))
    segment = min(
        int(np.searchsorted(cumulative[1:], float(time), side="right")),
        trajectory.num_segments - 1,
    )
    local = float(time) - float(cumulative[segment])
    tau = local / float(trajectory.durations[segment])
    return segment, min(1.0, max(0.0, tau))


def _boundary_local(window: Any, samples_per_primitive: int) -> list[np.ndarray]:
    parameter = np.linspace(0.0, 1.0, samples_per_primitive)
    return [
        np.asarray([primitive.evaluate(float(u)) for u in parameter], dtype=float)
        for primitive in window.boundary
    ]


def _boundary_world(
    window: Any, primitive: Any, time: float, parameter: np.ndarray
) -> np.ndarray:
    local = np.asarray([primitive.evaluate(float(u)) for u in parameter], dtype=float)
    center, rotation, scale = window.state_at(time)
    local3 = np.column_stack((scale * local, np.zeros(len(local))))
    return center + local3 @ rotation.T


def _clearance_at(
    problem: Any,
    trajectory: PolynomialTrajectory,
    config: SIPConfig,
    time: float,
    window_index: int,
    boundary_index: int,
    parameter: float,
) -> float:
    segment, tau = _locate(trajectory, time)
    flat = point_flatness(trajectory, segment, tau, config)
    window = problem.windows[window_index]
    point = _boundary_world(
        window,
        window.boundary[boundary_index],
        float(time),
        np.asarray([parameter]),
    )[0]
    return math.sqrt(
        max(
            0.0,
            float(
                point_to_oriented_cuboid_distance_squared(
                    point, flat.position, flat.rotation, config.body
                )
            ),
        )
    )


def _sample_replay(
    problem: Any,
    trajectory: PolynomialTrajectory,
    config: SIPConfig,
    *,
    num_times: int = 1601,
    boundary_samples: int = 65,
) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
    crossings = np.cumsum(trajectory.durations)[:-1]
    times = np.unique(
        np.concatenate(
            (np.linspace(0.0, trajectory.total_time, num_times), crossings)
        )
    )
    parameter = np.linspace(0.0, 1.0, boundary_samples)
    clearance = np.empty(len(times))
    positions = np.empty((len(times), 3))
    nearest_window = np.empty(len(times), dtype=int)
    nearest_boundary = np.empty(len(times), dtype=int)
    nearest_parameter = np.empty(len(times))

    for index, time in enumerate(times):
        segment, tau = _locate(trajectory, float(time))
        flat = point_flatness(trajectory, segment, tau, config)
        positions[index] = flat.position
        best = (math.inf, -1, -1, 0.0)
        for window_index, window in enumerate(problem.windows):
            center, rotation, scale = window.state_at(float(time))
            for boundary_index, primitive in enumerate(window.boundary):
                local = np.asarray(
                    [primitive.evaluate(float(u)) for u in parameter], dtype=float
                )
                local3 = np.column_stack((scale * local, np.zeros(len(local))))
                points = center + local3 @ rotation.T
                squared = np.asarray(
                    point_to_oriented_cuboid_distance_squared(
                        points, flat.position, flat.rotation, config.body
                    )
                )
                point_index = int(np.argmin(squared))
                candidate = (
                    float(squared[point_index]),
                    window_index,
                    boundary_index,
                    float(parameter[point_index]),
                )
                if candidate[0] < best[0]:
                    best = candidate
        clearance[index] = math.sqrt(max(0.0, best[0]))
        nearest_window[index] = best[1]
        nearest_boundary[index] = best[2]
        nearest_parameter[index] = best[3]

    # Refine several distinct low-clearance events on the original primitive.
    # This improves the visual marker only; certification still comes from Arb.
    order = np.argsort(clearance)
    candidates: list[tuple[int, int, int]] = []
    seen: set[tuple[int, int]] = set()
    for index in order:
        key = (int(nearest_window[index]), int(nearest_boundary[index]))
        if key in seen:
            continue
        seen.add(key)
        candidates.append((int(index), key[0], key[1]))
        if len(candidates) == 8:
            break
    refined: list[dict[str, float | int]] = []
    grid_step = trajectory.total_time / max(1, num_times - 1)
    for index, window_index, boundary_index in candidates:
        t0 = float(times[index])
        lower = max(0.0, t0 - 2.5 * grid_step)
        upper = min(trajectory.total_time, t0 + 2.5 * grid_step)

        def objective(value: np.ndarray) -> float:
            distance = _clearance_at(
                problem,
                trajectory,
                config,
                float(value[0]),
                window_index,
                boundary_index,
                float(value[1]),
            )
            return distance * distance

        result = differential_evolution(
            objective,
            ((lower, upper), (0.0, 1.0)),
            seed=2317 + window_index * 101 + boundary_index,
            popsize=10,
            maxiter=80,
            tol=1e-10,
            polish=True,
            workers=1,
            updating="immediate",
        )
        refined.append(
            {
                "time": float(result.x[0]),
                "parameter": float(result.x[1]),
                "distance": math.sqrt(max(0.0, float(result.fun))),
                "window": window_index,
                "boundary": boundary_index,
            }
        )
    critical = min(refined, key=lambda value: float(value["distance"]))
    replay = {
        "time": times,
        "position": positions,
        "clearance": clearance,
        "nearest_window": nearest_window,
        "nearest_boundary": nearest_boundary,
        "nearest_parameter": nearest_parameter,
    }
    return replay, critical


def _round_nested(value: Any, digits: int = 8) -> Any:
    if isinstance(value, np.ndarray):
        return _round_nested(value.tolist(), digits)
    if isinstance(value, (np.floating, float)):
        return round(float(value), digits)
    if isinstance(value, (np.integer, int)):
        return int(value)
    if isinstance(value, list):
        return [_round_nested(item, digits) for item in value]
    if isinstance(value, tuple):
        return [_round_nested(item, digits) for item in value]
    if isinstance(value, dict):
        return {key: _round_nested(item, digits) for key, item in value.items()}
    return value


def _load_data(result_directory: Path) -> dict[str, Any]:
    result_payload = json.loads((result_directory / "result.json").read_text())
    verification = json.loads(
        (result_directory / "verification_256.json").read_text()
    )
    values = np.load(result_directory / "trajectory.npz")
    trajectory = PolynomialTrajectory(values["durations"], values["coefficients"])
    config = SIPConfig.from_dict(result_payload["config"]["sip"])
    _, problem = build_benchmark(cache_directory=result_directory / "preprocessing_cache")
    replay, critical = _sample_replay(problem, trajectory, config)
    crossing_times = np.cumsum(trajectory.durations)[:-1]
    crossing_positions = np.asarray(
        [trajectory.evaluate(float(time)) for time in crossing_times]
    )
    windows = []
    for window in problem.windows:
        local = _boundary_local(window, 41)
        windows.append(
            {
                "name": window.name,
                "center": window.center0,
                "rotation": window.fixed_rotation,
                "motion": {
                    "angleAmplitude": window.motion.angle_amplitude,
                    "anglePeriod": window.motion.angle_period,
                    "scaleAmplitude": window.motion.scale_amplitude,
                    "scalePeriod": window.motion.scale_period,
                    "phase": window.motion.phase,
                },
                "primitives": local,
            }
        )
    certificate = verification["certificate"]
    safety_margin_squared = float(certificate["minimum_safety_squared_margin"])
    certified_distance = math.sqrt(config.clearance**2 + safety_margin_squared)
    data = {
        "title": "Planar-RS-DynaTOGT 六窗口闭环赛道",
        "totalTime": trajectory.total_time,
        "clearanceRequired": config.clearance,
        "certifiedDistanceLowerBound": certified_distance,
        "certifiedSurplusMicron": (certified_distance - config.clearance) * 1e6,
        "certificate": {
            "status": certificate["status"],
            "precisionBits": certificate["precision_bits"],
            "checkedCells": certificate["checked_cells"],
            "maximumDepth": certificate["maximum_depth"],
            "minimumDynamicMargin": certificate["minimum_dynamic_margin"],
        },
        "timing": {
            **result_payload["timing_s"],
            "highPrecisionReplay": verification["elapsed_s"],
        },
        "dynamics": result_payload["sampled_dynamic_diagnostics"],
        "crossings": [
            {
                "time": float(time),
                "position": position,
                "window": int(problem.order[index]),
                "name": problem.windows[int(problem.order[index])].name,
            }
            for index, (time, position) in enumerate(
                zip(crossing_times, crossing_positions)
            )
        ],
        "windows": windows,
        "replay": {
            "time": replay["time"],
            "position": replay["position"],
            "clearance": replay["clearance"],
            "nearestWindow": replay["nearest_window"],
            "nearestBoundary": replay["nearest_boundary"],
        },
        "criticalDiagnostic": critical,
    }
    return _round_nested(data)


def _html(data: dict[str, Any]) -> str:
    payload = json.dumps(data, ensure_ascii=False, separators=(",", ":")).replace(
        "</", "<\\/"
    )
    return f'''<div id="prs-viz" class="prs-viz">
  <style>
    #prs-viz {{ --c0:#2563eb;--c1:#f97316;--c2:#16a34a;--c3:#9333ea;--c4:#dc2626;--c5:#0891b2; color:var(--foreground); font-family:var(--font-sans,ui-sans-serif,system-ui,sans-serif); line-height:1.45; }}
    #prs-viz * {{ box-sizing:border-box; }}
    #prs-viz .wrap {{ max-width:1180px; margin:0 auto; padding:18px; }}
    #prs-viz h1 {{ font-size:clamp(22px,3vw,34px); line-height:1.1; margin:0 0 8px; letter-spacing:-.025em; }}
    #prs-viz .lede {{ margin:0 0 16px; color:var(--muted-foreground); max-width:900px; }}
    #prs-viz .metrics {{ display:grid; grid-template-columns:repeat(4,minmax(0,1fr)); gap:10px; margin:0 0 14px; }}
    #prs-viz .metric {{ border:1px solid var(--border); border-radius:12px; padding:12px 13px; background:var(--card); min-width:0; }}
    #prs-viz .metric span {{ display:block; color:var(--muted-foreground); font-size:12px; }}
    #prs-viz .metric strong {{ display:block; font-size:18px; margin-top:2px; white-space:nowrap; overflow:hidden; text-overflow:ellipsis; }}
    #prs-viz #m-status {{ font-size:clamp(12px,3.4vw,18px); }}
    #prs-viz .safe {{ color:#15803d; }}
    #prs-viz .controls {{ display:grid; grid-template-columns:auto 1fr auto; align-items:center; gap:12px; border:1px solid var(--border); border-radius:12px; padding:10px 12px; background:var(--card); margin-bottom:12px; }}
    #prs-viz button {{ border:1px solid var(--border); border-radius:9px; background:var(--background); color:var(--foreground); padding:7px 13px; font:inherit; cursor:pointer; }}
    #prs-viz button:hover {{ background:var(--muted); }}
    #prs-viz input[type=range] {{ width:100%; accent-color:#2563eb; }}
    #prs-viz .now {{ font-variant-numeric:tabular-nums; font-size:13px; min-width:235px; text-align:right; }}
    #prs-viz .views {{ display:grid; grid-template-columns:1fr 1fr; gap:12px; }}
    #prs-viz .panel {{ border:1px solid var(--border); border-radius:12px; background:var(--card); overflow:hidden; min-width:0; }}
    #prs-viz .panel h2 {{ font-size:14px; margin:0; padding:11px 13px 0; }}
    #prs-viz .panel p {{ font-size:12px; color:var(--muted-foreground); margin:3px 13px 0; }}
    #prs-viz .clearance {{ grid-column:1 / -1; }}
    #prs-viz svg {{ display:block; width:100%; height:auto; overflow:visible; }}
    #prs-viz .axis text {{ fill:var(--muted-foreground); font-size:11px; }}
    #prs-viz .axis path,#prs-viz .axis line {{ stroke:var(--border); }}
    #prs-viz .grid line {{ stroke:var(--border); stroke-opacity:.55; }}
    #prs-viz .grid path {{ display:none; }}
    #prs-viz .axis-title {{ fill:var(--muted-foreground); font-size:11px; }}
    #prs-viz .route {{ fill:none; stroke:#2563eb; stroke-width:2.2; stroke-linecap:round; stroke-linejoin:round; }}
    #prs-viz .gate {{ fill:none; stroke-width:1.5; stroke-linejoin:round; opacity:.88; }}
    #prs-viz .cross-dot {{ stroke:var(--card); stroke-width:1.5; }}
    #prs-viz .cross-label {{ fill:var(--foreground); font-size:10px; font-weight:700; text-anchor:middle; dominant-baseline:central; pointer-events:none; }}
    #prs-viz .drone {{ fill:#111827; stroke:white; stroke-width:2; }}
    @media (prefers-color-scheme:dark) {{ #prs-viz .drone {{ fill:#f8fafc; stroke:#111827; }} #prs-viz .safe {{ color:#4ade80; }} }}
    #prs-viz .guide {{ stroke:var(--muted-foreground); stroke-width:1; stroke-dasharray:4 4; }}
    #prs-viz .profile {{ fill:none; stroke:#2563eb; stroke-width:2; }}
    #prs-viz .threshold {{ stroke:#dc2626; stroke-width:1.5; stroke-dasharray:6 4; }}
    #prs-viz .unsafe-band {{ fill:#dc2626; opacity:.11; }}
    #prs-viz .threshold-label {{ fill:#dc2626; font-size:11px; }}
    #prs-viz .critical {{ fill:#dc2626; stroke:var(--card); stroke-width:2; }}
    #prs-viz .cross-line {{ stroke:var(--border); stroke-dasharray:2 4; }}
    #prs-viz .cross-time {{ fill:var(--muted-foreground); font-size:10px; text-anchor:middle; }}
    #prs-viz .legend {{ display:flex; flex-wrap:wrap; gap:8px 15px; margin:9px 13px 12px; font-size:12px; color:var(--muted-foreground); }}
    #prs-viz .legend b {{ display:inline-block; width:17px; height:3px; border-radius:9px; vertical-align:middle; margin-right:5px; }}
    #prs-viz .note {{ margin:10px 2px 0; color:var(--muted-foreground); font-size:12px; }}
    #prs-viz .tooltip {{ position:fixed; z-index:1000; pointer-events:none; opacity:0; background:var(--popover,var(--card)); color:var(--popover-foreground,var(--foreground)); border:1px solid var(--border); border-radius:8px; box-shadow:0 6px 22px rgba(0,0,0,.16); padding:7px 9px; font-size:12px; font-variant-numeric:tabular-nums; }}
    @media(max-width:780px) {{ #prs-viz .metrics {{ grid-template-columns:1fr 1fr; }} #prs-viz .views {{ grid-template-columns:1fr; }} #prs-viz .clearance {{ grid-column:auto; }} #prs-viz .controls {{ grid-template-columns:auto 1fr; }} #prs-viz .now {{ grid-column:1 / -1; text-align:left; min-width:0; }} }}
    @media(max-width:430px) {{ #prs-viz .wrap {{ padding:10px; }} #prs-viz .metrics {{ gap:7px; }} #prs-viz .metric {{ padding:9px; }} #prs-viz .metric strong {{ font-size:15px; }} }}
  </style>
  <div class="wrap">
    <h1>Planar-RS-DynaTOGT 六窗口闭环赛道</h1>
    <p class="lede">拖动时间轴观察无人机与六个固定平面窗口的快速面内旋转和缩放。轨迹从同一点出发并返回，依次穿越 L、U、星形、圆、波浪 Bezier 和线/Bezier 窗口。</p>
    <div class="metrics">
      <div class="metric"><span>Arb 证书</span><strong class="safe" id="m-status"></strong></div>
      <div class="metric"><span>飞行时间</span><strong id="m-time"></strong></div>
      <div class="metric"><span>安全净距阈值</span><strong id="m-clearance"></strong></div>
      <div class="metric"><span>证书最坏下界超额</span><strong id="m-surplus"></strong></div>
    </div>
    <div class="controls">
      <button id="play" type="button" aria-label="播放或暂停轨迹">播放</button>
      <input id="time-slider" type="range" min="0" max="1000" value="0" step="1" aria-label="轨迹时间" />
      <div class="now" id="now"></div>
    </div>
    <div class="views">
      <section class="panel"><h2>俯视图·XY</h2><p>窗口边界会随时间旋转和缩放；实心点为当前无人机中心。</p><svg id="xy" role="img" aria-label="轨迹与时变窗口俯视图"></svg></section>
      <section class="panel"><h2>侧视图·XZ</h2><p>用于辨认高度变化，穿越点编号与赛道顺序一致。</p><svg id="xz" role="img" aria-label="轨迹与时变窗口侧视图"></svg></section>
      <section class="panel clearance"><h2>整机—原始窗口边界最短距离</h2><p>红色区域为净距 &lt; 15 mm；蓝线是高密度采样诊断，严格安全性由 256-bit Arb 区间证书给出。为看清阈值附近，超过 60 mm 的部分在图顶截断。</p><svg id="clearance" role="img" aria-label="整机到所有原始窗口边界的最短距离时间曲线"></svg><div class="legend" id="legend"></div></section>
    </div>
    <p class="note" id="proof-note"></p>
  </div>
  <div class="tooltip" id="tip"></div>
  <script src="https://cdn.jsdelivr.net/npm/d3@7.9.0/dist/d3.min.js"></script>
  <script>
  (() => {{
    const data={payload};
    const root=document.getElementById('prs-viz');
    const colors=['#2563eb','#f97316','#16a34a','#9333ea','#dc2626','#0891b2'];
    const fmt2=d3.format('.2f'), fmt3=d3.format('.3f');
    root.querySelector('#m-status').textContent=data.certificate.status;
    root.querySelector('#m-time').textContent=fmt3(data.totalTime)+' s';
    root.querySelector('#m-clearance').textContent=fmt2(data.clearanceRequired*1000)+' mm';
    root.querySelector('#m-surplus').textContent=fmt3(data.certifiedSurplusMicron)+' µm';
    root.querySelector('#proof-note').textContent=`证书以 ${{data.certificate.precisionBits}}-bit 精度覆盖 ${{data.certificate.checkedCells.toLocaleString()}} 个 Arb 单元，最大细分深度 ${{data.certificate.maximumDepth}}。证明的全域距离下界为 ${{(data.certifiedDistanceLowerBound*1000).toFixed(8)}} mm，不是由图上采样点推断。`;
    root.querySelector('#legend').innerHTML=data.windows.map((w,i)=>`<span><b style="background:${{colors[i]}}"></b>W${{i+1}} ${{w.name}}</span>`).join('')+'<span><b style="background:#dc2626"></b>15 mm 安全阈值</span>';

    const T=data.totalTime, time=data.replay.time, pos=data.replay.position, gap=data.replay.clearance;
    const bisect=d3.bisector(d=>d).left;
    let current=0, playing=false, raf=null, started=0, base=0;
    const slider=root.querySelector('#time-slider'), now=root.querySelector('#now'), play=root.querySelector('#play');
    function nearestIndex(t) {{ const i=bisect(time,t); return Math.max(0,Math.min(time.length-1,(i>0&&Math.abs(time[i-1]-t)<Math.abs((time[i]??Infinity)-t))?i-1:i)); }}
    function angleScale(w,t) {{ const m=w.motion; return {{a:m.angleAmplitude*Math.sin(2*Math.PI*t/m.anglePeriod+m.phase), s:1+m.scaleAmplitude*Math.sin(2*Math.PI*t/m.scalePeriod+m.phase)}}; }}
    function matVec(R,v) {{ return [R[0][0]*v[0]+R[0][1]*v[1]+R[0][2]*v[2],R[1][0]*v[0]+R[1][1]*v[1]+R[1][2]*v[2],R[2][0]*v[0]+R[2][1]*v[1]+R[2][2]*v[2]]; }}
    function worldPrimitive(w,primitive,t) {{ const q=angleScale(w,t), c=Math.cos(q.a),s=Math.sin(q.a); return primitive.map(p=>{{ const local=[q.s*(c*p[0]-s*p[1]),q.s*(s*p[0]+c*p[1]),0]; const v=matVec(w.rotation,local); return [w.center[0]+v[0],w.center[1]+v[1],w.center[2]+v[2]]; }}); }}
    function setTime(t,fromSlider=false) {{ current=Math.max(0,Math.min(T,t)); if(!fromSlider) slider.value=Math.round(current/T*1000); const i=nearestIndex(current), wi=data.replay.nearestWindow[i]; now.textContent=`t=${{fmt3(current)}} s · 最近 W${{wi+1}} ${{data.windows[wi].name}} · 采样净距 ${{fmt2(gap[i]*1000)}} mm`; updateMarks(i); }}
    slider.addEventListener('input',()=>{{ playing=false;play.textContent='播放'; if(raf)cancelAnimationFrame(raf);setTime(+slider.value/1000*T,true); }});
    play.addEventListener('click',()=>{{ playing=!playing;play.textContent=playing?'暂停':'播放';if(playing){{base=current;started=performance.now();raf=requestAnimationFrame(tick);}}else if(raf)cancelAnimationFrame(raf); }});
    function tick(ts) {{ if(!playing)return; let t=base+(ts-started)/1000; if(t>T){{t=0;base=0;started=ts;}} setTime(t);raf=requestAnimationFrame(tick); }}

    let viewState=[];
    function drawView(selector,a,b,aLabel,bLabel) {{
      const node=root.querySelector(selector), w=Math.max(300,node.clientWidth||520), h=Math.max(270,Math.min(390,w*.68)), m={{t:18,r:18,b:43,l:52}};
      const svg=d3.select(node).attr('viewBox',`0 0 ${{w}} ${{h}}`);svg.selectAll('*').remove();
      const all=data.replay.position.concat(data.windows.map(d=>d.center)); const ex=d3.extent(all,d=>d[a]), ey=d3.extent(all,d=>d[b]); const px=(ex[1]-ex[0])*.08+1, py=(ey[1]-ey[0])*.12+1;
      const x=d3.scaleLinear().domain([ex[0]-px,ex[1]+px]).range([m.l,w-m.r]); const y=d3.scaleLinear().domain([ey[0]-py,ey[1]+py]).range([h-m.b,m.t]);
      const g=svg.append('g');g.append('g').attr('class','grid').attr('transform',`translate(0,${{h-m.b}})`).call(d3.axisBottom(x).ticks(w<430?4:6).tickSize(-(h-m.t-m.b)).tickFormat(''));g.append('g').attr('class','grid').attr('transform',`translate(${{m.l}},0)`).call(d3.axisLeft(y).ticks(5).tickSize(-(w-m.l-m.r)).tickFormat(''));
      g.append('g').attr('class','axis').attr('transform',`translate(0,${{h-m.b}})`).call(d3.axisBottom(x).ticks(w<430?4:6));g.append('g').attr('class','axis').attr('transform',`translate(${{m.l}},0)`).call(d3.axisLeft(y).ticks(5));
      svg.append('text').attr('class','axis-title').attr('x',(m.l+w-m.r)/2).attr('y',h-7).attr('text-anchor','middle').text(aLabel+' (m)');svg.append('text').attr('class','axis-title').attr('transform','rotate(-90)').attr('x',-(m.t+h-m.b)/2).attr('y',13).attr('text-anchor','middle').text(bLabel+' (m)');
      const line=d3.line().x(d=>x(d[a])).y(d=>y(d[b]));g.append('path').datum(data.replay.position).attr('class','route').attr('d',line);
      const gates=g.append('g').selectAll('g').data(data.windows).join('g');gates.each(function(win,wi){{d3.select(this).selectAll('path').data(win.primitives).join('path').attr('class','gate').attr('stroke',colors[wi]);}});
      const crosses=g.append('g').selectAll('g').data(data.crossings).join('g');crosses.append('circle').attr('class','cross-dot').attr('r',9).attr('fill',(d,i)=>colors[i]).attr('cx',d=>x(d.position[a])).attr('cy',d=>y(d.position[b]));crosses.append('text').attr('class','cross-label').attr('x',d=>x(d.position[a])).attr('y',d=>y(d.position[b])).text((d,i)=>i+1);
      const drone=g.append('circle').attr('class','drone').attr('r',6);viewState.push({{a,b,x,y,line,gates,drone}});
    }}
    function drawClearance() {{
      const node=root.querySelector('#clearance'), w=Math.max(300,node.clientWidth||1000),h=Math.max(290,Math.min(360,w*.38)),m={{t:25,r:22,b:45,l:60}},maxY=.060;
      const svg=d3.select(node).attr('viewBox',`0 0 ${{w}} ${{h}}`);svg.selectAll('*').remove();const x=d3.scaleLinear().domain([0,T]).range([m.l,w-m.r]),y=d3.scaleLinear().domain([0,maxY]).range([h-m.b,m.t]);
      svg.append('rect').attr('class','unsafe-band').attr('x',m.l).attr('width',w-m.l-m.r).attr('y',y(data.clearanceRequired)).attr('height',y(0)-y(data.clearanceRequired));
      svg.append('g').attr('class','grid').attr('transform',`translate(0,${{h-m.b}})`).call(d3.axisBottom(x).ticks(w<500?5:9).tickSize(-(h-m.t-m.b)).tickFormat(''));svg.append('g').attr('class','grid').attr('transform',`translate(${{m.l}},0)`).call(d3.axisLeft(y).ticks(6).tickSize(-(w-m.l-m.r)).tickFormat(''));
      svg.append('g').selectAll('line').data(data.crossings).join('line').attr('class','cross-line').attr('x1',d=>x(d.time)).attr('x2',d=>x(d.time)).attr('y1',m.t).attr('y2',h-m.b);svg.append('g').selectAll('text').data(data.crossings).join('text').attr('class','cross-time').attr('x',d=>x(d.time)).attr('y',m.t-7).text((d,i)=>'W'+(i+1));
      svg.append('path').datum(time.map((t,i)=>[t,Math.min(maxY,gap[i])])).attr('class','profile').attr('d',d3.line().x(d=>x(d[0])).y(d=>y(d[1])));
      svg.append('line').attr('class','threshold').attr('x1',m.l).attr('x2',w-m.r).attr('y1',y(data.clearanceRequired)).attr('y2',y(data.clearanceRequired));svg.append('text').attr('class','threshold-label').attr('x',w-m.r-3).attr('y',y(data.clearanceRequired)-5).attr('text-anchor','end').text('15 mm');
      const crit=data.criticalDiagnostic;svg.append('circle').attr('class','critical').attr('r',5).attr('cx',x(crit.time)).attr('cy',y(Math.min(maxY,crit.distance)));svg.append('text').attr('class','threshold-label').attr('x',x(crit.time)+8).attr('y',y(Math.min(maxY,crit.distance))-7).text('精细化诊断 '+(crit.distance*1000).toFixed(4)+' mm');
      svg.append('g').attr('class','axis').attr('transform',`translate(0,${{h-m.b}})`).call(d3.axisBottom(x).ticks(w<500?5:9));svg.append('g').attr('class','axis').attr('transform',`translate(${{m.l}},0)`).call(d3.axisLeft(y).ticks(6).tickFormat(d=>d*1000));svg.append('text').attr('class','axis-title').attr('x',(m.l+w-m.r)/2).attr('y',h-7).attr('text-anchor','middle').text('时间 (s)');svg.append('text').attr('class','axis-title').attr('transform','rotate(-90)').attr('x',-(m.t+h-m.b)/2).attr('y',14).attr('text-anchor','middle').text('最短距离 (mm)');
      const guide=svg.append('line').attr('class','guide').attr('y1',m.t).attr('y2',h-m.b),dot=svg.append('circle').attr('r',4.5).attr('fill','#2563eb').attr('stroke','white').attr('stroke-width',1.5);clearanceState={{x,y,guide,dot,maxY}};
      const overlay=svg.append('rect').attr('x',m.l).attr('y',m.t).attr('width',w-m.l-m.r).attr('height',h-m.t-m.b).attr('fill','transparent').style('cursor','crosshair');const tip=root.querySelector('#tip');overlay.on('mousemove',event=>{{const p=d3.pointer(event),t=x.invert(p[0]),i=nearestIndex(t),wi=data.replay.nearestWindow[i];tip.style.opacity='1';tip.style.left=(event.clientX+12)+'px';tip.style.top=(event.clientY+12)+'px';tip.textContent=`${{fmt3(time[i])}} s · ${{fmt3(gap[i]*1000)}} mm · W${{wi+1}} ${{data.windows[wi].name}}`;}}).on('mouseleave',()=>tip.style.opacity='0').on('click',event=>{{const p=d3.pointer(event);setTime(x.invert(p[0]));}});
    }}
    let clearanceState=null;
    function updateMarks(i) {{
      viewState.forEach(v=>{{v.gates.each(function(win){{d3.select(this).selectAll('path').attr('d',primitive=>v.line(worldPrimitive(win,primitive,current)));}});v.drone.attr('cx',v.x(pos[i][v.a])).attr('cy',v.y(pos[i][v.b]));}});
      if(clearanceState){{clearanceState.guide.attr('x1',clearanceState.x(current)).attr('x2',clearanceState.x(current));clearanceState.dot.attr('cx',clearanceState.x(current)).attr('cy',clearanceState.y(Math.min(clearanceState.maxY,gap[i])));}}
    }}
    let resizeTimer=null;function redraw(){{viewState=[];drawView('#xy',0,1,'X','Y');drawView('#xz',0,2,'X','Z');drawClearance();setTime(current);}}
    new ResizeObserver(()=>{{clearTimeout(resizeTimer);resizeTimer=setTimeout(redraw,80);}}).observe(root.querySelector('.views'));
    redraw();
  }})();
  </script>
</div>'''


def generate(result_directory: Path, output: Path) -> Path:
    data = _load_data(result_directory)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(_html(data), encoding="utf-8")
    return output


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--result-dir", type=Path, default=DEFAULT_RESULT_DIRECTORY)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    path = generate(args.result_dir.expanduser(), args.output.expanduser())
    print(path)


if __name__ == "__main__":
    main()
