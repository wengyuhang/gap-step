"""Motion-envelope / nonincident-flight-corridor geometry preflight."""
import numpy as np
import trimesh
from . import scenario
from ..trajectory_only.collision import BODY_RADIUS

CORRIDOR_EXTRA_M=1.
MIN_FOREIGN_GAP_M=2.


def point_segment_distance(p,a,b):
    u=float(np.clip((p-a)@(b-a)/np.sum((b-a)**2),0.,1.))
    return float(np.linalg.norm(p-(a+u*(b-a))))


def audit():
    tr,_=scenario.build_case(scenario.Case(level=scenario.LEVELS[-1]));motion=[];frame=[]
    for w in tr.windows:
        mesh=trimesh.load_mesh(scenario.ASSETS/'meshes'/f'gate_{w.name}_sleeve.stl')
        frame.append(float(np.linalg.norm(mesh.vertices,axis=1).max()))
        m=w.motion
        motion.append(m.amplitude if m.kind=='translation' else 2*m.length*np.sin(m.angle/2))
    motion=np.array(motion);frame=np.array(frame)
    sweep=motion+frame+BODY_RADIUS;corridor=float(motion.max()+CORRIDOR_EXTRA_M)
    nodes=np.vstack([scenario.START,scenario.ANCHORS,scenario.START]);rows=[]
    for piece,(a,b) in enumerate(zip(nodes[:-1],nodes[1:])):
        for gate,w in enumerate(tr.windows):
            if piece in (gate,gate+1):continue
            distance=point_segment_distance(w.center0,a,b)
            rows.append(dict(piece=piece,foreign_gate=gate+1,anchor_distance_m=distance,
                             corridor_to_inflated_sweep_gap_m=float(distance-corridor-sweep[gate])))
    pairs=[dict(gates=[i+1,j+1],inflated_sweep_gap_m=float(np.linalg.norm(scenario.ANCHORS[i]-scenario.ANCHORS[j])-sweep[i]-sweep[j]))
           for i in range(7) for j in range(i)]
    minimum=min(r['corridor_to_inflated_sweep_gap_m'] for r in rows)
    pair_min=min(r['inflated_sweep_gap_m'] for r in pairs)
    return dict(passed=bool(minimum>=MIN_FOREIGN_GAP_M and pair_min>0),
        nominal_route_length_m=float(np.linalg.norm(np.diff(nodes,axis=0),axis=1).sum()),
        min_nonincident_gap_m=minimum,min_pair_sweep_gap_m=pair_min,
        required_nonincident_gap_m=MIN_FOREIGN_GAP_M,corridor_radius_m=corridor,
        body_radius_m=BODY_RADIUS,maximum_motion_displacement_m=motion.tolist(),
        mesh_bounding_radii_m=frame.tolist(),inflated_sweep_radii_m=sweep.tolist(),
        per_piece_foreign_checks=rows,pair_checks=pairs,
        scope='all phase extrema at the highest per-gate amplitude; nonincident anchor-segment corridors only',
        limitation='does not constrain or certify arbitrary optimized trajectories outside these corridors; all actual curves remain fully checked')


def motion_audit():
    import casadi as ca
    t=ca.MX.sym('t');max_error=0.;checks=0
    for level in scenario.LEVELS:
        tr,_=scenario.build_case(scenario.Case(level=level))
        for w in tr.windows:
            fn=ca.Function('pose',[t],list(scenario.pose(w,t)))
            for a,b in [(0.,.173),(.137,.581),(.531,1.213),(1.101,2.372)]:
                tt=np.linspace(a,b,101);p,_=w.motion.local(tt);lo,hi=w.motion.local_bounds(a,b)
                assert np.all(p>=lo-1e-11) and np.all(p<=hi+1e-11)
                assert np.max(abs(p[:,2]))<1e-12
                for at in tt[::25]:
                    c,R,_,_=w.state_at(float(at));error=float(np.max(abs(np.array(fn(at)[0]).ravel()-c)))
                    max_error=max(max_error,error);assert error<1e-10;assert np.max(abs(np.array(fn(at)[1])-R))<1e-12
                    checks+=1
    return dict(passed=True,symbolic_numeric_pose_max_error_m=max_error,pose_checks=checks,
                all_ten_profiles_motion_bounds_tested=True,planes_fixed=True)


def foreign_diagnostics(assessment):
    checks=assessment.get('checks',[])
    foreign=[r for r in checks if r['piece'] not in (int(r['gate'][:2])-1,int(r['gate'][:2])) and r['outcome']!='clear']
    return dict(nonincident_collision_count=sum(r['outcome']=='collision' for r in foreign),
                nonincident_unresolved_count=sum(r['outcome']=='unresolved' for r in foreign),
                foreign_frame_checks=foreign,
                scene_assumption_passed=not foreign)


def figure(out,geometry):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle
    tr,_=scenario.build_case(scenario.Case(level=scenario.LEVELS[-1]))
    fig,ax=plt.subplots(figsize=(11,6))
    nodes=np.vstack([scenario.START,scenario.ANCHORS,scenario.START]);ax.plot(*nodes[:,:2].T,'--',color='.5',lw=1.2,label='Ordered route + return')
    for i,w in enumerate(tr.windows):
        c='#246ca4' if w.motion.kind=='translation' else '#d57424'
        ax.add_patch(Circle(w.center0[:2],geometry['inflated_sweep_radii_m'][i],facecolor=c,edgecolor=c,alpha=.13))
        path=w.state_at(np.linspace(0,1/w.motion.frequency,501))[0]
        ax.plot(*path[:,:2].T,color=c,lw=1.5)
        boundary=w.boundary_at(0);ax.plot(*np.vstack([boundary,boundary[0]])[:,:2].T,color=c,lw=2)
        ax.annotate(f"W{i+1}: {'translate' if i%2==0 else 'swing'}",w.center0[:2],xytext=(3,7),textcoords='offset points',fontsize=8)
    ax.scatter(*scenario.START[:2],marker='*',s=120,c='black',label='Start / finish')
    ax.set(xlabel='x (m)',ylabel='y (m)',title='Seven-gate course: maximum swept-frame envelopes')
    ax.set_aspect('equal');ax.grid(alpha=.15);ax.legend(fontsize=8);fig.tight_layout()
    fig.savefig(out/'course_layout.png',dpi=180);fig.savefig(out/'course_layout.pdf');plt.close(fig)
