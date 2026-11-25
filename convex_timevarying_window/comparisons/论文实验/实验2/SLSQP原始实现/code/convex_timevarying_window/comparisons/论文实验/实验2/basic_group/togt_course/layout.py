"""Oriented-frame swept AABBs and 3-D TOGT-layout inspection, not a new route."""
import numpy as np
import trimesh
from . import scenario
from ..trajectory_only.collision import BODY_RADIUS
from ..separated_course.layout import foreign_diagnostics


def segment_box_distance(a,b,lo,hi):
    d=b-a;cuts=[0.,1.]
    for i in range(3):
        if abs(d[i])>1e-14:
            cuts += [float(u) for u in ((lo[i]-a[i])/d[i],(hi[i]-a[i])/d[i]) if 0<u<1]
    cuts=sorted(set(cuts));best=np.inf
    def value(u):return float(np.linalg.norm(np.maximum(np.maximum(lo-a-u*d,a+u*d-hi),0.)))
    for x,y in zip(cuts[:-1],cuts[1:]):
        p=a+(x+y)/2*d;active=(p<lo)|(p>hi);edge=np.where(p<lo,lo,hi)
        denom=float(d[active]@d[active]);u=float(np.clip(d[active]@(edge-a)[active]/denom,x,y)) if denom>0 else x
        best=min(best,value(x),value(y),value(u))
    return best


def audit():
    tr,_=scenario.build_case(scenario.Case(level=scenario.LEVELS[-1]));boxes=[]
    for w in tr.windows:
        v=trimesh.load_mesh(scenario.ASSETS/'meshes'/f'gate_{w.name}_sleeve.stl').vertices@w.R.T
        lo,hi=w.motion.translation_bounds(0.,1/w.motion.frequency)
        boxes.append((w.center0+v.min(0)+lo-BODY_RADIUS,w.center0+v.max(0)+hi+BODY_RADIUS))
    nodes=np.vstack([scenario.START,scenario.ANCHORS,scenario.START]);checks=[]
    for k,(a,b) in enumerate(zip(nodes[:-1],nodes[1:])):
        for j,(lo,hi) in enumerate(boxes):
            if k in (j,j+1):continue
            checks.append(dict(piece=k,foreign_gate=j+1,transit_tube_radius_m=1.,
                tube_to_inflated_swept_box_gap_m=segment_box_distance(a,b,lo,hi)-1.))
    pairs=[dict(gates=[i+1,j+1],inflated_box_gap_m=float(np.linalg.norm(np.maximum(np.maximum(boxes[i][0]-boxes[j][1],boxes[j][0]-boxes[i][1]),0.))))
           for i in range(7) for j in range(i)]
    gap=min(r['tube_to_inflated_swept_box_gap_m'] for r in checks);pair=min(r['inflated_box_gap_m'] for r in pairs)
    return dict(passed=bool(gap>2. and pair>0.),nominal_route_length_m=float(np.linalg.norm(np.diff(nodes,axis=0),axis=1).sum()),
        min_nonincident_gap_m=gap,min_pair_sweep_gap_m=pair,maximum_swept_boxes=[dict(lower=a.tolist(),upper=b.tolist()) for a,b in boxes],
        per_piece_foreign_checks=checks,pair_checks=pairs,
        scope='maximum per-gate amplitudes, exact analytic motion bounds and oriented physical STL vertex extents; 1m anchor-segment tubes',
        limitation='transit-tube clearance is a layout screen, not an arbitrary-trajectory guarantee; all actual references remain checked')


def motion_audit():
    import casadi as ca
    t=ca.MX.sym('t');error=0.;count=0
    for level in scenario.LEVELS:
        tr,_=scenario.build_case(scenario.Case(level=level))
        for w in tr.windows:
            fn=ca.Function('pose',[t],list(scenario.pose(w,t)))
            for a,b in ((0.,.173),(.137,.581),(.531,1.213),(1.101,2.372)):
                tt=np.linspace(a,b,101);p,_=w.motion.local(tt);lo,hi=w.motion.local_bounds(a,b)
                assert np.all(p>=lo-1e-11) and np.all(p<=hi+1e-11)
                for at in tt[::25]:
                    c,R,_,_=w.state_at(float(at));e=float(np.max(abs(np.array(fn(at)[0]).ravel()-c)))
                    error=max(error,e);assert e<1e-10;assert np.max(abs(np.array(fn(at)[1])-R))<1e-12;count+=1
    return dict(passed=True,symbolic_numeric_pose_max_error_m=error,pose_checks=count,planes_fixed=True)


def figure(out,geometry):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    tr,_=scenario.build_case(scenario.Case());nodes=np.vstack([scenario.START,scenario.ANCHORS,scenario.START])
    fig=plt.figure(figsize=(13,6));ax=fig.add_subplot(121,projection='3d');top=fig.add_subplot(122)
    ax.plot(*nodes.T,'--',color='.5',lw=1);top.plot(*nodes[:,:2].T,'--',color='.5',lw=1)
    for i,w in enumerate(tr.windows):
        c='#246ca4' if w.motion.kind=='translation' else '#d57424';p=w.boundary_at(0);p=np.vstack([p,p[0]])
        ax.plot(*p.T,c=c,lw=2);top.plot(*p[:,:2].T,c=c,lw=2)
        ax.text(*w.center0,str(i+1),fontsize=10)
        top.annotate(str(i+1),w.center0[:2],xytext=((7 if i==3 else -12),7),textcoords='offset points')
    ax.scatter(*scenario.START,c='black',marker='*',s=80);top.scatter(*scenario.START[:2],c='black',marker='*',s=80)
    ax.set(xlabel='x (m)',ylabel='y (m)',zlabel='z (m)',title='Official TOGT layout, x500 task adaptation')
    ax.view_init(elev=23,azim=-57);ax.set_box_aspect((1.4,1.4,.7))
    top.set(xlabel='x (m)',ylabel='y (m)',title='Top view: interlacing and stacked W4/W5')
    top.set_aspect('equal');top.grid(alpha=.15)
    fig.tight_layout();fig.savefig(out/'course_layout.png',dpi=180);fig.savefig(out/'course_layout.pdf');plt.close(fig)
