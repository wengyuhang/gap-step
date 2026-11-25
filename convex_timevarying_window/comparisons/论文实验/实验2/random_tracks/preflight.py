from bootstrap import load,HERE,FROZEN
import hashlib,json
import numpy as np
import casadi as ca
scene,higher,common,best,rollout=load()
expected=common.read(HERE/'results/v2/FROZEN_SHA256.json')
for name,sha in expected.items():assert hashlib.sha256((HERE/'results/v2'/name).read_bytes()).hexdigest()==sha
rows=[]
for level in scene.LEVELS:
    track,_=scene.build_case(scene.Case(level))
    for w in track.windows:
        t=ca.SX.sym('t');p,_=scene.pose(w,t);f=ca.Function('pose',[t],[p,ca.jacobian(p,t)])
        for at in np.linspace(0,2/w.motion.frequency,31):
            p,v=f(at);c,_,velocity,_=w.state_at(at)
            np.testing.assert_allclose(np.array(p).ravel(),c,atol=1e-12)
            np.testing.assert_allclose(np.array(v).ravel(),velocity,atol=1e-11)
            lo,hi=w.motion.local_bounds(at,at+.03)
            values=w.motion.local(np.linspace(at,at+.03,101))[0]
            assert np.all(values>=lo-1e-12) and np.all(values<=hi+1e-12)
    rows.append(dict(level=level,passed=True))
# The four methods must start from the same curves, with analytic TOGT gradients.
from convex_timevarying_window.comparisons.论文实验.实验2.matched_full_matrix.togt import matched_objective,initial_vector
track,_=scene.build_case(scene.Case(scene.LEVELS[0]));pl=higher.make_planner(track,True);cl=higher.make_planner(track,False)
a,b=pl.seeds()[0],cl.seeds()[0]
np.testing.assert_allclose(pl.curve_values(a),cl.curve_values(b),atol=1e-8)
obj=matched_objective(track,False,pl);x=initial_vector(obj,pl,0);value,g=obj.value_and_gradient(x)
errors=[]
for index in range(8):
    step=1e-6;dx=np.zeros_like(x);dx[index]=step
    fd=(obj.value_and_gradient(x+dx)[0]-obj.value_and_gradient(x-dx)[0])/(2*step)
    errors.append(abs(fd-g[index])/max(1,abs(fd)))
assert max(errors)<1e-4,errors
result=dict(passed=True,tracks=rows,symbolic_numerical_motion_and_velocity=True,analytic_bounds_checked=True,initial_curve_equality=True,togt_time_gradient_relative_errors=errors,frozen_algorithms=str(FROZEN))
(HERE/'results/v2/PREFLIGHT.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
