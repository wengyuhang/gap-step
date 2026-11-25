"""Three-piece joint extension of hard_v2; two free interior P/V knots."""
import tempfile
import time
from types import SimpleNamespace
import casadi as ca
import numpy as np
from scipy.optimize import minimize
from convex_timevarying_window.dynamic_passage_velocity_minco.hard_planner import Config,Planner as Single
from convex_timevarying_window.dynamic_passage_velocity_minco.hard_dynamics import constraints
from convex_timevarying_window.dynamic_passage_velocity_minco.trajectory import Plan
from convex_timevarying_window.dynamic_passage_minco.planner import Stats,BudgetExpired
from convex_timevarying_window.dynamic_passage_minco.safety import passage,power_to_bernstein
from convex_timevarying_window.togt_minco_mpc.trajectory import basis
from convex_timevarying_window.comparisons.original_fixed_plane_ours.adapter import symbolic_pose
from nonconvex_timevarying_window.sc_dynatogt.environment import SCWindowTrack


def coefficients(head,tail,points,velocities,T):
    """Globally coupled minimum snap with C5 and prescribed interior P/V."""
    n=int(T.numel());A=ca.MX.zeros(8*n,8*n);B=ca.MX.zeros(8*n,3);row=0
    for d in range(4):
        A[row,:8]=(basis(0.,d)/T[0]**d).T;B[row,:]=head[:,d].T;row+=1
    for k in range(n-1):
        for d in range(6):
            A[row,8*k:8*k+8]=(basis(1.,d)/T[k]**d).T
            A[row,8*k+8:8*k+16]=-(basis(0.,d)/T[k+1]**d).T;row+=1
        A[row,8*k:8*k+8]=basis(1.).T;B[row,:]=points[:,k].T;row+=1
        A[row,8*k:8*k+8]=(basis(1.,1)/T[k]).T;B[row,:]=velocities[:,k].T;row+=1
    for d in range(4):
        A[row,-8:]=(basis(1.,d)/T[-1]**d).T;B[row,:]=tail[:,d].T;row+=1
    return ca.solve(A,B,'qr')


def route_velocity_seeds(start,centers,goal,speed):
    """World-frame route tangents; initialization only, not constraints."""
    nodes=np.vstack([start,centers,goal])
    edges=np.diff(nodes,axis=0)
    directions=edges/np.maximum(np.linalg.norm(edges,axis=1,keepdims=True),1e-12)
    tangent=directions[:-1]+directions[1:]
    lengths=np.linalg.norm(tangent,axis=1,keepdims=True)
    tangent=np.where(lengths>1e-10,tangent/np.maximum(lengths,1e-12),directions[1:])
    return (speed*tangent).ravel()


class Planner:
    def __init__(self,track,cfg):
        if len(track.windows)!=2:raise ValueError('joint test requires exactly two gates')
        self.track=track;self.cfg=cfg;self.windows=list(track.windows);self.admitted=None
        self.rotations=[w.state_at(0.)[1] for w in self.windows]
        self.signs=[float(np.sign((track.start-w.center0)@R[:,2])) for w,R in zip(self.windows,self.rotations)]
        self._jit_dir=tempfile.TemporaryDirectory(prefix='two_gate_joint_jit_')
        self._build()
        self.last=Single(SCWindowTrack('remaining_gate',track.start,track.goal,(track.windows[1],),(0,)),cfg)
        self.solvers=[(Stats(),),self.last.solvers[0],self.last.solvers[1]]

    def _build(self):
        cfg=self.cfg;S=cfg.witness_slots
        x=ca.MX.sym('joint_x',13);p=ca.MX.sym('joint_p',13+4*S)
        T=ca.exp(x[:3]);cross=[p[0]+T[0],p[0]+T[0]+T[1]]
        poses=[symbolic_pose(w,t) for w,t in zip(self.windows,cross)]
        points=ca.horzcat(*[c+R[:,:2]@x[3+2*k:5+2*k] for k,(c,R) in enumerate(poses)])
        V=ca.reshape(x[7:13],3,2)
        C=coefficients(ca.reshape(p[1:13],3,4),ca.horzcat(ca.DM(self.track.goal),ca.DM.zeros(3,3)),points,V,T)
        def state(k,u,d=0):return C[8*k:8*k+8,:].T@basis(u,d)/T[k]**d
        g=constraints(state,T,cfg.nodes_per_piece,cfg.workspace_lower,cfg.workspace_upper)
        radius=cfg.radius+cfg.frame_thickness+cfg.passage_margin+cfg.witness_slack
        for k,(w,(c,R)) in enumerate(zip(self.windows,poses)):
            half=np.max(np.abs(w.aperture.physical_vertices),axis=0)+cfg.frame_offset
            for side in range(2):
                piece=k+side
                for j in range(S):
                    offset=ca.fmin(p[13+(2*k+side)*S+j],T[piece])
                    u=1-offset/T[piece] if side==0 else offset/T[piece]
                    t=cross[k]+(-offset if side==0 else offset)
                    cc,rr=symbolic_pose(w,t);q=rr.T@(state(piece,u)-cc)
                    for axis in (0,1):
                        for sign in (-1,1):
                            inside=ca.fmax(half[axis]-sign*q[axis],0.)
                            g.append(inside**2+q[2]**2-radius**2)
            normal=-self.signs[k]*self.rotations[k][:,2]
            g.append(ca.dot(ca.DM(normal),V[:,k])-.1)
            # Same exit-side position hull as hard_v2, for every later piece.
            for piece in range(k+1,3):
                power=C[8*piece:8*piece+8,:]@ca.DM(normal);power[0]-=w.center0@normal
                controls=ca.DM(power_to_bernstein(np.eye(8)))@power
                g.extend(ca.vertsplit(controls[1:]/10.))
            # Non-adjacent earlier pieces remain outside the upcoming plane.
            for piece in range(k):
                power=C[8*piece:8*piece+8,:]@ca.DM(normal);power[0]-=w.center0@normal
                controls=ca.DM(power_to_bernstein(np.eye(8)))@power
                g.extend(ca.vertsplit(-controls/10.))
        G=ca.vertcat(*g);cost=ca.sum1(T)/1000
        opts={'cse':True}
        if cfg.compile_objective:opts.update(jit=True,compiler='shell',jit_options={
            'flags':[cfg.compiler_optimization],'directory':self._jit_dir.name+'/'})
        self.function=ca.Function('two_joint_eval',[x,p],[cost,ca.gradient(cost,x),G,ca.jacobian(G,x),C],opts)
        half=np.concatenate([np.max(np.abs(w.aperture.physical_vertices),axis=0) for w in self.windows])
        self.lower=np.r_[np.log([.3]*3),-half if cfg.free_points else np.zeros(4),[-np.inf]*6]
        self.upper=np.r_[np.log([15.]*3),half if cfg.free_points else np.zeros(4),[np.inf]*6]

    def prefix_clearance(self,plan,at):return None
    def commit(self,plan):
        self.admitted=plan
        if plan is not None and plan.first_gate==1:self.last.commit(plan)

    def solve(self,state,now,first):
        if first:
            plan=self.last.solve(state,now,first-1);plan.first_gate=first;return plan
        began=time.perf_counter();deadline=began+self.cfg.max_solve_ms/1000
        state=np.asarray(state)
        if len(state)==9:state=np.r_[state,np.zeros(3)]
        cfg=self.cfg;S=cfg.witness_slots
        base=np.r_[cfg.passage_seed_offsets,np.zeros(S-len(cfg.passage_seed_offsets))]
        seeds=[];dist=np.linalg.norm(self.windows[0].center0-state[:3])
        first_time=max(.8,dist/6.5+.5)
        between=max(.8,np.linalg.norm(self.windows[1].center0-self.windows[0].center0)/6.5)
        distance2=np.linalg.norm(self.track.goal-self.windows[1].center0)
        tail=max(.8,np.sqrt(6*distance2/cfg.acceleration),(60*distance2/cfg.jerk)**(1/3))*cfg.initial_tail_scale
        arrivals=[]
        for k,w in enumerate(self.windows):
            omega=2*np.pi/w.motion.translation_period
            estimate=now+first_time+(between if k else 0.)
            turn=(np.pi/2+round((omega*estimate+w.motion.phase-np.pi/2)/np.pi)*np.pi-w.motion.phase)/omega
            arrivals.append([turn+j*np.pi/omega for j in (0,-1,-2,1)])
        for a,b in zip(*arrivals):
            if a>now+.3 and b>a+.3:
                centers=[w.state_at(t)[0] for w,t in zip(self.windows,(a,b))]
                velocities=route_velocity_seeds(state[:3],centers,self.track.goal,.8*cfg.speed)
                seeds.append(np.r_[np.log([a-now,b-a,tail]),np.zeros(4),velocities])
        if self.admitted is not None and self.admitted.first_gate==0 and len(self.admitted.x)==13:
            seeds.insert(0,self.admitted.x.copy())
        offsets=np.tile(base,(4,1));par=np.r_[now,state,offsets.ravel()]
        seeds.sort(key=lambda x:(float(np.max(np.maximum(-np.asarray(self.function(x,par)[2]),0.))),float(self.function(x,par)[0])))
        attempts=[];nit=0
        for seed in seeds:
            if time.perf_counter()>=deadline:break
            x=np.clip(seed,self.lower,self.upper);offsets=np.tile(base,(4,1));next_slot=[len(cfg.passage_seed_offsets)]*4
            basin_lower=self.lower.copy();basin_upper=self.upper.copy()
            omega0=2*np.pi/self.windows[0].motion.translation_period
            turn0=now+np.exp(seed[0])
            basin_lower[0]=np.log(max(.3,turn0-np.pi/(2*omega0)-now))
            basin_upper[0]=np.log(max(.3,min(15.,turn0+np.pi/(2*omega0)-now)))
            turn1=now+np.exp(seed[0])+np.exp(seed[1])
            half_basin=np.pi/(2*(2*np.pi/self.windows[1].motion.translation_period))
            for outer in range(cfg.exchange_rounds):
                if time.perf_counter()>=deadline:break
                par=np.r_[now,state,offsets.ravel()];cache=None;value=None;last=x.copy();origin=x.copy()
                scale=np.r_[1/max(1.,2*np.pi/self.windows[0].motion.translation_period*np.exp(x[0])),
                    1/max(1.,2*np.pi/self.windows[1].motion.translation_period*np.exp(x[1])),1.,np.ones(4),np.full(6,cfg.speed)]
                def evaluate(z):
                    nonlocal cache,value
                    if time.perf_counter()>=deadline:raise BudgetExpired()
                    if cache is None or not np.array_equal(z,cache):cache=z.copy();value=self.function(z,par)
                    return value
                def objective(y):
                    out=evaluate(origin+scale*y);v=1000*float(out[0])
                    return np.log1p(v),np.asarray(out[1]).ravel()*scale*1000/(1+v)
                def region(y):
                    z=origin+scale*y;arrival=now+np.exp(z[0])+np.exp(z[1])
                    return np.array([arrival-turn1+half_basin,turn1+half_basin-arrival])
                def region_jac(y):
                    z=origin+scale*y;gradient=np.r_[np.exp(z[:2])*scale[:2],np.zeros(11)]
                    return np.array([gradient,-gradient])
                def callback(y):
                    nonlocal last
                    last=origin+scale*y
                    if time.perf_counter()>=deadline:raise BudgetExpired()
                try:
                    result=minimize(objective,np.zeros(13),jac=True,method='SLSQP',
                        bounds=list(zip((basin_lower-origin)/scale,(basin_upper-origin)/scale)),
                        constraints=[{'type':'ineq','fun':lambda y:np.asarray(evaluate(origin+scale*y)[2]).ravel(),
                                     'jac':lambda y:np.asarray(evaluate(origin+scale*y)[3])*scale},
                                     {'type':'ineq','fun':region,'jac':region_jac}],
                        callback=callback,options={'maxiter':cfg.constrained_iterations,'ftol':1e-7})
                    x=origin+scale*result.x;nit+=result.nit
                except BudgetExpired:break
                out=self.function(x,par)
                plan=Plan(now,0,np.exp(x[:3]),np.asarray(out[4]).reshape(3,8,3),x.copy(),0.,0.,'joint_two_gate',2)
                parts=[(k,side,passage(plan,w,cfg.radius+cfg.frame_thickness+cfg.passage_margin,
                                      cfg.frame_offset,piece_indices=(k+side,)))
                       for k,w in enumerate(self.windows) for side in range(2)]
                passed=result.success and np.min(np.asarray(out[2]))>=-1e-5 and all(p['passed'] for _,_,p in parts)
                attempts.append(dict(status=str(result.message),passed=passed,objective_s=float(np.exp(x[:3]).sum())))
                if passed:
                    elapsed=1000*(time.perf_counter()-began);plan.solve_ms=elapsed
                    stats=dict(iter_count=int(nit),optimization_ms=elapsed,converged=True,attempts=attempts,
                        return_status='joint_two_gate',formal_certificate=False)
                    self.solvers[0][0].value=stats;plan.optimizer_stats=stats
                    plan.event_valid=True;plan.event_reason='joint_two_gate_numerically_bounded';plan.terminal_gate=None
                    return plan
                added=False
                for k,side,p in parts:
                    slot=2*k+side
                    if p['passed'] or p['witness_u'] is None or next_slot[slot]>=S:continue
                    offset=((1-p['witness_u']) if side==0 else p['witness_u'])*plan.durations[k+side]
                    offsets[slot,next_slot[slot]]=offset;next_slot[slot]+=1;added=True
                if not added:break
        raise RuntimeError(f'no_joint_passage_under_budget: {1000*(time.perf_counter()-began)}')


def serve(track,cfg,requests,results,ready):
    from convex_timevarying_window.wang_free_state_mpc import worker
    worker.Planner=Planner;worker._serve(track,cfg,requests,results,ready)


from convex_timevarying_window.dynamic_passage_velocity_minco import repaired_worker


class Worker(repaired_worker.Worker):
    def __init__(self,track,cfg):
        original=repaired_worker.serve;repaired_worker.serve=serve
        try:super().__init__(track,cfg)
        finally:repaired_worker.serve=original
        self.geometry=SimpleNamespace(cfg=cfg,windows=list(track.windows),signs=self.startup['signs'])
