"""Pure trajectory-tracking native motor-PWM NMPC, no gate costs."""
import tempfile
import time
import casadi as ca
import numpy as np
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation
from .model import GenericQuad, G, dynamics, flat_map


class BudgetExpired(Exception):
    pass


class MPC:
    def __init__(self, steps=20, dt=.04, max_solve_ms=85., model=GenericQuad()):
        self.steps,self.dt,self.max_solve_ms=steps,dt,max_solve_ms
        self.model=model
        self.step=dynamics(model);self.flat=flat_map(model);self.logs=[]
        self.previous=np.full(4,model.mass*G/(4*model.max_rotor_thrust*model.air_density/model.density))
        self.controls=None;self.last_time=None
        self.motor=self.previous.copy();self.motor_time=None
        u=ca.MX.sym('u',4,steps);x=ca.MX.sym('x',17)
        ref=ca.MX.sym('ref',17,steps+1);ff=ca.MX.sym('ff',4,steps);prev=ca.MX.sym('prev',4)
        predicted=ca.horzcat(x,self.step.mapaccum(steps)(x,u,dt))
        cost=0.;Q=ca.DM(np.r_[np.full(3,2000.),np.full(3,30.),np.full(4,20.),np.full(3,.3),np.zeros(4)])
        for k in range(steps+1):
            cost+=(4 if k==steps else 1)*ca.dot(Q,(predicted[:,k]-ref[:,k])**2)
        for k in range(steps):
            du=u[:,k]-(prev if k==0 else u[:,k-1])
            cost+=.2*ca.sumsqr(u[:,k]-ff[:,k])+.5*ca.sumsqr(du)
        self.jit=tempfile.TemporaryDirectory(prefix='airsim_mpc_')
        self.objective=ca.Function('native_tracking',[u,x,ref,ff,prev],
            [cost,ca.gradient(cost,u)],dict(cse=True,jit=True,compiler='shell',
            jit_options=dict(flags=['-O2'],directory=self.jit.name+'/')))

    def references(self, reference, times, q0):
        derivatives=[reference(times,d) for d in range(5)]
        states=[];controls=[];last=q0
        for k in range(len(times)):
            R,w,pwm=self.flat(*[d[k] for d in derivatives[1:]])
            q=Rotation.from_matrix(np.asarray(R)).as_quat()[[3,0,1,2]]
            if q@last<0:q=-q
            last=q
            pwm=np.asarray(pwm).ravel()
            states.append(np.r_[derivatives[0][k],derivatives[1][k],q,np.asarray(w).ravel(),pwm])
            controls.append(np.clip(pwm,0.,1.))
        return np.array(states),np.array(controls[:-1])

    def command(self, physical, now, reference):
        began=time.perf_counter()
        if self.motor_time is not None:
            self.motor=self.previous+(self.motor-self.previous)*np.exp(-(now-self.motor_time)/self.model.motor_tau)
        x=np.r_[physical,self.motor];self.motor_time=now
        refs,ff=self.references(reference,now+np.arange(self.steps+1)*self.dt,x[6:10])
        if self.controls is None:controls=ff.copy()
        else:
            knots=np.arange(self.steps);offset=(now-self.last_time)/self.dt
            controls=np.column_stack([np.interp(knots+offset,knots,self.controls[:,j]) for j in range(4)])
        best=controls.copy();value_best=np.inf;evals=0
        def objective(z):
            nonlocal best,value_best,evals
            if evals and (time.perf_counter()-began)*1000>=self.max_solve_ms:
                raise BudgetExpired
            controls=z.reshape(self.steps,4)
            value,gradient=self.objective(controls.T,x,refs.T,ff.T,self.previous)
            value=float(value);evals+=1
            if np.isfinite(value) and value<value_best:value_best=value;best=controls.copy()
            return value,np.asarray(gradient).T.ravel()
        status='budget';iterations=None
        try:
            result=minimize(objective,controls.ravel(),jac=True,method='L-BFGS-B',
                bounds=[(0.,1.)]*(4*self.steps),options=dict(maxiter=50,maxls=12,maxcor=12,ftol=1e-8,gtol=1e-5))
            status=str(result.message);iterations=int(result.nit)
        except BudgetExpired:pass
        self.controls=best;self.previous=best[0].copy();self.last_time=now
        self.logs.append(dict(time=now,cycle_ms=1000*(time.perf_counter()-began),status=status,
            iterations=iterations,evaluations=evals,pwm=self.previous.tolist(),
            position_error_m=float(np.linalg.norm(physical[:3]-refs[0,:3]))))
        return self.previous.copy()
