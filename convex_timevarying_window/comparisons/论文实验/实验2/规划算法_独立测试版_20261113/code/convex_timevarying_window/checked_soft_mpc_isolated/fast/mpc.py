"""Exact state elimination of the SAME linearized MPC QP.

No changed cost, horizon, model, input bounds, tilt bound, trust region, QP
tolerances or nonlinear merit acceptance. Analytic elimination reduces QP
variables from 542 to 100 (25 inputs x 4); 442 dynamics equalities disappear.
"""
import time
import numpy as np
import osqp
from scipy import sparse
from convex_timevarying_window.checked_soft_mpc_racing.mpc import (
    MPC as BaseMPC, Config, NX, NU, LOWER, UPPER, hover_state, reference_states)


class MPC(BaseMPC):
    def __init__(self, cfg=Config()):
        super().__init__(cfg)
        n = cfg.steps
        self._accumulate = self.step.mapaccum(n)
        self._w = self.qdiag*np.tile(self.xscale**2,n+1)
        U = sparse.diags(np.tile(self.uscale,n))
        self._pu = (U@(sparse.diags(np.tile(self.R,n))+self.D.T@self.SD@self.D)@U).toarray()
        self._upper = np.triu_indices(n*NU)
        self._ar = np.r_[np.arange(n*NU), np.repeat(n*NU+np.arange(n),n*NU)]
        self._ac = np.r_[np.arange(n*NU), np.tile(np.arange(n*NU),n)]

    def rollout(self, x, controls):
        return np.vstack((x,np.asarray(self._accumulate(x,controls.T,self.cfg.dt)).T))

    def affine_prediction(self, x, states, controls):
        """All scaled predicted states equal offset + mapping @ scaled U."""
        n = self.cfg.steps
        f, aa, bb = self.linear(states[:-1].T,controls.T,self.cfg.dt)
        aa=np.asarray(aa).reshape(NX,n,NX).transpose(1,0,2)
        bb=np.asarray(bb).reshape(NX,n,NU).transpose(1,0,2)
        defect=np.asarray(f).T-np.einsum('kij,kj->ki',aa,states[:-1])-np.einsum('kij,kj->ki',bb,controls)
        offsets=np.empty((n+1,NX));offsets[0]=x/self.xscale
        maps=np.zeros((n+1,NX,n*NU))
        for k,(A,B,d) in enumerate(zip(aa,bb,defect)):
            A=A*self.xscale[None,:]/self.xscale[:,None]
            B=B*self.uscale[None,:]/self.xscale[:,None]
            offsets[k+1]=A@offsets[k]+d/self.xscale
            maps[k+1]=A@maps[k]
            maps[k+1,:,k*NU:(k+1)*NU]+=B
        return offsets.ravel(),maps.reshape(self.xsize,n*NU)

    def condense(self, x, states, controls, refs, ff):
        n=self.cfg.steps
        offset,mapping=self.affine_prediction(x,states,controls)
        H=mapping.T@(self._w[:,None]*mapping)+self._pu
        H=(H+H.T)  # 2*sym(H), OSQP uses 1/2 U^T P U.
        P=sparse.csc_matrix((H[self._upper],self._upper),shape=(n*NU,n*NU))
        d0=np.zeros(n*NU);d0[:NU]=self.previous
        qu=-2*(np.tile(self.R,n)*ff.ravel()+self.D.T@(self.SD@d0))*np.tile(self.uscale,n)
        q=2*mapping.T@(self._w*(offset-refs.ravel()/np.tile(self.xscale,n+1)))+qu
        # Exactly the same quaternion-based linearized tilt inequalities.
        gains=-4*states[1:,7:9]
        rows=(np.arange(1,n+1)[:,None]*NX+np.array([7,8])).ravel()
        tilt_map=np.einsum('ki,kij->kj',gains,mapping[rows].reshape(n,2,n*NU))
        tilt_offset=np.sum(gains*offset[rows].reshape(n,2),axis=1)
        tilt_lower=np.cos(np.radians(self.cfg.tilt_limit_deg))-1-2*np.sum(states[1:,7:9]**2,axis=1)
        A=sparse.csc_matrix((np.r_[np.ones(n*NU),tilt_map.ravel()],(self._ar,self._ac)),shape=(n*NU+n,n*NU))
        trust=np.array([self.cfg.trust_thrust_n]+[self.cfg.trust_rate_rps]*3)
        lower=np.r_[(np.maximum(LOWER,controls-trust)/self.uscale).ravel(),tilt_lower-tilt_offset]
        upper=np.r_[(np.minimum(UPPER,controls+trust)/self.uscale).ravel(),np.full(n,np.inf)]
        return P,q,A,lower,upper

    def command(self, state, now, reference):
        """reference(times)->(N,9) physical PVA, NOT legacy preview PVA."""
        began = time.perf_counter(); cfg = self.cfg; n = cfg.steps
        if not np.isfinite(state).all(): raise ValueError('nonfinite MPC state')
        x = np.array(self.step(state, self.previous, cfg.state_prediction_s)).ravel()
        at = now+cfg.state_prediction_s
        pva = reference(at+np.arange(n+1)*cfg.dt)
        refs, ff = reference_states(pva, cfg.dt, x[6:10])
        if self.controls is None:
            controls = ff.copy()
        else:
            offset = max(0., (now-self.last_time)/cfg.dt)
            knots = np.arange(n)
            controls = np.column_stack([np.interp(knots+offset, knots, self.controls[:, j]) for j in range(NU)])
        status = None; qp_iters = 0; inexact = 0; worst_qp_residual = 0.
        for _ in range(cfg.iterations):
            states = self.rollout(x, controls)
            P, q, A, lower, upper = self.condense(x, states, controls, refs, ff)
            if self.solver is None:
                self.solver = osqp.OSQP()
                self.solver.setup(P=P, q=q, A=A, l=lower, u=upper, verbose=False,
                                  eps_abs=1e-5, eps_rel=1e-5, max_iter=cfg.qp_iterations,
                                  polish=False, warm_start=True, time_limit=.025)
            else: self.solver.update(Px=P.data, q=q, Ax=A.data, l=lower, u=upper)
            self.solver.warm_start(x=(controls/self.uscale).ravel())
            solution = self.solver.solve(); status = solution.info.status
            qp_iters += solution.info.iter
            # RTI does not require the QP's optimality flag. A time-limited
            # iterate may be usable, but only with an explicit primal residual
            # bound AND nonlinear descent below. Infeasible/invalid solutions
            # are never admitted this way. 0.002 is a NUMERICAL QP tolerance,
            # not a changed frame margin or a claimed safety certificate.
            residual = np.inf
            vector = None if solution.x is None else np.asarray(solution.x, dtype=float)
            finite = vector is not None and vector.shape == (n*NU,) and np.isfinite(vector).all()
            if finite:
                predicted = A@vector
                residual = float(max(np.max(lower-predicted), np.max(predicted-upper), 0.))
            partial_ok = status in ('maximum iterations reached', 'run time limit reached') and residual <= .002
            if not finite or (solution.info.status_val not in (1, 2) and not partial_ok):
                raise RuntimeError(f'MPC QP {status}; primal={solution.info.pri_res}')
            inexact += int(partial_ok); worst_qp_residual = max(worst_qp_residual, residual)
            proposal = np.clip(vector.reshape(n, NU)*self.uscale, LOWER, UPPER)
            best_cost = self.merit(states, controls, refs, ff)
            selected = controls
            # Globalize RTI: a good LINEAR prediction can be a bad nonlinear
            # rollout. Never accept a large QP step without checking its merit.
            for alpha in (1., .5, .25, .125):
                trial = controls+alpha*(proposal-controls)
                cost = self.merit(self.rollout(x, trial), trial, refs, ff)
                if cost < best_cost:
                    selected = trial; best_cost = cost
            controls = selected
        self.controls = controls; self.last_time = now; self.previous = controls[0].copy()
        elapsed = (time.perf_counter()-began)*1000
        self.logs.append(dict(time=float(now), cycle_ms=elapsed, qp_status=status, qp_iterations=qp_iters,
                              inexact_qp_steps=inexact, max_normalized_qp_primal_residual=worst_qp_residual,
                              position_error_m=float(np.linalg.norm(state[:3]-reference(np.array([now]))[0, :3]))))
        if elapsed >= 100.: raise RuntimeError(f'MPC deadline {elapsed:.3f} ms')
        return self.previous.copy()
