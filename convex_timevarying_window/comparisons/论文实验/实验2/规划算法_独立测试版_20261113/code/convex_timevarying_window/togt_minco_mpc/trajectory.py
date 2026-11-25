"""Degree-seven, globally coupled MINCO; never a Hermite interpolation.

Normalized coefficients satisfy the same PVAJ/C6 linear system as TOGT's
released MincoSnap. CasADi differentiates that solve for constrained online
optimization. Sampling and IPC keep all eight coefficients without fitting.
"""
from dataclasses import dataclass
import math
import casadi as ca
import numpy as np


def basis(s, derivative=0):
    values=[math.factorial(k)/math.factorial(k-derivative)*s**(k-derivative)
            if k >= derivative else 0. for k in range(8)]
    return ca.sparsify(ca.DM(values)) if isinstance(s,(int,float)) else ca.vertcat(*values)


def minco_coefficients(head, tail, points, durations):
    n = int(durations.numel())
    # Structural zeros matter: a dense zero constructor would make AD/codegen
    # differentiate thousands of identically-zero matrix entries.
    A, B = ca.MX(ca.Sparsity(8*n,8*n)), ca.MX(ca.Sparsity(8*n,3))
    for d in range(4):
        A[d,:8] = (basis(0.,d)/durations[0]**d).T
        B[d,:] = head[:,d].T
    for i in range(n-1):
        row, col = 8*i+4, 8*i
        for offset,d in enumerate((4,5,6)):
            A[row+offset,col:col+8] = (basis(1.,d)/durations[i]**d).T
            A[row+offset,col+8:col+16] = -(basis(0.,d)/durations[i+1]**d).T
        A[row+3,col:col+8] = basis(1.).T
        B[row+3,:] = points[:,i].T
        for offset,d in enumerate(range(4),4):
            A[row+offset,col:col+8] = (basis(1.,d)/durations[i]**d).T
            A[row+offset,col+8:col+16] = -(basis(0.,d)/durations[i+1]**d).T
    for d in range(4):
        A[8*n-4+d,-8:] = (basis(1.,d)/durations[-1]**d).T
        B[8*n-4+d,:] = tail[:,d].T
    return ca.solve(A,B,'qr')


@dataclass
class Plan:
    made_at: float
    first_gate: int
    durations: np.ndarray
    coefficients: np.ndarray
    x: np.ndarray
    solve_ms: float
    violation: float
    status: str
    gate_count: int
    clearance: float = -np.inf

    @property
    def end_time(self): return self.made_at+float(self.durations.sum())

    @property
    def crossing_times(self):
        return self.made_at+np.cumsum(self.durations)[1:2*self.gate_count:2]

    def sample_many(self,times,order=0):
        times=np.asarray(times,dtype=float)
        elapsed=np.maximum(0.,times-self.made_at)
        ends=np.cumsum(self.durations)
        ids=np.minimum(np.searchsorted(ends,elapsed,side='right'),len(ends)-1)
        out=np.empty((len(times),3))
        for i,T in enumerate(self.durations):
            mask=ids==i
            s=np.clip((elapsed[mask]-(ends[i-1] if i else 0.))/T,0.,1.)
            b=np.column_stack([math.factorial(k)/math.factorial(k-order)*s**(k-order)/T**order
                               if k>=order else np.zeros_like(s) for k in range(8)])
            out[mask]=b@self.coefficients[i]
        return out

    def sample(self,t,order=0):
        elapsed=max(0.,float(t)-self.made_at);ends=np.cumsum(self.durations)
        i=min(int(np.searchsorted(ends,elapsed,side='right')),len(ends)-1)
        T=self.durations[i];s=np.clip((elapsed-(ends[i-1] if i else 0.))/T,0.,1.)
        b=np.array([math.factorial(k)/math.factorial(k-order)*s**(k-order)/T**order if k>=order else 0. for k in range(8)])
        return b@self.coefficients[i]
