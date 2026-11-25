"""Estimate continuous estimator-position correction velocity from telemetry.

No simulator truth is consumed. This does not correct absolute localization
bias, and does not supply a certified state-error bound.
"""
import numpy as np


class PositionVelocityObserver:
    def __init__(self,tau=.08):
        self.tau=tau;self.previous=None;self.correction=np.zeros(3)

    def update(self,t,p,v):
        p=np.asarray(p);v=np.asarray(v)
        if self.previous is not None:
            old_t,old_p,old_v=self.previous;dt=t-old_t
            if dt<=0:return v+self.correction
            if dt<=.1:
                measured=(p-old_p)/dt-(v+old_v)/2
                self.correction+=(1-np.exp(-dt/self.tau))*(measured-self.correction)
            else:self.correction[:]=0.
        self.previous=(t,p.copy(),v.copy())
        return v+self.correction
