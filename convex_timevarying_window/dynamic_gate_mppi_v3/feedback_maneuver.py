"""Feedback around a admitted CTBR motion, not blind motor replay.

Domain limits are engineering checks, not a proved invariant tube.
"""
from dataclasses import dataclass
import numpy as np
from scipy.spatial.transform import Rotation
from .rollout import MASS,KF


def matrix(x):
    return Rotation.from_quat(np.r_[x[7:10],x[6]]).as_matrix()


@dataclass
class FeedbackManeuver:
    selection: object
    gate: int

    def target(self,now):
        s=self.selection;elapsed=now-s.made_at
        if elapsed<0 or now>=s.expires_at-1e-9:raise ValueError('maneuver_expired_or_not_active')
        f=elapsed/.01;i=min(int(f),len(s.states)-2);alpha=f-i
        x=(1-alpha)*s.states[i]+alpha*s.states[i+1]
        # Hemisphere alignment avoids interpolating q and -q through zero.
        qa=s.states[i,6:10];qb=s.states[i+1,6:10]
        if qa@qb<0:qb=-qb
        x[6:10]=(1-alpha)*qa+alpha*qb;x[6:10]/=np.linalg.norm(x[6:10])
        u=s.controls[min(int(elapsed/.02+1e-8),len(s.controls)-1)]
        return x,u

    def errors(self,actual,now):
        desired,_=self.target(now)
        return dict(position=float(np.linalg.norm(actual[:3]-desired[:3])),
                    velocity=float(np.linalg.norm(actual[3:6]-desired[3:6])),
                    attitude=float((Rotation.from_matrix(matrix(desired).T@matrix(actual))).magnitude()),
                    rate=float(np.linalg.norm(actual[10:13]-desired[10:13])))

    def within_domain(self,actual,now):
        if not np.isfinite(actual).all():return False
        try:e=self.errors(actual,now)
        except ValueError:return False
        return e['position']<=.35 and e['velocity']<=1. and e['attitude']<=np.deg2rad(20) and e['rate']<=1.5

    def command(self,actual,now):
        if not self.within_domain(actual,now):raise ValueError(f'maneuver_full_state_domain_exceeded: {self.errors(actual,now)}')
        desired,u=self.target(now);R=matrix(actual);Rd=matrix(desired)
        correction=2.*(desired[:3]-actual[:3])+2.8*(desired[3:6]-actual[3:6])
        force=Rd[:,2]*u[0]+MASS*correction
        thrust=float(force@R[:,2])
        E=Rd.T@R;err=.5*np.array([E[2,1]-E[1,2],E[0,2]-E[2,0],E[1,0]-E[0,1]])
        # Small force-direction correction expressed in current body axes.
        rate=R.T@Rd@u[1:]-5.*err+5.*np.cross(R.T@Rd[:,2],R.T@(MASS*correction))/max(u[0],1.)
        return np.r_[np.clip(thrust,4*KF*150**2,4*KF*1000**2),np.clip(rate,-3.,3.)]
