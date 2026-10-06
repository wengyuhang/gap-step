"""Separate collective-thrust/body-rate path. Old motor model remains default."""
import ctypes
import numpy as np
from .rollout import Rollout,KF


def encode_ctbr(command):
    u=np.asarray(command,dtype=float)
    if u.shape!=(4,) or not np.isfinite(u).all():raise ValueError('invalid CTBR')
    if not 4*KF*150**2<=u[0]<=4*KF*1000**2 or np.any(np.abs(u[1:])>3.84):
        raise ValueError('CTBR out of actuator range')
    return u[1:]*[1,-1,-1],float((np.sqrt(u[0]/(4*KF))-150)/850)


class CTBRRollout(Rollout):
    ctbr=True

    def replay_feedback(self,state,policy,now,count,previous=None):
        desired,ff=policy.targets(now+np.arange(count)*.02)
        state=np.ascontiguousarray(state,dtype=float)
        previous=None if previous is None else np.ascontiguousarray(previous,dtype=float)
        commands=np.empty((count,4))
        fn=self.lib.v3_feedback_replay
        fn.argtypes=[ctypes.c_int]+[ctypes.c_void_p]*5;fn.restype=ctypes.c_int
        ok=fn(count,state.ctypes.data,desired.ctypes.data,ff.ctypes.data,
              None if previous is None else previous.ctypes.data,commands.ctypes.data)
        return commands if ok else None

    def feedback(self,state,position,velocity,acceleration):
        x=np.ascontiguousarray(state,dtype=float)
        pva=np.ascontiguousarray(np.r_[position,velocity,acceleration],dtype=float)
        if x.shape!=(17,) or pva.shape!=(9,) or not np.isfinite(np.r_[x,pva]).all():
            raise ValueError('invalid feedback state/reference')
        fn=self.lib.v3_feedback;fn.argtypes=[ctypes.c_void_p]*4;fn.restype=None
        out=np.empty(4)
        fn(x.ctypes.data,pva.ctypes.data,self.cfg.ctypes.data,out.ctypes.data)
        return out
