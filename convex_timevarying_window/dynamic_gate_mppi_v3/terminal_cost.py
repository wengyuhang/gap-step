"""Terminal velocity mismatch measured as equivalent correction distance."""


def velocity_correction_distance(x,indices,poses,signs,goal,acceleration,speed,xp,exit_depth):
    count=len(signs);ids=xp.minimum(indices,count-1)
    p=poses[-1,ids];n=p[:,[5,8,11]]*signs[ids,None]
    target=p[:,:3]+n*exit_depth
    done=indices>=count
    target=xp.where(done[:,None],xp.asarray(goal)[None],target)
    delta=target-x[:,:3];distance=xp.linalg.norm(delta,axis=1)
    direction=delta/xp.maximum(distance[:,None],1e-6)
    cruise=xp.where(done,xp.minimum(.75*speed,xp.sqrt(2*acceleration*distance)),.75*speed)
    desired=direction*cruise[:,None]
    return xp.sum((x[:,3:6]-desired)**2,axis=1)/(2*acceleration)
