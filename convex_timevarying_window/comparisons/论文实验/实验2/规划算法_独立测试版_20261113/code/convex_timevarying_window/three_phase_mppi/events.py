"""User-specified replanning events, separate from control phases."""
def reason(stage,passed=False,position_error=0.,velocity_error=0.,avoiding=False):
    if passed:return 'gate_exit'
    if stage!='TRANSIT':return None
    if avoiding:return 'obstacle_recovery'
    if position_error>.5 or velocity_error>2.:return 'tracking_error'
    return None

def computation_safe(metrics):
    """This is execution-prefix safety, NOT safety of the new TOGT path."""
    return bool(metrics[5]>=10 and metrics[6])
