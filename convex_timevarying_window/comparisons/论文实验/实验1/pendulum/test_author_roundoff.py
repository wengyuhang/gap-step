import numpy as np
import pytest
from .high_mpc_adapter import project_solver_roundoff
from convex_timevarying_window.togt_style_mpc.model import FMAX,FMIN

def test_ipopt_roundoff_maps_into_same_physical_input_box():
    command,delta=project_solver_roundoff([4*FMAX+3.3e-7,-3.800000037,0,0])
    assert command[0]==4*FMAX and command[1]==-3.8
    assert 0<delta<1e-5
    command,delta=project_solver_roundoff([4*FMIN-1e-9,0,0,0])
    assert command[0]==4*FMIN

def test_actual_bound_violation_is_not_silently_projected():
    with pytest.raises(ValueError,match='beyond numerical tolerance'):
        project_solver_roundoff([4*FMAX+.1,0,0,0])
    with pytest.raises(ValueError):project_solver_roundoff([np.nan,0,0,0])
