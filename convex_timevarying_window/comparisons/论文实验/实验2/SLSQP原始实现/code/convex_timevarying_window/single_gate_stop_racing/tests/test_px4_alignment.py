import casadi as ca
import numpy as np
import pytest
from convex_timevarying_window.single_gate_stop_racing import px4_model as model
from convex_timevarying_window.single_gate_stop_racing.px4_mpc import MPC
from convex_timevarying_window.checked_soft_mpc_racing.mpc import hover_state, MASS, G, LOWER, UPPER


@pytest.fixture(scope='module')
def functions():
    return model.dynamics()


def test_normalized_allocation_hover_and_rotation_signs(functions):
    allocation = functions[3]
    assert np.allclose(np.asarray(allocation([.6, 0., 0., 0.])).ravel(), 660.)
    for axis in range(3):
        control = np.zeros(4); control[0] = .6; control[axis+1] = .01
        speeds = np.asarray(allocation(control)).ravel()
        physical_torque = model.MIXER@(model.KF*speeds**2)
        assert physical_torque[axis+1] > 0.


def test_allocator_uses_speed_not_force_and_reduces_at_saturation(functions):
    allocation = functions[3]
    speeds = np.asarray(allocation([1., .08, -.04, .03])).ravel()
    assert np.all(speeds <= 1000.) and np.all(speeds >= 150.)
    assert np.sum(model.KF*speeds**2) < 4*model.KF*1000**2
    # The real CA geometry is not the physical Gazebo mixer.
    assert not np.allclose(model.NORMALIZED_MIXER[:, :3], np.linalg.inv(model.MIXER)[:, 1:])


def test_hover_equilibrium_and_native_identical(functions):
    tick, step, linear, allocation = functions
    physical = hover_state(); x = model.initial_state(physical); u = [MASS*G, 0., 0., 0.]
    assert x.shape == (model.NX,)
    assert np.allclose(np.asarray(step(x, u, .04)).ravel(), x, atol=1e-10)
    for symbolic, native in zip(functions, model.native_dynamics()):
        args = ([.9, .02, -.05, .01],) if symbolic is allocation else (x, u, .004 if symbolic is tick else .04)
        sy, na = symbolic(*args), native(*args)
        sy = sy if isinstance(sy, tuple) else (sy,); na = na if isinstance(na, tuple) else (na,)
        for s, n in zip(sy, na): assert np.allclose(s, n, atol=1e-9)


def test_native_linearization_finite_difference(functions):
    x = model.initial_state(hover_state()); x[3:6] = [8., 3., -.5]
    x[17:20] = [.02, -.015, .005]; u = np.array([30., .2, -.1, .03])
    _, step, linear, _ = model.native_dynamics()
    y, A, B = linear(x, u, .04)
    for index in (0, 4, 10, 13, 17, 25, 29, 32):
        dx = np.zeros(model.NX); dx[index] = 1e-6
        numeric = (np.asarray(step(x+dx, u, .04))-np.asarray(step(x-dx, u, .04))).ravel()/2e-6
        assert np.allclose(numeric, np.asarray(A)[:, index], atol=2e-5, rtol=2e-4)
    for index in range(4):
        du = np.zeros(4); du[index] = 1e-6
        numeric = (np.asarray(step(x, u+du, .04))-np.asarray(step(x, u-du, .04))).ravel()/2e-6
        assert np.allclose(numeric, np.asarray(B)[:, index], atol=2e-5, rtol=2e-4)


def test_drag_has_physical_moment_not_only_linear_deceleration(functions):
    tick = functions[0]; x = model.initial_state(hover_state()); x[3] = 10.
    y = np.asarray(tick(x, [MASS*G, 0., 0., 0.], .004)).ravel()
    assert y[3] < x[3]
    assert abs(y[11]) > 1e-3  # drag at elevated rotors produces pitch moment


def test_tracking_objective_and_input_bounds_unchanged():
    m = MPC(); x = hover_state()
    ref = lambda ts: np.tile(np.r_[x[:3], np.zeros(6)], (len(ts), 1))
    u = m.command(x, 0., ref)
    assert np.all(u >= LOWER) and np.all(u <= UPPER)
    assert np.all(m.Q[17:] == 0.)
    assert np.all(m.Q[:3] == 2000.) and np.all(m.Q[3:6] == 30.)
    assert np.allclose(u, [MASS*G, 0., 0., 0.], atol=1e-5)
    controls = np.tile(u, (m.cfg.steps, 1)); initial = model.initial_state(x)
    states = m.rollout(initial, controls)
    assert np.allclose(states[:, :17], x, atol=1e-9)
    from convex_timevarying_window.checked_soft_mpc_racing.mpc import reference_states
    refs, ff = reference_states(ref(np.arange(m.cfg.steps+1)*m.cfg.dt), m.cfg.dt, x[6:10])
    controls[:, 1] = .02
    value, gradient = m.objective(controls.T, initial, refs.T, ff.T, u)
    delta = np.zeros_like(controls); delta[3, 1] = 1e-6
    plus = float(m.objective((controls+delta).T, initial, refs.T, ff.T, u)[0])
    minus = float(m.objective((controls-delta).T, initial, refs.T, ff.T, u)[0])
    assert np.isclose((plus-minus)/2e-6, np.asarray(gradient)[1, 3], atol=1e-4, rtol=2e-4)


def test_solve_budget_retains_evaluated_physical_controls():
    from dataclasses import replace
    m = MPC(); m.cfg = replace(m.cfg, max_solve_ms=0.)
    x = hover_state(); ref = lambda ts: np.tile(np.r_[x[:3], np.zeros(6)], (len(ts), 1))
    u = m.command(x, 0., ref)
    assert np.all(u >= LOWER) and np.all(u <= UPPER)
    assert m.logs[-1]['box_solves'][0]['status'] == 'online_solve_budget_best_evaluated_iterate'


def test_geographic_projection_mapping_is_not_track_fitted():
    from convex_timevarying_window.single_gate_stop_racing.metric_session import metric_scale,convert_position
    scale = metric_scale(47.397971057728974)
    assert np.allclose(scale, [1.00294074, .99985318, 1.], atol=5e-9)
    for latitude in (0., 30., 47., 80.):
        scale = metric_scale(latitude); anchor = np.array([-16., 4., 3.2])
        points = np.array([[0., 0., 1.], [100., -50., 10.]])
        converted = convert_position(points, anchor, scale)
        assert np.allclose(convert_position(converted, anchor, scale, inverse=True), points)
        assert np.array_equal(converted[:, 2], points[:, 2])
