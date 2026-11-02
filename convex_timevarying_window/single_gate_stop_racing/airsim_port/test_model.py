import numpy as np
from .model import ENU_TO_NED, GenericQuad, enu_to_ned, ned_to_enu, allocation, dynamics, flat_map,G


def test_coordinate_map_is_proper_rotation_and_roundtrip():
    np.testing.assert_allclose(ENU_TO_NED@ENU_TO_NED.T, np.eye(3))
    assert np.linalg.det(ENU_TO_NED) == 1.
    origin = np.array([-16.,4.,-6.])
    np.testing.assert_allclose(enu_to_ned([-16.,4.,3.2], origin), [0.,0.,-9.2])
    p = np.array([10.,20.,7.])
    np.testing.assert_allclose(ned_to_enu(enu_to_ned(p,origin),origin),p)


def test_nominal_motor_and_inertia_values():
    p=GenericQuad()
    assert 4.17 < p.max_rotor_thrust < 4.19
    assert .055 < p.max_rotor_torque < .056
    assert np.all(p.inertia > 0.)
    assert 4*p.max_rotor_thrust > p.mass*9.81
    assert p.inertia[2] > p.inertia[0]


def test_planning_seeds_are_derived_from_native_vehicle():
    from .model import planning_seed_parameters
    model=GenericQuad(air_density=1.210699200630188)
    seeds=planning_seed_parameters(model)
    assert 20<seeds['speed']<23
    np.testing.assert_allclose(np.max(np.asarray(flat_map(model)([seeds['speed'],0,0],
        [0,0,0],[0,0,0],[0,0,0])[2])),.9,atol=1e-9)
    np.testing.assert_allclose(seeds['acceleration'],np.sqrt((allocation(model)[0].sum()/model.mass)**2-G**2))
    heavier=planning_seed_parameters(GenericQuad(mass=1.2))
    assert heavier['speed']<seeds['speed']


def test_native_hover_and_allocation_signs():
    model=GenericQuad();mix=allocation(model)
    hover=np.full(4,G/(4*model.max_rotor_thrust))
    np.testing.assert_allclose(mix@hover,[G,0,0,0],atol=1e-12)
    assert mix[1,0]<0 and mix[2,0]<0 and mix[3,0]<0
    q=np.array([np.sqrt(.5),0,0,np.sqrt(.5)])
    x=np.r_[[1,2,3],np.zeros(3),q,np.zeros(3),hover]
    np.testing.assert_allclose(np.asarray(dynamics()(x,hover,.04)).ravel(),x,atol=1e-10)
    _,w,u=flat_map()(np.zeros(3),np.zeros(3),np.zeros(3),np.zeros(3))
    np.testing.assert_allclose(np.asarray(w).ravel(),0,atol=1e-10)
    np.testing.assert_allclose(np.asarray(u).ravel(),hover,atol=1e-10)


def test_native_motor_response_and_rotation():
    model=GenericQuad();hover=np.full(4,G/(4*model.max_rotor_thrust))
    x=np.r_[[0,0,10],np.zeros(3),[1,0,0,0],np.zeros(3),hover]
    command=hover+np.array([.005,-.005,-.005,.005])
    after=np.asarray(dynamics()(x,command,.04)).ravel()
    assert after[10]<0 and abs(after[11])<1e-6
    np.testing.assert_allclose(after[13:],command+(hover-command)*np.exp(-.04/model.motor_tau),atol=1e-10)


def test_native_pose_conversion_and_preparation_reference():
    from .flight import physical_state,climb_reference
    from .scene import pose,vector
    origin=np.array([-16,4,-6])
    raw={'kinematics_estimated':dict(pose([0,0,-9.2]),linear_velocity=vector([1,2,3]),angular_velocity=vector([.1,.2,.3]))}
    state=physical_state(raw,origin)
    np.testing.assert_allclose(state[:3],[-16,4,3.2])
    np.testing.assert_allclose(state[3:6],[2,1,-3])
    np.testing.assert_allclose(state[10:13],[.1,-.2,-.3])
    ref=climb_reference(np.zeros(3),np.ones(3),7.)
    np.testing.assert_allclose(ref(np.array([0,7]),0),[np.zeros(3),np.ones(3)],atol=1e-10)
    for d in (1,2,3):np.testing.assert_allclose(ref(np.array([0,7,8]),d),0,atol=1e-10)
