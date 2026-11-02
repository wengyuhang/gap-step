"""Source-derived v1.8.1 GenericQuad parameters, NOT an identified MPC model."""
from dataclasses import dataclass
import numpy as np

ENU_TO_NED = np.array([[0., 1., 0.], [1., 0., 0.], [0., 0., -1.]])
LOCAL_TO_UNREAL = np.diag([1., 1., -1.])
FRD_TO_FLU = np.diag([1., -1., -1.])
G = 9.80665


@dataclass(frozen=True)
class GenericQuad:
    mass: float = 1.
    arm_length: float = .2275
    motor_mass: float = .055
    rotor_z: float = .025
    body_box: tuple = (.180, .110, .040)
    ct: float = .109919
    cp: float = .040164
    density: float = 1.225
    diameter: float = .2286
    max_rpm: float = 6396.667
    motor_tau: float = .005
    air_density: float = 1.225

    @property
    def max_rotor_thrust(self):
        return self.ct*self.density*(self.max_rpm/60.)**2*self.diameter**4

    @property
    def max_rotor_torque(self):
        return self.cp*self.density*(self.max_rpm/60.)**2*self.diameter**5/(2*np.pi)

    @property
    def inertia(self):
        x, y, z = self.body_box
        core = (self.mass-4*self.motor_mass)/12.*np.array([y*y+z*z,x*x+z*z,x*x+y*y])
        return core+self.motor_mass*np.array([
            2*self.arm_length**2+4*self.rotor_z**2,
            2*self.arm_length**2+4*self.rotor_z**2,
            4*self.arm_length**2])


def enu_to_ned(position, origin):
    return ENU_TO_NED@(np.asarray(position)-np.asarray(origin))


def ned_to_enu(position, origin):
    return ENU_TO_NED.T@np.asarray(position)+np.asarray(origin)


def allocation(model=GenericQuad()):
    """Force/torque from native FR,RL,FL,RR rotor PWM, expressed in FLU."""
    a = model.arm_length/np.sqrt(2)
    arms = np.array([[a,-a,-model.rotor_z],[-a,a,-model.rotor_z],
                     [a,a,-model.rotor_z],[-a,-a,-model.rotor_z]])
    return model.air_density/model.density*np.vstack((np.full(4,model.max_rotor_thrust),
        arms[:,1]*model.max_rotor_thrust, -arms[:,0]*model.max_rotor_thrust,
        np.array([-1.,-1.,1.,1.])*model.max_rotor_torque))


def drag_factors(model=GenericQuad()):
    x,y,z = model.body_box
    cross = np.pi*model.diameter*.01
    area = np.pi*model.diameter**2  # This is the actual AirSim formula, not pi*r².
    return model.air_density*.325/2*np.array([y*z+4*cross,x*z+4*cross,x*y+4*area])


def quaternion_rotation(q):
    import casadi as ca
    w,x,y,z = ca.vertsplit(q)
    return ca.vertcat(
        ca.horzcat(1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y)),
        ca.horzcat(2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x)),
        ca.horzcat(2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)))


def dynamics(model=GenericQuad()):
    """Native PWM model; FLU rigid body and exact first-order motor filter."""
    import casadi as ca
    x,u,h = ca.SX.sym('x',17),ca.SX.sym('u',4),ca.SX.sym('h')
    R = quaternion_rotation(x[6:10])
    vb = R.T@x[3:6]
    force = ca.DM(allocation(model))@x[13:17]
    acc = R@(ca.vertcat(0,0,force[0])-ca.DM(drag_factors(model))*vb*ca.fabs(vb))/model.mass-ca.DM([0,0,G])
    w,a,b,c = ca.vertsplit(x[6:10]);p,q,r = ca.vertsplit(x[10:13])
    qdot=.5*ca.vertcat(-a*p-b*q-c*r,w*p+b*r-c*q,w*q+c*p-a*r,w*r+a*q-b*p)
    J=ca.DM(model.inertia)
    angular=(force[1:]-ca.cross(x[10:13],J*x[10:13]))/J
    # Motor states are integrated analytically; mechanical states by RK4.
    f=ca.Function('native_mechanics',[x],[ca.vertcat(x[3:6],acc,qdot,angular,ca.SX.zeros(4))])
    y=x
    for _ in range(10):
        hh=h/10
        mid=u+(y[13:]-u)*ca.exp(-hh/(2*model.motor_tau))
        ym=ca.vertcat(y[:13],mid)
        k1=f(ym);k2=f(ym+hh*k1/2);k3=f(ym+hh*k2/2);k4=f(ym+hh*k3)
        yn=ym+hh*(k1+2*k2+2*k3+k4)/6
        y=ca.vertcat(yn[:6],yn[6:10]/ca.norm_2(yn[6:10]),yn[10:13],
                    u+(y[13:]-u)*ca.exp(-hh/model.motor_tau))
    return ca.Function('native_pwm_step',[x,u,h],[y])


def flat_map(model=GenericQuad()):
    """Constant-heading flatness approximation including native quadratic drag."""
    import casadi as ca
    v,a,j,s=[ca.SX.sym(n,3) for n in ('v','a','j','s')]
    force=model.mass*(a+ca.DM([0,0,G]))
    for _ in range(5):
        z=force/ca.sqrt(ca.sumsqr(force)+1e-12)
        y=ca.cross(z,ca.DM([0,1,0]));y/=ca.sqrt(ca.sumsqr(y)+1e-12)
        R=ca.horzcat(ca.cross(y,z),y,z)
        vb=R.T@v
        force=model.mass*(a+ca.DM([0,0,G]))+R@(ca.DM(drag_factors(model))*vb*ca.fabs(vb))
    z=force/ca.sqrt(ca.sumsqr(force)+1e-12)
    y=ca.cross(z,ca.DM([0,1,0]));y/=ca.sqrt(ca.sumsqr(y)+1e-12)
    R=ca.horzcat(ca.cross(y,z),y,z)
    rd=ca.reshape(ca.jtimes(ca.vec(R),ca.vertcat(v,a),ca.vertcat(a,j)),3,3)
    W=R.T@rd
    omega=ca.vertcat(W[2,1]-W[1,2],W[0,2]-W[2,0],W[1,0]-W[0,1])/2
    alpha=ca.jtimes(omega,ca.vertcat(v,a,j),ca.vertcat(a,j,s))
    J=ca.DM(model.inertia)
    torque=J*alpha+ca.cross(omega,J*omega)
    pwm=ca.DM(np.linalg.inv(allocation(model)))@ca.vertcat(ca.norm_2(force),torque)
    return ca.Function('native_flat_map',[v,a,j,s],[R,omega,pwm])


def planning_seed_parameters(model=GenericQuad()):
    """Vehicle-derived numerical seeds, not added physical constraints."""
    from scipy.optimize import brentq
    f=flat_map(model)
    speed=brentq(lambda v:float(np.max(np.asarray(f([v,0,0],[0,0,0],[0,0,0],[0,0,0])[2])))-.9,0.,40.)
    thrust=float(allocation(model)[0].sum())
    acceleration=np.sqrt(max(0.,(thrust/model.mass)**2-G**2))
    return dict(speed=speed,acceleration=acceleration,
        jerk=acceleration/model.motor_tau,initial_tail_scale=1.)


def body_envelope(client):
    """Conservative sphere around native rendered vehicle meshes, not a certificate.

    v1.8.1's mesh API allocates using byte count instead of vertex count;
    only index-referenced vertices are valid. Never include trailing garbage.
    """
    from scipy.spatial.transform import Rotation
    meshes=client.call('simGetMeshPositionVertexBuffers')
    entries=[];radius=0.
    for mesh in meshes:
        if mesh['name'].lower()!='racedrone' or not mesh['indices']:continue
        n=max(mesh['indices'])+1
        vertices=np.asarray(mesh['vertices'][:3*n]).reshape(-1,3)
        center=np.array([mesh['position'][k] for k in ('x_val','y_val','z_val')])
        q=mesh['orientation'];R=Rotation.from_quat([q[k] for k in ('x_val','y_val','z_val','w_val')]).as_matrix()
        local=(vertices-center)@R/100.
        extent=np.maximum(abs(local.min(0)),abs(local.max(0)))
        # Main body mesh encloses the smaller animated propeller meshes too.
        if n>5000:radius=max(radius,float(np.linalg.norm(extent)))
        entries.append(dict(vertices=n,min_m=local.min(0).tolist(),max_m=local.max(0).tolist()))
    if not .4<radius<1.5:raise RuntimeError('Native visual vehicle envelope not resolved')
    return radius+.01,dict(meshes=entries,radius_m=radius+.01,
        source='native visual root mesh bounding-box sphere plus 1cm; collision is independently queried',
        collision_shape_certificate=False)
