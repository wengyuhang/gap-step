"""A closed, forward-facing seven-gate course with genuinely fixed planes."""
from contextlib import contextmanager
from dataclasses import asdict, dataclass
import importlib
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from convex_dynamic_seven_window_gazebo.x500_togt.experiment import build_track as original_track
from convex_timevarying_window.receding_gate_racing import scenario as transport
from convex_timevarying_window.comparisons.checked_soft_racing.run import save
from nonconvex_timevarying_window.sc_dynatogt.environment import MotionProfile

LEGACY_WORLD_WRITER = transport.write_variant_world
CENTERS = np.array([[-6,4,4],[10,4,4],[20,-7,4],[10,-20,4],[-6,-20,4],[-20,-11,4],[-20,0,4]],float)
YAWS = np.array([math.pi/2,math.pi/2,0.,-math.pi/2,-math.pi/2,math.pi,math.pi])
NORMALS = np.array([[1,0,0],[1,0,0],[0,-1,0],[-1,0,0],[-1,0,0],[0,1,0],[0,1,0]],float)
MOTION_AXES = (1,1,0,1,1,0,0)


@dataclass(frozen=True)
class Case:
    frequency_hz: float
    amplitude_m: float
    phase: float

    def __post_init__(self):
        if not np.isfinite([self.frequency_hz,self.amplitude_m,self.phase]).all():
            raise ValueError('Non-finite scenario parameter')
        if self.frequency_hz <= 0 or self.amplitude_m < 0:
            raise ValueError('Positive frequency and nonnegative amplitude required')


def build_case(case):
    track,config = original_track()
    for i,w in enumerate(track.windows):
        axis = MOTION_AXES[i]
        amplitude = np.zeros(3);amplitude[axis] = case.amplitude_m
        w.center0 = CENTERS[i].copy()
        w.angles0_rpy = np.array([math.pi/2,0.,YAWS[i]])
        w.motion = MotionProfile(amplitude,np.zeros(3),0.,
            translation_period=1/case.frequency_hz,rotation_period=1.,scale_period=1.,
            phase=case.phase+.9*i-(.7 if axis==1 else 0.),
            rotation_enabled=False,scale_enabled=False)
    object.__setattr__(track,'name','fixed_plane_horizontal_seven_closed_x500')
    return track,config


def neutral_vary(track, phase=0., layout=0., motion_rate=1., translation_scale=1., rotation_scale=1.):
    if not np.allclose([phase,layout,motion_rate,translation_scale,rotation_scale],[0,0,1,1,1],atol=0,rtol=0):
        raise ValueError('The fixed-plane adapter takes absolute f/A/phase; legacy scaling must stay neutral')
    return track


def description(case):
    track,_ = build_case(case)
    return dict(case=asdict(case),track=track.name,start=track.start.tolist(),goal=track.goal.tolist(),
        motion='c0 + A*horizontal_in_plane_axis*sin(2*pi*f*t + phase + 0.9*i)',
        fixed_plane=True,rotation_amplitude=0.,motion_epoch_sim_s=30.,
        center_peak_speed_mps=2*np.pi*case.frequency_hz*case.amplitude_m,
        center_peak_acceleration_mps2=(2*np.pi*case.frequency_hz)**2*case.amplitude_m,
        windows=[dict(name=w.name,center0=w.center0.tolist(),base_rpy=w.angles0_rpy.tolist(),
            normal=NORMALS[i].tolist(),motion_axis=MOTION_AXES[i],
            translation_amplitude=w.motion.translation_amplitude.tolist(),
            translation_period=w.motion.translation_period,plugin_phase=w.motion.phase)
            for i,w in enumerate(track.windows)])


def write_world(track,assets,outdir):
    path = LEGACY_WORLD_WRITER(track,assets,outdir)
    tree = ET.parse(path)
    models = [m for m in tree.findall('.//world/model') if m.get('name','').startswith('gate_')]
    for model,w in zip(models,track.windows):
        plugin = model.find("plugin[@name='convex_dynamic_course::PeriodicGateMotion']")
        # The old writer only varied centers/amplitudes, assuming old orientations.
        plugin.find('base_rpy').text = ' '.join(map(str,w.angles0_rpy))
        plugin.find('motion_start_time').text = '30'
    tree.write(path,encoding='unicode')
    check_world(track,path)
    return path


def check_world(track,path):
    """Independently evaluate the actual plugin XML formula against the planner."""
    models = [m for m in ET.parse(path).findall('.//world/model') if m.get('name','').startswith('gate_')]
    if len(models)!=len(track.windows):
        raise ValueError('Incorrect physical gate count')
    from nonconvex_timevarying_window.sc_dynatogt.environment import rotation_and_derivative
    max_position,max_rotation = 0.,0.
    for i,(model,w) in enumerate(zip(models,track.windows)):
        node = model.find("plugin[@name='convex_dynamic_course::PeriodicGateMotion']")
        vec = lambda key: np.fromstring(node.findtext(key),sep=' ')
        c0,base,amp,rot = [vec(key) for key in ('center0','base_rpy','translation_amplitude','rotation_amplitude')]
        period = float(node.findtext('translation_period'));phase = float(node.findtext('phase'))
        if np.any(rot) or abs(float(node.findtext('motion_start_time'))-30)>1e-12:
            raise ValueError('World is not a zero-rotation, aligned-clock scenario')
        for t in (0.,.137,.831,2.419,11.537):
            c = c0+amp*np.sin(2*np.pi*t/period+phase+np.array([0.,.7,1.4]))
            R,_ = rotation_and_derivative(base,np.zeros(3))
            cp,Rp,_,_ = w.state_at(t)
            max_position = max(max_position,float(np.max(np.abs(c-cp))))
            max_rotation = max(max_rotation,float(np.max(np.abs(R-Rp))))
            if abs(NORMALS[i]@(c-c0))>1e-10:
                raise ValueError('Window center leaves its fixed plane')
    if max(max_position,max_rotation)>1e-10:
        raise ValueError('Gazebo plugin/planner scene disagreement')
    return dict(max_position_error_m=max_position,max_rotation_matrix_error=max_rotation,
                fixed_plane=True,evidence='Serialized plugin formula parity, not live Gazebo telemetry')


def launch(track,assets,outdir):
    old = transport.write_variant_world
    transport.write_variant_world = write_world
    try:
        return transport.launch_variant(track,assets,outdir)
    finally:
        transport.write_variant_world = old


@contextmanager
def audit_scene(case):
    """Explicit per-run scene injection into unchanged independent auditors."""
    changed = []
    factory = lambda: build_case(case)
    bindings = [
        ('convex_dynamic_seven_window_gazebo.x500_togt.experiment','build_track',factory),
        ('convex_timevarying_window.receding_gate_racing.scenario','vary_track',neutral_vary),
        ('convex_timevarying_window.receding_gate_racing.analyze','build_track',factory),
        ('convex_timevarying_window.receding_gate_racing.analyze','vary_track',neutral_vary),
        ('convex_timevarying_window.receding_gate_racing.localization_audit','build_track',factory),
        ('convex_timevarying_window.receding_gate_racing.localization_audit','vary_track',neutral_vary),
        ('convex_timevarying_window.comparisons.checked_soft_racing.adjudicate','build_track',factory),
    ]
    try:
        for name,attribute,value in bindings:
            module = importlib.import_module(name)
            changed.append((module,attribute,getattr(module,attribute)))
            setattr(module,attribute,value)
        yield
    finally:
        for module,attribute,value in reversed(changed):
            setattr(module,attribute,value)
