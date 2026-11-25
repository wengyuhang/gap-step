"""Gazebo-only flight-direction third-person chase view.

Run inside the Gazebo container, which supplies gz transport / message bindings.
Gazebo's follow and look-at offsets are body-relative.
No flight commands, PX4 interfaces, or simulator model poses are changed.
"""
import time
import math

OFFSET = (-8., -10., 6.)


def body_offset(q, offset=OFFSET):
    x, y, z, w = q
    scale = x*x+y*y+z*z+w*w
    if scale <= 1e-20:
        return offset
    s = 2./scale
    R = ((1-s*(y*y+z*z), s*(x*y-z*w), s*(x*z+y*w)),
         (s*(x*y+z*w), 1-s*(x*x+z*z), s*(y*z-x*w)),
         (s*(x*z-y*w), s*(y*z+x*w), 1-s*(x*x+y*y)))
    return tuple(sum(R[j][i]*offset[j] for j in range(3)) for i in range(3))


def main():
    from gz.transport13 import Node
    from gz.msgs10.cameratrack_pb2 import CameraTrack
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.msgs10.pose_pb2 import Pose
    from gz.msgs10.boolean_pb2 import Boolean
    from gz.msgs10.entity_factory_pb2 import EntityFactory
    from gz.msgs10.empty_pb2 import Empty
    from gz.msgs10.scene_pb2 import Scene
    node = Node()
    world = '/world/convex_seven_dynamic_px4_togt'
    target = 'flight_direction_camera_target'
    factory = EntityFactory()
    factory.sdf = ('<sdf version="1.9"><model name="'+target+'">'
                   '<static>false</static><link name="visual_only">'
                   '<gravity>false</gravity><kinematic>true</kinematic>'
                   '<visual name="invisible"><transparency>1</transparency>'
                   '<geometry><sphere><radius>0.001</radius></sphere></geometry>'
                   '</visual></link></model></sdf>')
    ok, reply = node.request(world+'/create', factory, EntityFactory, Boolean, 2000)
    if not ok or not reply.data:
        found, scene = node.request(world+'/scene/info', Empty(), Empty, Scene, 2000)
        if not found or not any(m.name == target for m in scene.model):
            raise RuntimeError('Could not create visual-only camera look-ahead target')
    publisher = node.advertise('/gui/track', CameraTrack)
    previous = [0.]
    last_position = [None]
    direction = [(1., 0., 0.)]

    def update(message):
        now = time.monotonic()
        if now-previous[0] < .04:
            return
        pose = next((p for p in message.pose if p.name == 'x500_0'), None)
        if pose is None:
            return
        previous[0] = now
        p = pose.position
        current = (p.x, p.y, p.z)
        stamp = message.header.stamp.sec + message.header.stamp.nsec*1e-9
        old = last_position[0]
        if old is not None and stamp > old[0]:
            # Use horizontal course direction: vertical takeoff must not put
            # the chase camera below the drone and point it into the sky.
            delta = (current[0]-old[1][0], current[1]-old[1][1], 0.)
            length = math.sqrt(sum(v*v for v in delta))
            if length/(stamp-old[0]) > .3:
                direction[0] = tuple(v/length for v in delta)
        last_position[0] = (stamp, current)
        forward = direction[0]
        look = Pose()
        look.name = target
        look.position.x, look.position.y, look.position.z = tuple(
            current[i] for i in range(3))
        look.orientation.w = 1.
        node.request(world+'/set_pose', look, Pose, Boolean, 100)
        m = CameraTrack()
        m.track_mode = CameraTrack.FOLLOW_LOOK_AT
        # Follow an identity-oriented, visual-only anchor. This avoids any
        # disagreement between transport and render airframe orientations.
        m.follow_target.name = target
        m.track_target.name = 'x500_0'
        offset = tuple(-6.*forward[i]+(2.5 if i == 2 else 0.) for i in range(3))
        m.follow_offset.x, m.follow_offset.y, m.follow_offset.z = offset
        m.follow_pgain = m.track_pgain = .35
        publisher.publish(m)

    node.subscribe(Pose_V, '/world/convex_seven_dynamic_px4_togt/dynamic_pose/info', update)
    print('THIRD_PERSON_FLIGHT_DIRECTION_CAMERA_READY: visual feedback only', flush=True)
    while True:
        time.sleep(1.)


if __name__ == '__main__':
    main()
