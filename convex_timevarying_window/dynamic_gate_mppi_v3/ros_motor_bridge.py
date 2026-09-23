"""ROS-only transport: timestamped commands arrive over loopback UDP.

No controller/hover fallback. Expired commands stop publication; this is not
a real-vehicle safety fallback. Used solely in the isolated Gazebo experiment.
"""
import json
import math
import socket
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from px4_msgs.msg import ActuatorMotors, OffboardControlMode


def main():
    rclpy.init()
    node = Node("v3_motor_bridge")
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE)
    motors = node.create_publisher(ActuatorMotors, "/fmu/in/actuator_motors", qos)
    mode = node.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", qos)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", 17884)); sock.setblocking(False)
    command = None
    print("V3 ROS motor bridge ready", flush=True)
    try:
        while rclpy.ok():
            tick = time.monotonic()
            while True:
                try: command = json.loads(sock.recv(4096))
                except BlockingIOError: break
            if command and tick <= command["expires_monotonic"]:
                values = command["controls"]
                if len(values) == 4 and all(math.isfinite(v) and 0 <= v <= 1 for v in values):
                    stamp = int(command["timestamp_us"])
                    m = OffboardControlMode(); m.timestamp = stamp; m.direct_actuator = True
                    a = ActuatorMotors(); a.timestamp = stamp; a.timestamp_sample = stamp
                    a.control = [*values, *([float("nan")]*8)]
                    mode.publish(m); motors.publish(a)
            rclpy.spin_once(node, timeout_sec=0.)
            time.sleep(max(0., .005-(time.monotonic()-tick)))
    finally:
        sock.close(); node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__": main()
