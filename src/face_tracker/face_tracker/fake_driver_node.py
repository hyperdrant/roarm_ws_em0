import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import JointState


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


class FakeDriverNode(Node):
    """
    Subscribe: /gimbal/cmd (Vector3)  x=pan_deg, y=tilt_deg
    Publish:   /joint_states (JointState) for roarm_driver command input

    IMPORTANT:
    roarm_driver expects these 4 joints to exist in name[]:
      - base_link_to_link1
      - link1_to_link2
      - link2_to_link3
      - link3_to_gripper_link
    Otherwise it may crash with name.index(...) ValueError.
    """

    def __init__(self):
        super().__init__('fake_driver_node')

        # === Joint names from your /joint_states --once output ===
        self.declare_parameter('pan_joint', 'base_link_to_link1')
        self.declare_parameter('tilt_joint', 'link1_to_link2')

        # Two other joints (keep at 0 or hold last)
        self.declare_parameter('elbow_joint', 'link2_to_link3')
        self.declare_parameter('wrist_joint', 'link3_to_gripper_link')

        # Safety limits (deg)
        self.declare_parameter('pan_limit_deg', 120.0)
        self.declare_parameter('tilt_limit_deg', 120.0)

        # Publish rate
        self.declare_parameter('publish_hz', 15.0)

        # Direction inversion (if needed)
        self.declare_parameter('invert_pan', True)
        self.declare_parameter('invert_tilt', True)

        # If True, keep elbow/wrist at last sent values; otherwise always 0
        self.declare_parameter('hold_other_joints', False)

        self.pan_joint = str(self.get_parameter('pan_joint').value)
        self.tilt_joint = str(self.get_parameter('tilt_joint').value)
        self.elbow_joint = str(self.get_parameter('elbow_joint').value)
        self.wrist_joint = str(self.get_parameter('wrist_joint').value)

        self.pan_limit = float(self.get_parameter('pan_limit_deg').value)
        self.tilt_limit = float(self.get_parameter('tilt_limit_deg').value)
        self.publish_hz = float(self.get_parameter('publish_hz').value)

        self.invert_pan = bool(self.get_parameter('invert_pan').value)
        self.invert_tilt = bool(self.get_parameter('invert_tilt').value)
        self.hold_other = bool(self.get_parameter('hold_other_joints').value)

        # latest command (deg)
        self.pan_deg = 0.0
        self.tilt_deg = 0.0

        # other joints (rad)
        self.elbow_rad = 0.0
        self.wrist_rad = 0.0

        self.sub = self.create_subscription(Vector3, '/gimbal/cmd', self.on_cmd, 10)
        self.pub = self.create_publisher(JointState, '/joint_states', 10)

        self.declare_parameter('pitch_mode', 'shoulder')  # 'shoulder' or 'elbow'
        self.pitch_mode = str(self.get_parameter('pitch_mode').value)

        self.timer = self.create_timer(1.0 / self.publish_hz, self.publish_joint_cmd)

        self.get_logger().info(
            f"FakeDriverNode (REAL) started. Publish /joint_states at {self.publish_hz}Hz. "
            f"pan_joint={self.pan_joint}, tilt_joint={self.tilt_joint}"
        )

    def on_cmd(self, msg: Vector3):
        pan = clamp(float(msg.x), -self.pan_limit, self.pan_limit)
        tilt = clamp(float(msg.y), -self.tilt_limit, self.tilt_limit)

        if self.invert_pan:
            pan = -pan
        if self.invert_tilt:
            tilt = -tilt

        self.pan_deg = pan
        self.tilt_deg = tilt

    def publish_joint_cmd(self):
        pan_rad = self.pan_deg * math.pi / 180.0
        tilt_rad = self.tilt_deg * math.pi / 180.0

        shoulder = 0.0
        elbow = 0.0

        if self.pitch_mode == 'shoulder':
            shoulder = tilt_rad
            elbow = 0.0
        elif self.pitch_mode == 'elbow':
            shoulder = 0.0
            elbow = tilt_rad
        else:
            shoulder = tilt_rad
            elbow = 0.0  # 默认

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [self.pan_joint, self.tilt_joint, self.elbow_joint, self.wrist_joint]
        js.position = [pan_rad, shoulder, elbow, self.wrist_rad]
        self.pub.publish(js)



def main():
    rclpy.init()
    node = FakeDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
