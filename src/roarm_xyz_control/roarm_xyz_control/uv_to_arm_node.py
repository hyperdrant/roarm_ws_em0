import json

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import String


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


class UvToArmNode(Node):
    def __init__(self):
        super().__init__('uv_to_arm_node')

        self.declare_parameter(
            'homography_file',
            '/home/hyperdrant/roarm_ws_em0/src/roarm_xyz_control/tools/homography_pixel_to_robot.npy'
        )

        self.declare_parameter('fixed_z', 0.0)
        self.declare_parameter('fixed_t', 3.14)

        self.declare_parameter('min_x', 100.0)
        self.declare_parameter('max_x', 500.0)
        self.declare_parameter('min_y', -250.0)
        self.declare_parameter('max_y', 250.0)

        self.declare_parameter('publish_only_when_detected', True)

        self.homography_file = str(self.get_parameter('homography_file').value)

        self.fixed_z = float(self.get_parameter('fixed_z').value)
        self.fixed_t = float(self.get_parameter('fixed_t').value)

        self.min_x = float(self.get_parameter('min_x').value)
        self.max_x = float(self.get_parameter('max_x').value)
        self.min_y = float(self.get_parameter('min_y').value)
        self.max_y = float(self.get_parameter('max_y').value)

        self.publish_only_when_detected = bool(
            self.get_parameter('publish_only_when_detected').value
        )

        self.H = np.load(self.homography_file)

        self.sub = self.create_subscription(
            Vector3,
            '/target/uv',
            self.on_uv,
            10
        )

        self.pub = self.create_publisher(String, '/arm/target_json', 10)

        self.get_logger().info('UvToArmNode started.')
        self.get_logger().info(f'Loaded homography: {self.homography_file}')
        self.get_logger().info('Mode: pixel uv -> robot xy using Homography.')

    def pixel_to_robot(self, u: float, v: float):
        pt = np.array([[[u, v]]], dtype=np.float32)
        out = cv2.perspectiveTransform(pt, self.H)
        x_robot = float(out[0, 0, 0])
        y_robot = float(out[0, 0, 1])
        return x_robot, y_robot

    def on_uv(self, msg: Vector3):
        valid = msg.z > 0.5

        if self.publish_only_when_detected and not valid:
            return

        u = float(msg.x)
        v = float(msg.y)

        x_robot, y_robot = self.pixel_to_robot(u, v)

        x_robot = clamp(x_robot, self.min_x, self.max_x)
        y_robot = clamp(y_robot, self.min_y, self.max_y)

        payload = {
            'x': round(x_robot, 3),
            'y': round(y_robot, 3),
            'z': round(self.fixed_z, 3),
            't': round(self.fixed_t, 3),
        }

        out = String()
        out.data = json.dumps(payload, separators=(',', ':'))
        self.pub.publish(out)

        self.get_logger().info(
            f'uv=({u:.1f},{v:.1f}), valid={valid} -> arm=({x_robot:.1f},{y_robot:.1f},{self.fixed_z:.1f})'
        )


def main(args=None):
    rclpy.init(args=args)
    node = UvToArmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()