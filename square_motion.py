#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


JOINTS = [
    "base_link_to_link1",
    "link1_to_link2",
    "link2_to_link3",
    "link3_to_gripper_link",
]


def deg(x: float) -> float:
    return x * math.pi / 180.0


def lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


class SquareMotion(Node):
    def __init__(self):
        super().__init__("square_motion")

        self.pub = self.create_publisher(JointState, "/joint_states", 10)

        # 运动参数：安全起见先小一点
        self.step_hz = 30.0          # 发布频率
        self.seg_time = 2.0          # 每条边用时（秒）
        self.corner_pause = 0.3      # 每个角停一下（秒）

        # “四个角”的关节角（单位：rad）
        # 你可以理解为：用 base 做左右，用 shoulder 做上下，形成一个“框”
        # base: 左(-) / 右(+)
        # shoulder(link1_to_link2): 上(小) / 下(大)  —— 方向可能因机械而异，先用小幅度试
        L = deg(-22.0)
        R = deg(+22.0)
        U = deg(+18.0)
        D = deg(-18.0)

        # 其他关节保持 0（你也可以给一点弯曲让动作更明显）
        e = deg(0.0)   # elbow link2_to_link3
        w = deg(0.0)   # wrist/gripper link3_to_gripper_link

        self.corners = [
            [L, U, e, w],  # 左上
            [R, U, e, w],  # 右上
            [R, D, e, w],  # 右下
            [L, D, e, w],  # 左下
        ]

        self.get_logger().info("Square motion ready. Starting in 1s...")
        time.sleep(1.0)

        self.run_square(loop_count=2)  # 走两圈，你想几圈改这里
        self.get_logger().info("Done. Going back to center.")
        self.send_positions([0.0, 0.0, 0.0, 0.0], hold=1.0)

        rclpy.shutdown()

    def publish_js(self, positions):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINTS
        msg.position = [float(x) for x in positions]
        self.pub.publish(msg)

    def send_positions(self, positions, hold=0.0):
        # 连续发布一小段时间，保证 driver 确实收到（更稳）
        t_end = time.time() + max(hold, 0.05)
        dt = 1.0 / self.step_hz
        while time.time() < t_end:
            self.publish_js(positions)
            time.sleep(dt)

    def move_segment(self, p0, p1):
        dt = 1.0 / self.step_hz
        steps = max(1, int(self.seg_time / dt))
        for i in range(steps + 1):
            t = i / steps
            p = [lerp(p0[j], p1[j], t) for j in range(4)]
            self.publish_js(p)
            time.sleep(dt)

    def run_square(self, loop_count=1):
        for k in range(loop_count):
            self.get_logger().info(f"Loop {k+1}/{loop_count}")
            for i in range(4):
                p0 = self.corners[i]
                p1 = self.corners[(i + 1) % 4]
                # 到角点并停一下
                self.send_positions(p0, hold=self.corner_pause)
                # 走一条边（插值）
                self.move_segment(p0, p1)


def main():
    rclpy.init()
    SquareMotion()
    # 不需要 spin：我们用 time.sleep 驱动节奏
    rclpy.shutdown()


if __name__ == "__main__":
    main()
