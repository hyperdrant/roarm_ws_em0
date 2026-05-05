import math
import time

import rclpy
from rclpy.node import Node

from face_tracker_msgs.msg import Offset
from geometry_msgs.msg import Vector3


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


class FaceControllerNode(Node):
    """
    Image-based visual servoing style controller.

    Subscribe:  /target/offset   (Offset: ux, uy, valid)
    Publish:    /gimbal/cmd      (Vector3: x=pan_deg, y=tilt_deg, z=0)

    - Treat ux/uy as normalized image-plane error
    - Convert error -> angular rate (deg/s), saturate smoothly with tanh
    - Integrate rate over dt to get pan/tilt angle
    - Add valid streak gate to suppress false positives
    - Lost target: hold or return-to-center
    """

    def __init__(self):
        super().__init__('face_controller_node')

        # ====== Parameters ======
        self.declare_parameter('rate_hz', 30.0)

        # Nonlinear gain for tanh (bigger -> more aggressive)
        self.declare_parameter('gain_pan', 2.0)
        self.declare_parameter('gain_tilt', 2.0)

        # Max angular speed (deg/s)
        self.declare_parameter('max_rate_pan_deg_s', 45.0)
        self.declare_parameter('max_rate_tilt_deg_s', 35.0)

        # Angle limits (deg)
        self.declare_parameter('pan_limit_deg', 60.0)
        self.declare_parameter('tilt_limit_deg', 40.0)

        # Small error deadzone (on ux/uy, not degrees)
        self.declare_parameter('dead_zone', 0.08)

        # Valid streak gate (avoid false positives)
        self.declare_parameter('valid_required_frames', 5)

        # Lost target behavior: 'hold' or 'return'
        self.declare_parameter('lost_mode', 'return')
        self.declare_parameter('return_rate_pan_deg_s', 15.0)   # deg/s back to 0
        self.declare_parameter('return_rate_tilt_deg_s', 12.0)  # deg/s back to 0

        # Debug logging
        self.declare_parameter('log_cmd', False)

        # ====== Read params ======
        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.gain_pan = float(self.get_parameter('gain_pan').value)
        self.gain_tilt = float(self.get_parameter('gain_tilt').value)

        self.max_rate_pan = float(self.get_parameter('max_rate_pan_deg_s').value)
        self.max_rate_tilt = float(self.get_parameter('max_rate_tilt_deg_s').value)

        self.pan_limit = float(self.get_parameter('pan_limit_deg').value)
        self.tilt_limit = float(self.get_parameter('tilt_limit_deg').value)

        self.dead_zone = float(self.get_parameter('dead_zone').value)
        self.valid_required = int(self.get_parameter('valid_required_frames').value)

        self.lost_mode = str(self.get_parameter('lost_mode').value)
        self.return_rate_pan = float(self.get_parameter('return_rate_pan_deg_s').value)
        self.return_rate_tilt = float(self.get_parameter('return_rate_tilt_deg_s').value)

        self.log_cmd = bool(self.get_parameter('log_cmd').value)

        # ====== State ======
        self.pan_deg = 0.0
        self.tilt_deg = 0.0

        self.last_offset = Offset()
        self.last_offset.valid = False

        self.valid_streak = 0
        self.last_time = time.time()

        # ====== ROS I/O ======
        self.sub = self.create_subscription(Offset, '/target/offset', self.on_offset, 10)
        self.pub = self.create_publisher(Vector3, '/gimbal/cmd', 10)

        self.timer = self.create_timer(1.0 / self.rate_hz, self.control_loop)

        self.get_logger().info(
            "FaceControllerNode (visual servo) started. "
            f"rate_hz={self.rate_hz}, dead_zone={self.dead_zone}, valid_required_frames={self.valid_required}, "
            f"max_rate_pan={self.max_rate_pan} deg/s, max_rate_tilt={self.max_rate_tilt} deg/s, "
            f"limits pan±{self.pan_limit}, tilt±{self.tilt_limit}, lost_mode={self.lost_mode}"
        )

    def on_offset(self, msg: Offset):
        self.last_offset = msg

    def control_loop(self):
        # Compute dt robustly
        now = time.time()
        dt = now - self.last_time
        self.last_time = now
        if dt <= 0.0 or dt > 0.5:
            dt = 1.0 / self.rate_hz

        # Update valid streak gate
        if self.last_offset.valid:
            self.valid_streak += 1
        else:
            self.valid_streak = 0

        # If not stably detected yet -> treat as lost
        stable_valid = (self.valid_streak >= self.valid_required)

        if not stable_valid:
            self._handle_lost(dt)
            self._publish_cmd()
            return

        # Read error
        ux = float(self.last_offset.ux)
        uy = float(self.last_offset.uy)

        # Deadzone on normalized error
        if abs(ux) < self.dead_zone:
            ux = 0.0
        if abs(uy) < self.dead_zone:
            uy = 0.0

        # ====== Visual servo control law ======
        # Convert normalized error -> angular rate with smooth saturation.
        # NOTE about sign:
        # - If you find it turns the wrong way, flip the sign of ux/uy here.
        #   E.g. use (+ux) instead of (-ux).
        pan_rate = -self.max_rate_pan * math.tanh(self.gain_pan * ux)    # deg/s
        tilt_rate = -self.max_rate_tilt * math.tanh(self.gain_tilt * uy) # deg/s

        # Integrate to angles
        self.pan_deg += pan_rate * dt
        self.tilt_deg += tilt_rate * dt

        # Clamp to safety limits
        self.pan_deg = clamp(self.pan_deg, -self.pan_limit, self.pan_limit)
        self.tilt_deg = clamp(self.tilt_deg, -self.tilt_limit, self.tilt_limit)

        self._publish_cmd(pan_rate, tilt_rate)

    def _handle_lost(self, dt: float):
        # When lost: either hold or return-to-center.
        if self.lost_mode != 'return':
            # hold: do nothing
            return

        # Move angles toward 0 at fixed rate
        self.pan_deg = self._move_towards(self.pan_deg, 0.0, self.return_rate_pan * dt)
        self.tilt_deg = self._move_towards(self.tilt_deg, 0.0, self.return_rate_tilt * dt)

    def _publish_cmd(self, pan_rate=None, tilt_rate=None):
        cmd = Vector3()
        cmd.x = float(self.pan_deg)
        cmd.y = float(self.tilt_deg)
        cmd.z = 0.0
        self.pub.publish(cmd)

        if self.log_cmd:
            if pan_rate is None:
                self.get_logger().info(f"cmd pan={self.pan_deg:.2f} deg, tilt={self.tilt_deg:.2f} deg (lost/unstable)")
            else:
                self.get_logger().info(
                    f"cmd pan={self.pan_deg:.2f} deg, tilt={self.tilt_deg:.2f} deg | "
                    f"rate pan={pan_rate:.1f} deg/s, tilt={tilt_rate:.1f} deg/s"
                )

    @staticmethod
    def _move_towards(current: float, target: float, max_delta: float) -> float:
        if current < target:
            return min(current + max_delta, target)
        return max(current - max_delta, target)


def main():
    rclpy.init()
    node = FaceControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
