import time

import cv2
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from ultralytics import YOLO


class LowPassFilter:
    def __init__(self, alpha=0.8):
        self.alpha = alpha
        self.prev = None

    def update(self, value: float) -> float:
        if self.prev is None:
            self.prev = value
        else:
            self.prev = self.alpha * self.prev + (1.0 - self.alpha) * value
        return self.prev

    def get(self):
        return self.prev


class LegoDetectorNode(Node):
    """
    Publish:
      /target/uv   geometry_msgs/Vector3

    Convention:
      msg.x = u
      msg.y = v
      msg.z = 1.0 if detected else 0.0
    """

    def __init__(self):
        super().__init__('lego_detector_node')

        self.declare_parameter('model_path', '/home/hyperdrant/dataset/runs/detect/train/weights/best.pt')
        self.declare_parameter('conf', 0.15)
        self.declare_parameter('camera_id', 0)
        self.declare_parameter('alpha', 0.8)
        self.declare_parameter('deadzone_px', 3)

        self.model_path = str(self.get_parameter('model_path').value)
        self.conf = float(self.get_parameter('conf').value)
        self.camera_id = int(self.get_parameter('camera_id').value)
        self.alpha = float(self.get_parameter('alpha').value)
        self.deadzone_px = int(self.get_parameter('deadzone_px').value)

        self.model = YOLO(self.model_path)
        self.cap = cv2.VideoCapture(self.camera_id)

        self.filter_x = LowPassFilter(alpha=self.alpha)
        self.filter_y = LowPassFilter(alpha=self.alpha)

        self.pub = self.create_publisher(Vector3, '/target/uv', 10)

        self.prev_time = time.time()
        self.cx_prev = None
        self.cy_prev = None

        self.timer = self.create_timer(0.03, self.process_frame)

        self.get_logger().info('LegoDetectorNode started.')

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to read frame from camera.')
            return

        results = self.model(frame, conf=self.conf, verbose=False)
        boxes = results[0].boxes

        detected = False
        status_text = 'NO DETECTION'

        msg = Vector3()
        msg.x = 0.0
        msg.y = 0.0
        msg.z = 0.0

        if len(boxes) > 0:
            detected = True

            # 先取第一个框，后面可升级成置信度最高或离中心最近
            x1, y1, x2, y2 = boxes.xyxy[0].tolist()

            cx_raw = (x1 + x2) / 2.0
            cy_raw = (y1 + y2) / 2.0

            cx = int(self.filter_x.update(cx_raw))
            cy = int(self.filter_y.update(cy_raw))

            status_text = 'DETECTED'

            if self.cx_prev is not None:
                dx = cx - self.cx_prev
                dy = cy - self.cy_prev
            else:
                dx, dy = 0, 0

            if abs(dx) < self.deadzone_px:
                dx = 0
            if abs(dy) < self.deadzone_px:
                dy = 0

            self.cx_prev, self.cy_prev = cx, cy

            msg.x = float(cx)
            msg.y = float(cy)
            msg.z = 1.0
            self.pub.publish(msg)

            cv2.rectangle(
                frame,
                (int(x1), int(y1)),
                (int(x2), int(y2)),
                (0, 255, 0),
                2,
            )
            cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
            cv2.circle(frame, (int(cx_raw), int(cy_raw)), 4, (255, 0, 255), -1)

            cv2.putText(
                frame,
                f"uv=({cx}, {cy})",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )
            cv2.putText(
                frame,
                f"raw=({int(cx_raw)}, {int(cy_raw)})",
                (20, 75),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 0, 255),
                2,
            )
            cv2.putText(
                frame,
                f"dx={dx}, dy={dy}",
                (20, 110),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2,
            )

        else:
            last_cx = self.filter_x.get()
            last_cy = self.filter_y.get()

            if last_cx is not None and last_cy is not None:
                cv2.circle(frame, (int(last_cx), int(last_cy)), 5, (0, 255, 255), -1)
                cv2.putText(
                    frame,
                    "NO DETECTION - HOLD LAST",
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 255, 255),
                    2,
                )
            else:
                cv2.putText(
                    frame,
                    "NO DETECTION",
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 0, 255),
                    2,
                )

        img_h, img_w = frame.shape[:2]
        img_cx = img_w // 2
        img_cy = img_h // 2
        cv2.circle(frame, (img_cx, img_cy), 5, (255, 255, 0), -1)

        curr_time = time.time()
        fps = 1.0 / max(curr_time - self.prev_time, 1e-6)
        self.prev_time = curr_time

        cv2.putText(
            frame,
            f"FPS: {fps:.1f}",
            (20, 145),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 0, 0),
            2,
        )
        cv2.putText(
            frame,
            f"Status: {status_text}",
            (20, 180),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (200, 200, 200),
            2,
        )

        cv2.imshow("Lego Detector Node", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord('q'):
            rclpy.shutdown()

    def destroy_node(self):
        try:
            self.cap.release()
            cv2.destroyAllWindows()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LegoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 