import cv2
import rclpy
from rclpy.node import Node
from ultralytics import YOLO

from face_tracker_msgs.msg import Offset


class FaceTrackerNode(Node):
    def __init__(self):
        super().__init__('face_tracker_node')

        # ROS 参数
        self.declare_parameter('camera_index', 2)
        self.declare_parameter('show_view', True)
        self.declare_parameter('dead_zone', 0.1)
        self.declare_parameter('model_path', '/home/hyperdrant/roarm_ws_em0/models/yolov12n-face.pt')
        self.declare_parameter('conf_thres', 0.5)
        self.declare_parameter('input_width', 640)
        self.declare_parameter('input_height', 480)

        cam_idx = int(self.get_parameter('camera_index').value)
        self.show_view = bool(self.get_parameter('show_view').value)
        self.dead_zone = float(self.get_parameter('dead_zone').value)
        self.model_path = str(self.get_parameter('model_path').value)
        self.conf_thres = float(self.get_parameter('conf_thres').value)
        self.input_width = int(self.get_parameter('input_width').value)
        self.input_height = int(self.get_parameter('input_height').value)

        # 打开摄像头
        self.cap = cv2.VideoCapture(cam_idx)
        if not self.cap.isOpened():
            raise RuntimeError(f"Camera open failed. Try another index. Current: {cam_idx}")

        # 加载 YOLO 模型
        self.get_logger().info(f"Loading YOLO model from: {self.model_path}")
        self.model = YOLO(self.model_path)

        # 发布器
        self.pub = self.create_publisher(Offset, '/target/offset', 10)

        # 定时器
        self.timer = self.create_timer(1.0 / 30.0, self.loop)

        self.get_logger().info(
            f"FaceTrackerNode started. camera_index={cam_idx}, "
            f"show_view={self.show_view}, dead_zone={self.dead_zone}, "
            f"conf_thres={self.conf_thres}"
        )

    def loop(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Frame read failed.")
            return

        # 可选：降分辨率，加快速度
        frame = cv2.resize(frame, (self.input_width, self.input_height))

        h, w = frame.shape[:2]
        img_cx, img_cy = w // 2, h // 2

        # 默认发布无效
        msg = Offset()
        msg.ux = 0.0
        msg.uy = 0.0
        msg.valid = False

        # YOLO 推理
        results = self.model(frame, verbose=False, conf=self.conf_thres)

        best_box = None
        best_area = 0

        for r in results:
            if r.boxes is None or len(r.boxes) == 0:
                continue

            boxes = r.boxes.xyxy.cpu().numpy()
            for box in boxes:
                x1, y1, x2, y2 = box[:4]
                area = (x2 - x1) * (y2 - y1)
                if area > best_area:
                    best_area = area
                    best_box = (int(x1), int(y1), int(x2), int(y2))

        if best_box is not None:
            x1, y1, x2, y2 = best_box
            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2

            # 归一化偏移
            ux = (cx - img_cx) / (w / 2)
            uy = (cy - img_cy) / (h / 2)

            # 死区防抖
            if abs(ux) < self.dead_zone:
                ux = 0.0
            if abs(uy) < self.dead_zone:
                uy = 0.0

            msg.ux = float(ux)
            msg.uy = float(uy)
            msg.valid = True

            if self.show_view:
                cv2.drawMarker(
                    frame, (img_cx, img_cy), (0, 255, 0),
                    markerType=cv2.MARKER_CROSS, thickness=2
                )
                cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
                cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
                cv2.putText(
                    frame, f"ux={ux:.2f}, uy={uy:.2f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2
                )
        else:
            if self.show_view:
                cv2.drawMarker(
                    frame, (img_cx, img_cy), (0, 255, 0),
                    markerType=cv2.MARKER_CROSS, thickness=2
                )
                cv2.putText(
                    frame, "No face", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2
                )

        # 发布 offset
        self.pub.publish(msg)

        # 显示画面
        if self.show_view:
            cv2.imshow("Face Tracker (YOLO, ROS2)", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == 27:
                self.get_logger().info("ESC pressed. Shutting down.")
                rclpy.shutdown()

    def destroy_node(self):
        if hasattr(self, 'cap') and self.cap is not None:
            self.cap.release()
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = FaceTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()