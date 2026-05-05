import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

class YoloFaceNode(Node):

    def __init__(self):
        super().__init__('yolo_face_node')

        self.bridge = CvBridge()
        self.model = YOLO("/home/hyperdrant/roarm_ws_em0/models/yolov12n-face.pt")

        self.subscription = self.create_subscription(
            Image,
            '/image',
            self.image_callback,
            10)

        self.publisher = self.create_publisher(
            Float32MultiArray,
            '/face_center',
            10)

    def image_callback(self, msg):

        self.get_logger().info("Image received", throttle_duration_sec=2.0)
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        results = self.model(frame, verbose=False)

        for r in results:
            if len(r.boxes) > 0:
                box = r.boxes.xyxy[0].cpu().numpy()
                x1, y1, x2, y2 = box[:4]

                center_x = float((x1 + x2) / 2)
                center_y = float((y1 + y2) / 2)

                msg_out = Float32MultiArray()
                msg_out.data = [center_x, center_y]
                self.publisher.publish(msg_out)
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0,255,0), 2)
                cv2.imshow("yolo_face", frame)
                cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = YoloFaceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()