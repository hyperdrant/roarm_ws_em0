import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class KeyboardTargetNode(Node):
    """
    Read target x y z t from terminal and publish as JSON string.

    Topic:
      /arm/target_json   std_msgs/String

    Example payload:
      {"x":235,"y":0,"z":234,"t":3.14}
    """

    def __init__(self):
        super().__init__('keyboard_target_node')

        self.pub = self.create_publisher(String, '/arm/target_json', 10)
        self.timer = self.create_timer(0.5, self.read_input_once)
        self._busy = False

        self.get_logger().info('KeyboardTargetNode started.')
        self.get_logger().info('Input format: x y z t')
        self.get_logger().info('Example: 235 0 234 3.14')

    def read_input_once(self):
        if self._busy:
            return

        self._busy = True
        try:
            text = input('Enter target x y z t: ').strip()
            if not text:
                return

            parts = text.split()
            if len(parts) != 4:
                self.get_logger().warn('Please input 4 numbers, e.g. 235 0 234 3.14')
                return

            x, y, z, t = map(float, parts)

            payload = {
                'x': x,
                'y': y,
                'z': z,
                't': t,
            }

            msg = String()
            msg.data = json.dumps(payload, separators=(',', ':'))
            self.pub.publish(msg)

            self.get_logger().info(f'Published target: {msg.data}')

        except Exception as e:
            self.get_logger().error(f'Input error: {e}')
        finally:
            self._busy = False


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTargetNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()