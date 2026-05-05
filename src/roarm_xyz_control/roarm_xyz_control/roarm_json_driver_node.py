import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import serial


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


class RoarmJsonDriverNode(Node):
    """
    Subscribe:
      /arm/target_json   std_msgs/String

    Input payload example:
      {"x":235,"y":0,"z":234,"t":3.14}

    Output serial JSON:
      {"T":1041,"x":235,"y":0,"z":234,"t":3.14}
    """

    def __init__(self):
        super().__init__('roarm_json_driver_node')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('cmd_type', 1041)

        self.declare_parameter('min_x', -999.0)
        self.declare_parameter('max_x', 999.0)
        self.declare_parameter('min_y', -999.0)
        self.declare_parameter('max_y', 999.0)
        self.declare_parameter('min_z', -999.0)
        self.declare_parameter('max_z', 999.0)
        self.declare_parameter('min_t', -10.0)
        self.declare_parameter('max_t', 10.0)

        self.port = str(self.get_parameter('port').value)
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.cmd_type = int(self.get_parameter('cmd_type').value)

        self.min_x = float(self.get_parameter('min_x').value)
        self.max_x = float(self.get_parameter('max_x').value)
        self.min_y = float(self.get_parameter('min_y').value)
        self.max_y = float(self.get_parameter('max_y').value)
        self.min_z = float(self.get_parameter('min_z').value)
        self.max_z = float(self.get_parameter('max_z').value)
        self.min_t = float(self.get_parameter('min_t').value)
        self.max_t = float(self.get_parameter('max_t').value)

        self.ser = None
        self.connect_serial()

        self.sub = self.create_subscription(
            String,
            '/arm/target_json',
            self.on_target_json,
            10
        )

        self.get_logger().info(
            f'RoarmJsonDriverNode started. port={self.port}, baudrate={self.baudrate}, cmd_type={self.cmd_type}'
        )

    def connect_serial(self):
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=0.2
            )
            self.ser.setRTS(False)
            self.ser.setDTR(False)
            self.get_logger().info(f'Serial connected: {self.port}')
        except Exception as e:
            self.get_logger().error(f'Failed to open serial {self.port}: {e}')
            self.ser = None

    def send_json_cmd(self, cmd: dict):
        if self.ser is None or not self.ser.is_open:
            self.get_logger().warn('Serial not connected, retrying...')
            self.connect_serial()
            if self.ser is None:
                return

        try:
            text = json.dumps(cmd, separators=(',', ':'))
            self.ser.write(text.encode('utf-8') + b'\n')
            self.get_logger().info(f'Sent: {text}')
        except Exception as e:
            self.get_logger().error(f'Failed to send JSON: {e}')

    def on_target_json(self, msg: String):
        try:
            data = json.loads(msg.data)

            x = clamp(float(data['x']), self.min_x, self.max_x)
            y = clamp(float(data['y']), self.min_y, self.max_y)
            z = clamp(float(data['z']), self.min_z, self.max_z)
            t = clamp(float(data['t']), self.min_t, self.max_t)

            cmd = {
                'T': self.cmd_type,
                'x': round(x, 3),
                'y': round(y, 3),
                'z': round(z, 3),
                't': round(t, 3),
            }

            self.send_json_cmd(cmd)

        except Exception as e:
            self.get_logger().error(f'Invalid target JSON: {e}; raw={msg.data}')

    def destroy_node(self):
        try:
            if self.ser is not None and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RoarmJsonDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()