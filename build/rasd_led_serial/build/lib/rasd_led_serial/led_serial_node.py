#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time


class LEDSerialNode(Node):
    def __init__(self):
        super().__init__("led_serial_node")

        # Serial port (adjust if needed)
        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baud", 115200)

        port = self.get_parameter("port").value
        baud = self.get_parameter("baud").value

        # Open serial connection
        try:
            self.ser = serial.Serial(port, baud, timeout=1)
            time.sleep(2)  # allow Arduino to reset
            self.get_logger().info(f"Connected to Arduino on {port} @ {baud}")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial: {e}")
            self.ser = None

        # Subscriber to LED command
        self.sub = self.create_subscription(
            String, "/rasd/led_command", self.cb_led, 10
        )

    def cb_led(self, msg):
        """Send LED command to Arduino over serial."""
        if self.ser is None:
            return

        cmd = msg.data.upper()

        # Map to single-letter serial commands
        table = {
            "RED": "R",
            "YELLOW": "Y",
            "GREEN": "G",
            "OFF": "O",
        }

        if cmd not in table:
            self.get_logger().warn(f"Unknown LED command: {cmd}")
            return

        serial_cmd = table[cmd] + "\n"
        self.ser.write(serial_cmd.encode())

        self.get_logger().info(f"Sent to Arduino: {serial_cmd.strip()}")


def main(args=None):
    rclpy.init(args=args)
    node = LEDSerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
