import threading

import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import Bool


class SuctionControllerNode(Node):
    def __init__(self):
        super().__init__("suction_controller_node")

        self.declare_parameter("suction_command_topic", "/suction/enable")
        self.declare_parameter("relay1_port", "/dev/arm_relay1")
        self.declare_parameter("relay1_baud", 9600)
        self.declare_parameter("relay1_cmd_on", [0xA0, 0x01, 0x01, 0xA2])
        self.declare_parameter("relay1_cmd_off", [0xA0, 0x01, 0x00, 0xA1])
        self.declare_parameter("relay2_port", "/dev/arm_relay2")
        self.declare_parameter("relay2_baud", 9600)
        self.declare_parameter("relay2_cmd_on", [0xA0, 0x01, 0x01, 0xA2])
        self.declare_parameter("relay2_cmd_off", [0xA0, 0x01, 0x00, 0xA1])
        self.declare_parameter("relay2_pulse_sec", 0.5)

        self._topic = self.get_parameter("suction_command_topic").value
        self._relay1_port = self.get_parameter("relay1_port").value
        self._relay1_baud = self.get_parameter("relay1_baud").value
        self._relay1_cmd_on = self.get_parameter("relay1_cmd_on").value
        self._relay1_cmd_off = self.get_parameter("relay1_cmd_off").value
        self._relay2_port = self.get_parameter("relay2_port").value
        self._relay2_baud = self.get_parameter("relay2_baud").value
        self._relay2_cmd_on = self.get_parameter("relay2_cmd_on").value
        self._relay2_cmd_off = self.get_parameter("relay2_cmd_off").value
        self._relay2_pulse_sec = self.get_parameter("relay2_pulse_sec").value

        self._sub = self.create_subscription(Bool, self._topic, self._on_suction_command, 10)

        self.get_logger().info(
            f"Suction controller ready: topic={self._topic}, "
            f"relay1={self._relay1_port}, relay2={self._relay2_port}, "
            f"pulse={self._relay2_pulse_sec}s"
        )

    def _write_relay(self, port, baud, cmd):
        try:
            ser = serial.Serial(port, baud, timeout=0.5)
            ser.write(bytes(cmd))
            ser.close()
            self.get_logger().info(f"Relay {port}: sent {[hex(b) for b in cmd]}")
        except Exception as e:
            self.get_logger().error(f"Relay write failed on {port}: {e}")

    def _pulse_air_valve_off(self):
        self._write_relay(self._relay2_port, self._relay2_baud, self._relay2_cmd_off)

    def _on_suction_command(self, msg: Bool):
        if msg.data:
            self.get_logger().info("Suction ON: relay1 ON (hold box)")
            self._write_relay(self._relay1_port, self._relay1_baud, self._relay1_cmd_on)
        else:
            self.get_logger().info("Suction OFF: relay1 OFF, relay2 pulse (release air)")
            self._write_relay(self._relay1_port, self._relay1_baud, self._relay1_cmd_off)
            self._write_relay(self._relay2_port, self._relay2_baud, self._relay2_cmd_on)
            threading.Timer(self._relay2_pulse_sec, self._pulse_air_valve_off).start()


def main(args=None):
    rclpy.init(args=args)
    node = SuctionControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
