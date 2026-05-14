import rclpy
import Jetson.GPIO as GPIO
from rclpy.node import Node
from std_msgs.msg import Float32


class ActuatorNode(Node):
    def __init__(self):
        super().__init__("actuator_node")

        self.declare_parameter("rpwm_pin", 33)
        self.declare_parameter("lpwm_pin", 32)
        self.declare_parameter("mock_mode", True)

        self.rpwm_pin = self.get_parameter("rpwm_pin").value
        self.lpwm_pin = self.get_parameter("lpwm_pin").value
        self.mock_mode = self.get_parameter("mock_mode").value

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.rpwm_pin, GPIO.OUT)
        GPIO.setup(self.lpwm_pin, GPIO.OUT)

        self.stop()

        self.sub = self.create_subscription(
            Float32,
            "/actuator_cmd",
            self.cmd_callback,
            10
        )

        self.get_logger().info("QBot actuator Python node started")
        self.get_logger().info(f"RPWM pin: {self.rpwm_pin}")
        self.get_logger().info(f"LPWM pin: {self.lpwm_pin}")
        self.get_logger().info(f"Mock mode: {self.mock_mode}")

    def cmd_callback(self, msg):
        cmd = max(-1.0, min(1.0, msg.data))

        if cmd > 0.05:
            self.extend()
        elif cmd < -0.05:
            self.retract()
        else:
            self.stop()

    def extend(self):
        self.get_logger().info("EXTEND")

        if self.mock_mode:
            return

        self.write_pwm(self.rpwm_pin, GPIO.HIGH)
        self.write_pwm(self.lpwm_pin, GPIO.LOW)

    def retract(self):
        self.get_logger().info("RETRACT")

        if self.mock_mode:
            return

        self.write_pwm(self.rpwm_pin, GPIO.LOW)
        self.write_pwm(self.lpwm_pin, GPIO.HIGH)

    def stop(self):
        self.get_logger().info("STOP")

        if self.mock_mode:
            return

        self.write_pwm(self.rpwm_pin, GPIO.LOW)
        self.write_pwm(self.lpwm_pin, GPIO.LOW)

    def write_pwm(self, pin, value):
        GPIO.output(pin, value)


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = ActuatorNode()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        if node is not None:
            node.stop()
            node.destroy_node()

        GPIO.cleanup()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()