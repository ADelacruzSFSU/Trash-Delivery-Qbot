import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class ActuatorNode(Node):
    def __init__(self):
        super().__init__("actuator_node")

        self.declare_parameter("rpwm_pin", 33)  # QBot physical pin 33, GPIO 13
        self.declare_parameter("lpwm_pin", 32)  # QBot physical pin 32, GPIO 12
        self.declare_parameter("mock_mode", True)

        self.rpwm_pin = self.get_parameter("rpwm_pin").value
        self.lpwm_pin = self.get_parameter("lpwm_pin").value
        self.mock_mode = self.get_parameter("mock_mode").value

        self.get_logger().info("QBot actuator Python node started")
        self.get_logger().info(f"RPWM pin: {self.rpwm_pin}")
        self.get_logger().info(f"LPWM pin: {self.lpwm_pin}")
        self.get_logger().info(f"Mock mode: {self.mock_mode}")

        self.sub = self.create_subscription(
            Float32,
            "/actuator_cmd",
            self.cmd_callback,
            10
        )

    def cmd_callback(self, msg):
        cmd = max(-1.0, min(1.0, msg.data))

        if cmd > 0.05:
            self.extend(cmd)
        elif cmd < -0.05:
            self.retract(abs(cmd))
        else:
            self.stop()

    def extend(self, duty):
        self.get_logger().info(f"EXTEND duty={duty:.2f}")

        if self.mock_mode:
            return

        # Real hardware later:
        # RPWM = duty
        # LPWM = 0
        self.write_pwm(self.rpwm_pin, duty)
        self.write_pwm(self.lpwm_pin, 0.0)

    def retract(self, duty):
        self.get_logger().info(f"RETRACT duty={duty:.2f}")

        if self.mock_mode:
            return

        # Real hardware later:
        # RPWM = 0
        # LPWM = duty
        self.write_pwm(self.rpwm_pin, 0.0)
        self.write_pwm(self.lpwm_pin, duty)

    def stop(self):
        self.get_logger().info("STOP")

        if self.mock_mode:
            return

        self.write_pwm(self.rpwm_pin, 0.0)
        self.write_pwm(self.lpwm_pin, 0.0)

    def write_pwm(self, pin, duty):
        """
        Placeholder for real QBot GPIO/PWM code.
        duty should be from 0.0 to 1.0.
        """
        self.get_logger().info(f"PWM pin {pin} = {duty:.2f}")


def main(args=None):
    rclpy.init(args=args)
    node = ActuatorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()