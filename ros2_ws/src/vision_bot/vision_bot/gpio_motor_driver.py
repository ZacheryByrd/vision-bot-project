#!/usr/bin/env python3
"""
gpio_motor_driver
==================
Hardware-only bridge node. Subscribes to the same /cmd_vel topic that
Gazebo's diff-drive plugin consumes in simulation, and instead drives a
real motor driver (wired for an L298N/TB6612-style two-motor driver) via
Raspberry Pi GPIO.

This is the ONLY file that changes between "runs in sim" and "runs on
a real rover" -- perception_node and motor_control_node are identical
in both cases. Only import RPi.GPIO here (not at the top of any other
file) so the rest of the package still runs fine on a dev machine that
isn't a Pi.

Wiring assumption (adjust GPIO pin numbers to your driver board):
    Left motor:  IN1 -> LEFT_FWD_PIN, IN2 -> LEFT_BWD_PIN, ENA -> PWM
    Right motor: IN3 -> RIGHT_FWD_PIN, IN4 -> RIGHT_BWD_PIN, ENB -> PWM
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Differential-drive geometry -- measure your own chassis and update these.
WHEEL_SEPARATION_M = 0.15
MAX_WHEEL_SPEED_MPS = 0.3

LEFT_FWD_PIN = 17
LEFT_BWD_PIN = 27
RIGHT_FWD_PIN = 22
RIGHT_BWD_PIN = 23
LEFT_PWM_PIN = 18
RIGHT_PWM_PIN = 13


class GpioMotorDriver(Node):
    def __init__(self):
        super().__init__("gpio_motor_driver")

        try:
            import RPi.GPIO as GPIO
            self._gpio = GPIO
        except ImportError:
            self._gpio = None
            self.get_logger().warn(
                "RPi.GPIO not available -- running in dry-run/log-only mode. "
                "This is expected on a dev machine; only the real Pi needs "
                "the RPi.GPIO package installed."
            )

        if self._gpio:
            self._gpio.setmode(self._gpio.BCM)
            for pin in (LEFT_FWD_PIN, LEFT_BWD_PIN, RIGHT_FWD_PIN, RIGHT_BWD_PIN):
                self._gpio.setup(pin, self._gpio.OUT)
            self._gpio.setup(LEFT_PWM_PIN, self._gpio.OUT)
            self._gpio.setup(RIGHT_PWM_PIN, self._gpio.OUT)
            self._left_pwm = self._gpio.PWM(LEFT_PWM_PIN, 1000)
            self._right_pwm = self._gpio.PWM(RIGHT_PWM_PIN, 1000)
            self._left_pwm.start(0)
            self._right_pwm.start(0)

        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
        self.get_logger().info("gpio_motor_driver up, listening on /cmd_vel")

    def _on_cmd_vel(self, msg: Twist):
        linear = msg.linear.x
        angular = msg.angular.z

        left_speed = linear - (angular * WHEEL_SEPARATION_M / 2.0)
        right_speed = linear + (angular * WHEEL_SEPARATION_M / 2.0)

        left_pct = self._to_duty_cycle(left_speed)
        right_pct = self._to_duty_cycle(right_speed)

        if not self._gpio:
            self.get_logger().info(
                f"[dry-run] left={left_speed:.2f}m/s ({left_pct:.0f}%) "
                f"right={right_speed:.2f}m/s ({right_pct:.0f}%)"
            )
            return

        self._drive_side(left_speed, left_pct, LEFT_FWD_PIN, LEFT_BWD_PIN, self._left_pwm)
        self._drive_side(right_speed, right_pct, RIGHT_FWD_PIN, RIGHT_BWD_PIN, self._right_pwm)

    def _to_duty_cycle(self, speed_mps: float) -> float:
        pct = min(1.0, abs(speed_mps) / MAX_WHEEL_SPEED_MPS) * 100.0
        return pct

    def _drive_side(self, speed, duty_pct, fwd_pin, bwd_pin, pwm):
        self._gpio.output(fwd_pin, speed >= 0)
        self._gpio.output(bwd_pin, speed < 0)
        pwm.ChangeDutyCycle(duty_pct)

    def destroy_node(self):
        if self._gpio:
            self._left_pwm.stop()
            self._right_pwm.stop()
            self._gpio.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GpioMotorDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
