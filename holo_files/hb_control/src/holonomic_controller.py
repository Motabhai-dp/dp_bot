#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

from hb_interfaces.msg import BotCmdArray, BotCmd


class ManualController(Node):

    def __init__(self):
        super().__init__('manual_controller')

        # ---------------------------------------------------------
        # Robot configuration
        # ---------------------------------------------------------

        # Select which robot to control
        self.bot_id = 0

        # Maximum wheel command
        self.max_speed = 22.0

        # ---------------------------------------------------------
        # Controller configuration
        # ---------------------------------------------------------

        # Standard controller mapping.
        #
        # Left stick:
        #   axis 0 -> left/right
        #   axis 1 -> up/down
        #
        # Right stick:
        #   axis 2 -> left/right
        #
        # Change these numbers after checking /joy.
        self.axis_vy = 0
        self.axis_vx = 1
        self.axis_w = 2

        # RB button used as dead-man/enable button.
        #
        # Change this after checking /joy.
        self.enable_button = 5

        # Small joystick deadband
        self.deadband = 0.08

        # ---------------------------------------------------------
        # Arm commands
        # ---------------------------------------------------------

        # Keep arm stationary for now.
        self.base_angle = 0.0
        self.elbow_angle = 0.0

        # ---------------------------------------------------------
        # ROS interfaces
        # ---------------------------------------------------------

        self.joy_subscriber = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        self.publisher = self.create_publisher(
            BotCmdArray,
            '/bot_cmd',
            10
        )

        # Publish continuously
        self.timer = self.create_timer(
            0.03,      # ~33 Hz
            self.publish_command
        )

        # Current controller values
        self.vx = 0.0
        self.vy = 0.0
        self.w = 0.0

        self.enabled = False

        self.get_logger().info(
            'Manual controller started.'
        )

    # ---------------------------------------------------------
    # Deadband
    # ---------------------------------------------------------

    def apply_deadband(self, value):

        if abs(value) < self.deadband:
            return 0.0

        return value

    # ---------------------------------------------------------
    # Controller callback
    # ---------------------------------------------------------

    def joy_callback(self, msg):

        # -----------------------------------------------------
        # Check enable button
        # -----------------------------------------------------

        if self.enable_button < len(msg.buttons):
            self.enabled = bool(
                msg.buttons[self.enable_button]
            )
        else:
            self.enabled = False

        # -----------------------------------------------------
        # If enable button is not pressed, stop robot
        # -----------------------------------------------------

        if not self.enabled:
            self.vx = 0.0
            self.vy = 0.0
            self.w = 0.0
            return

        # -----------------------------------------------------
        # Read axes
        # -----------------------------------------------------

        if self.axis_vx < len(msg.axes):
            self.vx = self.apply_deadband(
                msg.axes[self.axis_vx]
            )
        else:
            self.vx = 0.0

        if self.axis_vy < len(msg.axes):
            self.vy = self.apply_deadband(
                msg.axes[self.axis_vy]
            )
        else:
            self.vy = 0.0

        if self.axis_w < len(msg.axes):
            self.w = self.apply_deadband(
                msg.axes[self.axis_w]
            )
        else:
            self.w = 0.0

    # ---------------------------------------------------------
    # Publish BotCmd
    # ---------------------------------------------------------

    def publish_command(self):

        # -----------------------------------------------------
        # Scale joystick values
        # -----------------------------------------------------

        VX = self.vx * self.max_speed
        VY = self.vy * self.max_speed
        W = self.w * self.max_speed

        # -----------------------------------------------------
        # Holonomic wheel mapping
        #
        # Same equations as your existing code
        # -----------------------------------------------------

        D = 1.0

        m1 = (
            -D * W
            - 0.5 * VX
            + math.sin(math.pi / 3) * VY
        )

        m2 = (
            -D * W
            - 0.5 * VX
            - math.sin(math.pi / 3) * VY
        )

        m3 = (
            -D * W
            + VX
        )

        # -----------------------------------------------------
        # Limit wheel speeds
        # -----------------------------------------------------

        m1 = max(min(m1, self.max_speed), -self.max_speed)
        m2 = max(min(m2, self.max_speed), -self.max_speed)
        m3 = max(min(m3, self.max_speed), -self.max_speed)

        # -----------------------------------------------------
        # Create BotCmdArray
        #
        # Same message structure as your existing code
        # -----------------------------------------------------

        cmd_msg = BotCmdArray()

        cmd = BotCmd()

        cmd.id = self.bot_id

        cmd.m1 = float(m1)
        cmd.m2 = float(m2)
        cmd.m3 = float(m3)

        cmd.base = float(self.base_angle)
        cmd.elbow = float(self.elbow_angle)

        cmd_msg.cmds.append(cmd)

        # -----------------------------------------------------
        # Publish
        # -----------------------------------------------------

        self.publisher.publish(cmd_msg)


def main(args=None):

    rclpy.init(args=args)

    controller = ManualController()

    try:
        rclpy.spin(controller)

    except KeyboardInterrupt:
        pass

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()