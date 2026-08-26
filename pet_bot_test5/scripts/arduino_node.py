#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray

import serial
import time
import math


class ArduinoBridge(Node):

    def _init_(self):
        super()._init_('arduino_bridge')

        # =====================================================
        # ARDUINO SERIAL CONNECTION
        # =====================================================

        self.arduino = serial.Serial(
            '/dev/ttyACM0',
            115200,
            timeout=1
        )

        # Arduino usually resets when serial connection opens
        time.sleep(2)

        self.get_logger().info(
            'Arduino connected successfully'
        )

        # =====================================================
        # SUBSCRIBE TO GAIT JOINT ANGLES
        # =====================================================

        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/forward_position_controller/commands',
            self.angle_callback,
            10
        )

        self.get_logger().info(
            'Subscribed to /forward_position_controller/commands'
        )

    # =========================================================
    # JOINT ANGLE CALLBACK
    # =========================================================

    def angle_callback(self, msg):

        # Get the 12 angles from ROS
        angles_rad = msg.data

        # -----------------------------------------------------
        # Check number of joints
        # -----------------------------------------------------

        if len(angles_rad) != 12:

            self.get_logger().error(
                f'Expected 12 joint angles, '
                f'but received {len(angles_rad)}'
            )

            return

        # -----------------------------------------------------
        # Convert radians → degrees
        # -----------------------------------------------------

        servo_angles = []

        for angle_rad in angles_rad:

            # radians → degrees
            angle_deg = math.degrees(angle_rad)


            servo_angle = 90.0 + angle_deg

            # -------------------------------------------------
            # Limit servo angle to 0–180
            # -------------------------------------------------

            servo_angle = max(
                0.0,
                min(180.0, servo_angle)
            )

            servo_angles.append(
                round(servo_angle, 2)
            )

        # =====================================================
        # CREATE SERIAL MESSAGE
        # =====================================================

        # Convert:
        #
        # [86.56, 118.77, 58.08, ...]
        #
        # into:
        #
        # 86.56,118.77,58.08,...\r

        data = ','.join(
            str(angle)
            for angle in servo_angles
        )

        # \r = carriage return
        # This tells Arduino where the message ends.

        payload = data + '\r'

        # =====================================================
        # SEND TO ARDUINO
        # =====================================================

        if self.arduino and self.arduino.is_open:

            self.arduino.write(
                payload.encode('utf-8')
            )

            self.get_logger().info(
                f'Sent to Arduino: {data}'
            )

        else:

            self.get_logger().error(
                'Arduino serial port is closed'
            )

    # =========================================================
    # CLEANUP
    # =========================================================

    def destroy_node(self):

        if self.arduino and self.arduino.is_open:

            self.arduino.close()

            self.get_logger().info(
                'Arduino serial port closed'
            )

        super().destroy_node()


# =============================================================
# MAIN
# =============================================================

def main(args=None):

    rclpy.init(args=args)

    node = ArduinoBridge()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__== '_main_':

    main()