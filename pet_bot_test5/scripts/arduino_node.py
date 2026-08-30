#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import serial
import time
import math


class ArduinoBridge(Node):

    def __init__(self):
        super().__init__('arduino_bridge')

        # =====================================================
        # POLYNOMIAL FIT COEFFICIENTS (Poly22)
        # =====================================================
        self.p00 = 45.75
        self.p10 = 0.7964
        self.p01 = -0.02648
        self.p20 = 0.002635
        self.p11 = -0.01639
        self.p02 = -0.00144

        # Flag to flip quadratic root sign (+ vs -) if physical movement is reversed
        self.use_positive_root = True

        # =====================================================
        # ARDUINO SERIAL CONNECTION
        # =====================================================
        try:
            self.arduino = serial.Serial(
                '/dev/ttyACM0',
                115200,
                timeout=1
            )
            # Arduino resets when serial connection opens
            time.sleep(2)
            self.get_logger().info('Arduino connected successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            self.arduino = None

        # =====================================================
        # SUBSCRIBE TO GAIT JOINT ANGLES
        # =====================================================
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/forward_position_controller/commands',
            self.angle_callback,
            10
        )

        self.get_logger().info('Subscribed to /forward_position_controller/commands')

    # =========================================================
    # KNEE POLYNOMIAL KINEMATIC SOLVER
    # =========================================================
    def calculate_knee_motor_deg(self, x_deg: float, z_deg: float) -> float:
        """
        Maps thigh motor angle x_deg and target IK knee angle z_deg 
        to physical knee motor angle y_deg.
        """
        A = self.p02
        B = self.p01 + (self.p11 * x_deg)
        C = self.p00 + (self.p10 * x_deg) + (self.p20 * (x_deg ** 2)) - z_deg

        discriminant = (B ** 2) - (4 * A * C)

        # Handle boundary/unreachable targets
        if discriminant < 0:
            discriminant = 0.0

        sqrt_disc = math.sqrt(discriminant)

        if self.use_positive_root:
            y_deg = (-B + sqrt_disc) / (2.0 * A)
        else:
            y_deg = (-B - sqrt_disc) / (2.0 * A)

        return y_deg

    # =========================================================
    # JOINT ANGLE CALLBACK
    # =========================================================
    def angle_callback(self, msg):
        angles_rad = msg.data

        # Check number of joints
        if len(angles_rad) != 12:
            self.get_logger().error(
                f'Expected 12 joint angles, but received {len(angles_rad)}'
            )
            return

        servo_angles = []

        # Process 4 legs, assuming joint array structure:
        # [Leg1_Ab, Leg1_Thigh, Leg1_Knee, Leg2_Ab, Leg2_Thigh, Leg2_Knee, ...]
        for leg_idx in range(4):
            base = leg_idx * 3

            ab_rad = angles_rad[base]
            x_rad  = angles_rad[base + 1]  # Thigh IK (x)
            z_rad  = angles_rad[base + 2]  # Knee IK (z)

            # Convert IK inputs to degrees
            ab_deg = math.degrees(ab_rad)
            x_deg  = math.degrees(x_rad)
            z_deg  = math.degrees(z_rad)

            # Derive physical knee motor angle y_deg via quadratic mapping
            y_deg = self.calculate_knee_motor_deg(x_deg, z_deg)

            # Convert to Servo Signals (applying 90 deg offset)
            servo_ab    = 90.0 + ab_deg
            servo_thigh = 90.0 + x_deg
            servo_knee  = 90.0 + y_deg

            # Clamp output ranges to 0-180 degrees
            for angle in [servo_ab, servo_thigh, servo_knee]:
                clamped = max(0.0, min(180.0, angle))
                servo_angles.append(round(clamped, 2))

        # =====================================================
        # CREATE SERIAL MESSAGE
        # =====================================================
        data = ','.join(str(angle) for angle in servo_angles)
        payload = data + '\r'

        # =====================================================
        # SEND TO ARDUINO
        # =====================================================
        if self.arduino and self.arduino.is_open:
            try:
                self.arduino.write(payload.encode('utf-8'))
                self.get_logger().info(f'Sent to Arduino: {data}')
            except Exception as e:
                self.get_logger().error(f'Failed to write to serial port: {e}')
        else:
            self.get_logger().error('Arduino serial port is closed')

    # =========================================================
    # CLEANUP
    # =========================================================
    def destroy_node(self):
        if self.arduino and self.arduino.is_open:
            self.arduino.close()
            self.get_logger().info('Arduino serial port closed')
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


if __name__ == '__main__':
    main()