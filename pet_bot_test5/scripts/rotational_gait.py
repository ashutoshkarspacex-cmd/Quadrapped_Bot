#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

L1 = 0.08
L2 = 0.08

RATE = 100.0

SWING_TIME = 0.30
STANCE_TIME = 0.15

STEP_HEIGHT = 0.008
BODY_HEIGHT = 0.17

STEP_ANGLE = math.radians(3.0)

STARTUP_HOLD = 1.0

DIRECTION = 1

LEGS = ["FL", "FR", "RL", "RR"]

LEG_INDEX = {
    "FL": 0,
    "FR": 3,
    "RL": 6,
    "RR": 9
}

HIP_X = {
    "FL": 0.045,
    "FR": 0.045,
    "RL": -0.045,
    "RR": -0.045
}

HIP_Y = {
    "FL": 0.035,
    "FR": -0.035,
    "RL": 0.035,
    "RR": -0.035
}

HIP_SIGN = {
    "FL": -1,
    "FR": +1,
    "RL": +1,
    "RR": -1
}

THIGH_SIGN = {
    "FL": +1,
    "FR": -1,
    "RL": -1,
    "RR": -1
}

KNEE_SIGN = {
    "FL": +1,
    "FR": -1,
    "RL": +1,
    "RR": -1
}


def leg_ik_3d(x, y, z):

    hip = math.atan2(y, z)

    A = math.sqrt(y * y + z * z)

    R = math.sqrt(x * x + A * A)

    R = max(
        min(R, L1 + L2 - 1e-6),
        abs(L1 - L2) + 1e-6
    )

    gamma = math.atan2(A, x)

    cos_beta = (
        L1 * L1 +
        R * R -
        L2 * L2
    ) / (2.0 * L1 * R)

    cos_beta = max(-1.0, min(1.0, cos_beta))

    beta = math.acos(cos_beta)

    thigh = gamma + beta - math.pi / 2.0

    cos_knee = (
        L1 * L1 +
        L2 * L2 -
        R * R
    ) / (2.0 * L1 * L2)

    cos_knee = max(-1.0, min(1.0, cos_knee))

    knee_internal = math.acos(cos_knee)

    knee = -(math.pi - knee_internal)

    return hip, thigh, knee


def smoothstep(s):

    return 3.0 * s * s - 2.0 * s * s * s


class RotateGait(Node):

    def __init__(self, steps):

        super().__init__("quadruped_rotation_controller")

        self.pub = self.create_publisher(
            Float64MultiArray,
            "/forward_position_controller/commands",
            10
        )

        self.timer = self.create_timer(
            1.0 / RATE,
            self.update
        )

        self.start_time = (
            self.get_clock().now().nanoseconds * 1e-9
        )

        self.last_time = self.start_time

        self.state = "startup"
        self.state_time = 0.0

        self.current_leg = 0
        self.cycles = 0
        self.max_cycles = steps

        if DIRECTION == 1:

            self.sequence = [
                "FL",
                "RR",
                "FR",
                "RL"
            ]

        else:

            self.sequence = [
                "FR",
                "RL",
                "RR",
                "FL"
            ]

        self.foot_x = {
            "FL": 0.02,
            "FR": 0.02,
            "RL": 0.02,
            "RR": 0.02
        }

        self.foot_y = {
            "FL": 0.0,
            "FR": 0.0,
            "RL": 0.0,
            "RR": 0.0
        }

        self.swing_start_x = 0.0
        self.swing_start_y = 0.0

        self.swing_target_x = 0.0
        self.swing_target_y = 0.0

    def begin_swing(self):

        leg = self.sequence[self.current_leg]

        self.swing_start_x = self.foot_x[leg]
        self.swing_start_y = self.foot_y[leg]

        body_x = HIP_X[leg]
        body_y = HIP_Y[leg]

        foot_body_x = (
            body_x +
            self.swing_start_x
        )

        foot_body_y = (
            body_y +
            self.swing_start_y
        )

        angle = DIRECTION * STEP_ANGLE

        c = math.cos(-angle)
        s = math.sin(-angle)

        rotated_x = (
            c * foot_body_x -
            s * foot_body_y
        )

        rotated_y = (
            s * foot_body_x +
            c * foot_body_y
        )

        self.swing_target_x = (
            rotated_x -
            body_x
        )

        self.swing_target_y = (
            rotated_y -
            body_y
        )

    def update(self):

        now = (
            self.get_clock().now().nanoseconds * 1e-9
        )

        dt = now - self.last_time
        self.last_time = now

        elapsed = now - self.start_time

        if elapsed < STARTUP_HOLD:

            self.publish_neutral()

            return

        if self.state == "startup":

            self.state = "swing"
            self.state_time = 0.0

            self.begin_swing()

        self.state_time += dt

        if self.state == "swing":

            self.update_swing()

        else:

            self.update_stance()

    def update_swing(self):

        leg = self.sequence[self.current_leg]

        s = self.state_time / SWING_TIME

        s = min(s, 1.0)

        p = smoothstep(s)

        x = (
            self.swing_start_x +
            (
                self.swing_target_x -
                self.swing_start_x
            ) * p
        )

        y = (
            self.swing_start_y +
            (
                self.swing_target_y -
                self.swing_start_y
            ) * p
        )

        z = (
            BODY_HEIGHT -
            STEP_HEIGHT *
            math.sin(math.pi * s) ** 2
        )

        self.publish_commands(
            swing_leg=leg,
            swing_x=x,
            swing_y=y,
            swing_z=z
        )

        if self.state_time >= SWING_TIME:

            self.foot_x[leg] = self.swing_target_x
            self.foot_y[leg] = self.swing_target_y

            self.state = "stance"
            self.state_time = 0.0

    def update_stance(self):

        self.publish_commands()

        if self.state_time >= STANCE_TIME:

            self.current_leg += 1

            if self.current_leg >= len(self.sequence):

                self.current_leg = 0
                self.cycles += 1

                if self.cycles >= self.max_cycles:

                    self.publish_neutral()

                    self.timer.cancel()

                    return

            self.state = "swing"
            self.state_time = 0.0

            self.begin_swing()

    def publish_commands(
        self,
        swing_leg=None,
        swing_x=None,
        swing_y=None,
        swing_z=None
    ):

        cmd = [0.0] * 12

        for leg in LEGS:

            if leg == swing_leg:

                x = swing_x
                y = swing_y
                z = swing_z

            else:

                x = self.foot_x[leg]
                y = self.foot_y[leg]
                z = BODY_HEIGHT

            hip_ik, thigh_ik, knee_ik = leg_ik_3d(
                x,
                y,
                z
            )

            hip = HIP_SIGN[leg] * hip_ik

            thigh = THIGH_SIGN[leg] * thigh_ik

            knee = KNEE_SIGN[leg] * knee_ik

            idx = LEG_INDEX[leg]

            cmd[idx] = hip
            cmd[idx + 1] = thigh
            cmd[idx + 2] = knee

        msg = Float64MultiArray()

        msg.data = cmd

        self.pub.publish(msg)

    def publish_neutral(self):

        msg = Float64MultiArray()

        msg.data = [
            -0.4, 0.0, 0.0,
             0.4, 0.0, 0.0,
             0.4, 0.0, 0.0,
            -0.4, 0.0, 0.0
        ]

        self.pub.publish(msg)


def main():

    steps = int(
        input("Enter number of rotation steps: ")
    )

    direction = int(
        input(
            "Enter direction\n"
            "CCW=1\n"
            "CW=-1\n"
            ": "
        )
    )

    global DIRECTION

    DIRECTION = direction

    rclpy.init()

    node = RotateGait(steps)

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()