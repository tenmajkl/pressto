# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

"""Simple test for a standard servo on channel 0 and a continuous rotation servo on channel 1."""

import time

from adafruit_servokit import ServoKit

kit = ServoKit(channels=16)


# TODO calibration program to find the dead point

class Servo:

    def __init__(self, servo, dead_point):
        self.servo = servo
        self.dead_point = dead_point

    # todo compute the distance
    def move(self, direction, distance):
        if direction < self.dead_point:
            direction += self.dead_point
        self.servo.throttle = direction 
        time.sleep(distance/100)
        self.servo.throttle = self.dead_point


class Robot:
    # todo config
    def __init__(self, kit):
        self.kit = kit
        self.x_arm = Servo(kit.continuous_servo[0], 0.15)
        self.y_arm = Servo(kit.continuous_servo[1], 0)

    def up(self, distance):
        self.y_arm.move(-1, distance)

    def down(self, distance):
        self.y_arm.move(1, distance)

    def forward(self, distance):
        self.x_arm.move(-1, distance)

    def backward(self, distance):
        self.x_arm.move(1, distance)


rob = Robot(kit)

rob.up(20)
rob.forward(20)
rob.down(20)
rob.backward(20)
