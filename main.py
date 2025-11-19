#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()


# Write your program here.

left_motor = Motor(Port.A)
right_motor = Motor(Port.D)
cs = ColorSensor(Port.S1)
cs2 = ColorSensor(Port.S4)
robot = DriveBase(left_motor, right_motor, 55.5, 104)
threshold = 18
kp = 1.2
now_dir1 = 1
now_dir2 = 4
now_dir3 = 2
target_dir1 =4
target_dir2 =1
target_dir3 =2
ev3.speaker.beep()
while True:
    left_reflection = cs.reflection()
    right_reflection = cs2.reflection()
    if left_reflection <30:
        robot.stop()
        break
    else:
        error = right_reflection - threshold
        turn_rate = kp*error
        robot.drive(100, turn_rate)
    wait(10)
robot.straight(5)
direction = (target_dir1 - now_dir1) % 4
turn_table = [0, 90, 180, -90]
angle = turn_table[direction]
robot.turn(angle)
wait(10)
for i in range(2):
    while True:
        left_reflection = cs.reflection()
        right_reflection = cs2.reflection()
        if left_reflection <30:
            robot.stop()
            break
        else:
            error = right_reflection - threshold
            turn_rate = kp*error
            robot.drive(100, turn_rate)
        wait(10)
    while True:
        left_reflection = cs.reflection()
        right_reflection = cs2.reflection()
        if left_reflection > 30:
            robot.stop()
            break
        else:
            error = right_reflection - threshold
            turn_rate = kp*error
            robot.drive(100, turn_rate)
        wait(10)
direction = (target_dir2 - now_dir2) % 4
turn_table = [0, 90, 180, -90]
angle = turn_table[direction]
robot.turn(angle)
robot.straight(10)
wait(10)
#2칸 전진        
for i in range(2):
    while True:
        left_reflection = cs.reflection()
        right_reflection = cs2.reflection()
        if right_reflection <30:
            robot.stop()
            break
        else:
            error = left_reflection - threshold
            turn_rate = kp*error
            robot.drive(100, turn_rate)
        wait(10)
    while True:
        left_reflection = cs.reflection()
        right_reflection = cs2.reflection()
        if right_reflection > 30:
            robot.stop()
            break
        else:
            error = left_reflection - threshold
            turn_rate = kp*error
            robot.drive(100, turn_rate)
        wait(10)
direction = (target_dir3 - now_dir1) % 4
turn_table = [0, 90, 180, -90]
angle = turn_table[direction]
robot.turn(angle)
wait(10)
#2칸 전진
for i in range(2):
    while True:
        left_reflection = cs.reflection()
        right_reflection = cs2.reflection()
        if right_reflection <30:
            robot.stop()
            break
        else:
            error = left_reflection - threshold
            turn_rate = kp*error
            robot.drive(100, turn_rate)
        wait(10)
    while True:
        left_reflection = cs.reflection()
        right_reflection = cs2.reflection()
        if right_reflection > 30:
            robot.stop()
            break
        else:
            error = left_reflection - threshold
            turn_rate = kp*error
            robot.drive(100, turn_rate)
        wait(10)

# direction = (target_dir2 - now_dir1) % 4
# turn_table = [0, 90, 180, -90]
# angle = turn_table[direction]
# robot.turn(angle)
# ev3.speaker.beep()
# wait(10000)
# print("debug")
# for i in range(2):
#     while True:
#         left_reflection = cs.reflection()
#         right_reflection = cs2.reflection()
#         if right_reflection <30:
#             robot.stop()
#             break
#         else:
#             error = left_reflection - threshold
#             turn_rate = kp*error
#             robot.drive(100, turn_rate)
#         wait(10)
#     while True:
#         left_reflection = cs.reflection()
#         right_reflection = cs2.reflection()
#         if right_reflection > 30:
#             robot.stop()
#             break
#         else:
#             error = left_reflection - threshold
#             turn_rate = kp*error
#             robot.drive(100, turn_rate)
#         wait(10)
robot.straight(10)
ev3.speaker.beep()
direction = (target_dir2 - now_dir3) % 4
turn_table = [0, 90, 180, -90]
angle = turn_table[direction]
robot.turn(angle)
robot.straight(20)
while True:
    left_reflection = cs.reflection()
    right_reflection = cs2.reflection()
    if left_reflection <30:
        robot.stop()
        break
    else:
        error = right_reflection - threshold
        turn_rate = kp*error
        robot.drive(100, turn_rate)
    wait(10)