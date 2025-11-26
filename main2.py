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
ev3.speaker.beep()

#sensors함수
left_sensor = ColorSensor(Port.S1)
right_sensor = ColorSensor(Port.S4)
object_detector = ColorSensor(Port.S2)
ultra_sensor = UltrasonicSensor(Port.S3)

#motors
grab_motor = Motor(Port.C)
left_motor = Motor(Port.A)
right_motor = Motor(Port.D)

#drive base
robot = DriveBase(left_motor, right_motor, wheel_diameter = 56, axle_track = 114)

# 왼쪽 센서 라인트레이스
def left_line_following(speed, kp):
    threshold = 50
    left_reflection = left_sensor.reflection()
    error = left_reflection - threshold
    turn_rate = kp *error
    robot.drive(speed, turn_rate)

#오른쪽 센서 라인트레이스
def right_line_following(speed, kp):
    threshold = 50
    right_reflection =right_sensor.reflection()
    error = right_reflection - threshold
    turn_rate = -kp *error
    robot.drive(speed, turn_rate)

#n 칸 주행하는 함수
def n_move(n, direction):
    for _ in range(n):
        if direction =="right":
            while right_sensor.reflection() > 18:
                left_line_following(100, 1.2)
            while right_sensor.reflection() <= 18:
                right_line_following(100, 1.2)
        elif direction == "left":
            while left_sensor.reflection() > 18:
                right_line_following(100, 1.2)
            while left_sensor.reflection() <= 18:
                left_line_following(100, 1.2)
    robot.stop()


#Release
def release_object():
    grab_motor.run_until_stalled(-200, then = Stop.COAST, duty_limit=50)

#Grab
def grab_object():
    grab_motor.run_until_stalled(200, then = Stop.COAST, duty_limit=50)


#--------주행 코드--------
distance = ultra_sensor.distance()
release_object()

#시작점에서 나오기
robot.straight(100)
n_move(1, "left")
while ultra_sensor.distance() > 30:
    left_line_following(100, 1.2)
print(distance)
ev3.speaker.beep()
robot.stop()
#물체 잡기
grab_object()
wait(500)
object_color = object_detector.color()
print("Detected object color:", object_color)

#RGB 분석


#red 물체 처리
if object_color == Color.RED:
    robot.straight(50)
    robot.turn(168)
    n_move(1, "left")
    robot.straight(60)
    robot.turn(-80)
    n_move(1, "right")
    robot.stop()
    robot.turn(90)
    ev3.speaker.beep()
    n_move(1, "right")
    robot.stop()
    release_object()
#blue 물체 처리
elif object_color == Color.BLUE:
    robot.turn(168)
    n_move(1, "right")
    robot.turn(-80)
    n_move(2, "left")
    robot.turn(90)
    n_move(1, "left")
    release_object()
