from controller import Robot
import sys
import evdev
import math


TIME_STEP = 64
robot = Robot()
leftMotor = robot.getDevice("left wheel motor")
rightMotor = robot.getDevice("right wheel motor")
leftPositionSensor = robot.getDevice("left wheel sensor")
leftPositionSensor.enable(TIME_STEP)
rightPositionSensor = robot.getDevice("right wheel sensor")
rightPositionSensor.enable(TIME_STEP)
EPSILON = 0.0001
MIN_R_FOR_MOVEMENT = 0.5  # stick moet al wat ingedrukt zijn

if len(sys.argv) != 2:
    print(
        "Er moet exact één argument zijn, namelijk het devicenummer van de gamepad. Check dat via `evtest`."
    )
    sys.exit(1)

device_number = int(sys.argv[1])
device = evdev.InputDevice(f"/dev/input/event{device_number}")

while robot.step(TIME_STEP) != -1:
    abs_info_axis_0 = device.absinfo(0).value
    abs_info_axis_1 = device.absinfo(1).value
    # krijgen horizontale/verticale waarde tussen -1 en 1
    # waarden stick zijn op voorhand gecontroleerd
    horizontal = max(
        min(abs_info_axis_0 / (32768.0 if abs_info_axis_0 < 0 else 32767.0), 1), -1
    )
    vertical = -max(
        min(abs_info_axis_1 / (32768.0 if abs_info_axis_1 < 0 else 32767.0), 1), -1
    )
    r = math.sqrt(horizontal**2 + vertical**2)
    theta = None
    if r >= MIN_R_FOR_MOVEMENT:
        if abs(horizontal) < EPSILON:
            if vertical > EPSILON:
                theta = math.pi / 2
            elif vertical < -EPSILON:
                theta = 3 * math.pi / 2
        else:
            # atan2 houdt rekening met kwadrant, atan niet
            # resultaat is tussen -pi en pi, ik pas aan naar tussen 0 en 2pi
            theta = math.atan2(vertical, horizontal)
            if theta < 0:
                theta += math.pi * 2
        if theta:
            # ik ben nog niet helemaal zeker hoe goed dit zal werken met een echte stappenmotor
            # kan die met fracties van stappen werken?
            adjustment_left = math.sin(theta + math.pi / 4)
            adjustment_right = math.sin(theta - math.pi / 4)
            leftMotor.setPosition(leftPositionSensor.getValue() + adjustment_left)
            rightMotor.setPosition(rightPositionSensor.getValue() + adjustment_right)
    else:
        leftMotor.setPosition(leftPositionSensor.getValue())
        rightMotor.setPosition(rightPositionSensor.getValue())
