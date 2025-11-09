from controller import Robot
from enum import Enum
from pathlib import Path
import numpy as np
import matplotlib
import matplotlib.image
import matplotlib.collections
import matplotlib.pyplot as plt
import cv2

# opgelet: veronderstelt een Python venv met numpy!


class RobotState(Enum):
    ORIENTING = 1  # weet bal liggen
    APPROACHING = 2  # bal is ongeveer centraal
    LOST_SIGHT = 3  # geen idee, dus spin
    ARRIVED = 4  # niets meer te doen


HUE_VAL = 0  # 0 is red
# HSV must all be at least this (on scale 360/255/255)
# but opencv color conversion produces H value between 0 and 180...
LOWER_COLOR = np.array([(HUE_VAL - 10) % 360, 100, 100])
UPPER_COLOR = np.array([(HUE_VAL + 10) % 360, 255, 255])
# zie https://cyberbotics.com/doc/reference/camera: 32 bits BGRA
RED_BGRA_PIXEL = bytes.fromhex("0000ffff")
# GREEN_BGRA_PIXEL = bytes.fromhex("00ff00ff")
# YELLOW_BGRA_PIXEL = bytes.fromhex("00f7ffff")
# BLUE_BGRA_PIXEL = bytes.fromhex("ff0000ff")
# MAGENTA_BGRA_PIXEL = bytes.fromhex("ff00ffff")
NEARRED_BGRA_PIXEL = bytes.fromhex("1000ffff")
IMG_WIDTH, IMG_HEIGHT = (4, 3)  # oorpsronkelijk 320x240

# 0 hue is red
# 30-ish is yellow
# 60-ish is green
# 120 is blue
# 150 is magenta
# so 180 will be red again
# but this seems inconsistent with docs

TIME_STEP = 64
robot = Robot()
robot.state = RobotState.ORIENTING
print(Path.cwd())

camera = robot.getDevice("camera")
leftMotor = robot.getDevice("left wheel motor")
rightMotor = robot.getDevice("right wheel motor")
# wat ook kan: gewoon vanaf start programma bijhouden vanaf 0
leftPositionSensor = robot.getDevice("left wheel sensor")
leftPositionSensor.enable(TIME_STEP)
rightPositionSensor = robot.getDevice("right wheel sensor")
rightPositionSensor.enable(TIME_STEP)
camera.enable(TIME_STEP)


def check_rgba_values(arr):
    plt.imshow(arr)
    plt.axis("off")
    plt.show()


def decide_state(current_state, image_bytes):
    # reference material assumes https://picamera.readthedocs.io/en/release-1.13/api_array.html#pirgbarray
    # denk dat r, g en b worden voorgesteld als bytes, niet als floats
    numpy_1_dim_bgra_byte_array = np.frombuffer(image_bytes, dtype=np.uint8)
    numpy_3_dim_bgra_byte_array = np.reshape(
        numpy_1_dim_bgra_byte_array, shape=(IMG_HEIGHT, IMG_WIDTH, 4), order="C"
    )
    numpy_3_dim_rgba_byte_array = numpy_3_dim_bgra_byte_array[..., [2, 1, 0, 3]]
    # check_rgba_values(numpy_3_dim_rgba_byte_array)

    # zie https://docs.opencv.org/3.4/d8/d01/group__imgproc__color__conversions.html
    # met 32-bit image zouden H, S en V al in juiste range moeten zijn
    hsv = cv2.cvtColor(numpy_3_dim_rgba_byte_array[..., :3], cv2.COLOR_RGB2HSV)
    restored = cv2.cvtColor(hsv, cv2.COLOR_HSV2RGB)
    check_rgba_values(restored)

    # dus print misschien heel hsv[...,[0]]?
    # dat zou een 4x3 moeten zijn met 3/4 groene hue en 1/4 rode
    print(hsv[..., [0]])

    color_mask = cv2.inRange(hsv, LOWER_COLOR, UPPER_COLOR)
    contours, _ = cv2.findContours(color_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    object_area = 0
    object_x = 0
    object_y = 0
    for contour in contours:
        x, y, width, height = cv2.boundingRect(contour)
        found_area = width * height
        center_x = x + width / 2
        center_y = y + height / 2
        if object_area < found_area:
            object_area = found_area
            object_x = center_x
            object_y = center_y
    if object_area > 0:
        ball_location = [object_area, object_x, object_y]
        print("Ik zie hem!")
    else:
        ball_location = None
        print("Ik zie hem niet!")
    # TODO: zoek grootste rode cluster in gezichtsveld
    # TODO: bepaal of die meer centraal moet
    match current_state:
        case RobotState.ARRIVED:
            return RobotState.ARRIVED
        case RobotState.APPROACHING:
            # TODO: wees milder, maar blijf monitoren
            return RobotState.APPROACHING
        case _:
            # TODO: analyseer beeld, kan elke state opleveren
            return RobotState.LOST_SIGHT


while robot.step(TIME_STEP) != -1:
    real_bytes = camera.getImage()
    # 3/4 groen en 1/4 rood, kan alvast checken dat verhouding klopt
    # formaat van camera.getImage zou BGRA (32 bits) moeten zijn
    # zie https://cyberbotics.com/doc/reference/camera
    fake_bytes = (NEARRED_BGRA_PIXEL * (IMG_WIDTH * IMG_HEIGHT * 3 // 4)) + (
        RED_BGRA_PIXEL * (IMG_WIDTH * IMG_HEIGHT // 4)
    )
    robot.state = decide_state(robot.state, fake_bytes)
    match robot.state:
        case RobotState.ORIENTING:
            pass
        case RobotState.APPROACHING:
            leftMotor.setPosition(leftPositionSensor.getValue() + 1)
            rightMotor.setPosition(rightPositionSensor.getValue() + 1)
        case RobotState.LOST_SIGHT:
            rightMotor.setPosition(rightPositionSensor.getValue() + 1)
        case RobotState.ARRIVED:
            print("Ik sta hier goed.")
