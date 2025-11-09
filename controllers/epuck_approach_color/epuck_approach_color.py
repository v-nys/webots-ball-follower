from controller import Robot, Motor
from enum import Enum
from pathlib import Path
import numpy as np
import matplotlib
import matplotlib.image
import matplotlib.collections
import matplotlib.pyplot
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
RED_BGRA_PIXEL = bytes.fromhex("0000ffff")
GREEN_BGRA_PIXEL = bytes.fromhex("00ff00ff")
IMG_WIDTH, IMG_HEIGHT = (320, 240)

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


def decide_state(current_state, image_bytes):
    # reference material assumes https://picamera.readthedocs.io/en/release-1.13/api_array.html#pirgbarray
    # denk dat r, g en b worden voorgesteld als bytes, niet als floats
    numpy_1_dim_bgra_byte_array = np.frombuffer(image_bytes, dtype=np.uint8)
    numpy_3_dim_bgra_byte_array = np.reshape(
        numpy_1_dim_bgra_byte_array, shape=(IMG_HEIGHT, IMG_WIDTH, 4), order="C"
    )
    numpy_3_dim_rgba_byte_array = numpy_3_dim_bgra_byte_array[..., [2, 1, 0, 3]]

    # zie https://docs.opencv.org/3.4/d8/d01/group__imgproc__color__conversions.html
    # FIXME: probleem in volgende drie regels?
    # kloppen HSV-waarden? klopt color_mask? contours is altijd leeg...
    # heb hier wel BGR2HSV, niet RGB2HSV
    # LET OP! BGR2HSV geeft H-waarde tussen 0 en 180, niet 0 en 360
    # en BGR2HSV_FULL geeft H-waarde tussen 0 en 255
    # iets met range tot 360 is er niet...
    hsv = cv2.cvtColor(numpy_3_dim_rgba_byte_array[..., :3], cv2.COLOR_RGB2HSV_FULL)
    # nu heb ik een driedimensionale array van afmetingen IMG_HEIGHT, IMG_WIDTH, 3
    # en ik wil elke eerste waarde van die 3 omzetten van schaal tot 255 naar schaal tot 360
    # dus alles op één "plane" wil ik * 360 / 255
    # met andere woorden: hsv[alle_rijen, alle_kolommen, 0] = hsv[alle_rijen, alle_kommen, 0] * 360 / 255
    # dimensionaliteit van de array (positie + waarde) en van de data (hsv-waarde) is 3, maar dat kan verschillen!

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
    fake_bytes = (GREEN_BGRA_PIXEL * (IMG_WIDTH * IMG_HEIGHT * 3 // 4)) + (
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
