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


# 0 hue is rood
# 30 is geel
# 60-ish is groen
# 90 is appelblauw
# 120 is blauw
# 150 is magenta
# 180 is terug rood
HUE_VAL = 0
assert 0 <= HUE_VAL < 180
HUE_OFFSET = 15
assert HUE_OFFSET >= 0 and HUE_OFFSET < 180, "geen zinvolle offsethoek"
# geen modulo 180, want dan zou inRange (zie beneden) niet werken
# zullen gecontroleerde hues moeten aanpassen
LOWER_COLOR = np.array([(HUE_VAL - HUE_OFFSET), 50, 50])
UPPER_COLOR = np.array([(HUE_VAL + HUE_OFFSET), 255, 255])
# zie https://cyberbotics.com/doc/reference/camera: 32 bits totaal
# dus "8-bit" afbeelding (see https://www.tourboxtech.com/en/news/bit-depth-explained.html)
RED_BGRA_PIXEL = bytes.fromhex("0000ffff")
GREEN_BGRA_PIXEL = bytes.fromhex("00ff00ff")
IMG_WIDTH, IMG_HEIGHT = (320, 240)  # (4, 3)  # oorpsronkelijk 320x240 genomen
TIME_STEP = 64
robot = Robot()
robot.state = RobotState.ORIENTING

camera = robot.getDevice("camera")
leftMotor = robot.getDevice("left wheel motor")
rightMotor = robot.getDevice("right wheel motor")
# wat ook kan: gewoon vanaf start programma tel bijhouden
leftPositionSensor = robot.getDevice("left wheel sensor")
leftPositionSensor.enable(TIME_STEP)
rightPositionSensor = robot.getDevice("right wheel sensor")
rightPositionSensor.enable(TIME_STEP)
camera.enable(TIME_STEP)


def decide_state(current_state, image_bytes):
    # boek gebruikt https://picamera.readthedocs.io/en/release-1.13/api_array.html#pirgbarray
    # staat eigenlijk niet hoe veel bits per kleurkanaal er dan zijn, vermoedelijk 8
    numpy_1_dim_bgra_byte_array = np.frombuffer(image_bytes, dtype=np.uint8)
    numpy_3_dim_bgra_byte_array = np.reshape(
        numpy_1_dim_bgra_byte_array, shape=(IMG_HEIGHT, IMG_WIDTH, 4), order="C"
    )
    numpy_3_dim_rgba_byte_array = numpy_3_dim_bgra_byte_array[..., [2, 1, 0, 3]]

    # zie https://docs.opencv.org/3.4/d8/d01/group__imgproc__color__conversions.html
    # met 8-bit afbeelding en COLOR_RGB2HSV zou H tussen 0 en 180 moeten liggen
    hsv = cv2.cvtColor(numpy_3_dim_rgba_byte_array[..., :3], cv2.COLOR_RGB2HSV)
    h = hsv[..., 0]
    # schuif waarden die niet vergelijkbaar zijn met grens op in de richting van grens
    shift_down_mask = np.abs(h - LOWER_COLOR[0]) >= 180
    # analoge redenering
    shift_up_mask = np.abs(h - UPPER_COLOR[0]) >= 180
    h_new = h.copy()
    h_new[shift_down_mask] = h_new[shift_down_mask] - 180
    h_new[shift_up_mask] = h_new[shift_up_mask] + 180
    hsv[..., 0] = h_new
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
        print(f"Ik zie hem op x-positie {ball_location[1]}!")
    else:
        ball_location = None
        print("Ik zie hem niet!")
    # TODO: bepaal wat er moet gebeuren
    # indien we aan het oriënteren zijn, moet hij op IMG_WIDTH+-FOCUS_MARGIN komen
    # indien we een het naderen zijn, mag hij wat off-center zijn, bv. IMG_WIDTH+-2FOCUS_MARGIN
    # indien object_area heel groot is, zitten we er tegen...
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
    # "real" want heb dit ook getest met manueel aangemaakte "fake" bytes
    real_bytes = camera.getImage()
    robot.state = decide_state(robot.state, real_bytes)
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
