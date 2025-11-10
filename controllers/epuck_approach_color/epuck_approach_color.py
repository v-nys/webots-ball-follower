from controller import Robot
from enum import Enum
import numpy as np
import cv2
import sys

# opgelet: veronderstelt dat NumPy en OpenCV bruikbaar zijn uit Python!


class RobotState(Enum):
    ORIENTING_COUNTERCLOCKWISE = 1  # bal gezien, ligt links van centrum
    ORIENTING_CLOCKWISE = 2  # bal gezien, ligt rechts van centrum
    APPROACHING = 3  # bal is ongeveer centraal, rijden vooruit
    LOST_SIGHT = 4  # geen idee, dus spin
    ARRIVED = 5  # niets meer te doen


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
IMG_WIDTH, IMG_HEIGHT = (320, 240)  # moet matchen met setting camera in Webots!
ALLOWED_DEVIATION_PERCENTAGE_ORIENTING = 10.0
ALLOWED_ABSOLUTE_DEVIATION_ORIENTING = IMG_WIDTH * (
    ALLOWED_DEVIATION_PERCENTAGE_ORIENTING / 100
)
ALLOWED_DEVIATION_PERCENTAGE_APPROACHING = ALLOWED_DEVIATION_PERCENTAGE_ORIENTING * 2
ALLOWED_ABSOLUTE_DEVIATION_APPROACHING = IMG_WIDTH * (
    ALLOWED_DEVIATION_PERCENTAGE_APPROACHING / 100
)
STOP_APPROACHING_AT_PERCENT = 30.0
REQUIRED_SCREEN_COVERAGE_IN_PIXELS = (
    IMG_WIDTH * IMG_HEIGHT * STOP_APPROACHING_AT_PERCENT / 100.0
)
TIME_STEP = 64
robot = Robot()
robot.state = RobotState.LOST_SIGHT
camera = robot.getDevice("camera")
leftMotor = robot.getDevice("left wheel motor")
rightMotor = robot.getDevice("right wheel motor")
# wat ook kan: gewoon vanaf start programma tel bijhouden
leftPositionSensor = robot.getDevice("left wheel sensor")
leftPositionSensor.enable(TIME_STEP)
rightPositionSensor = robot.getDevice("right wheel sensor")
rightPositionSensor.enable(TIME_STEP)
camera.enable(TIME_STEP)


def get_ball_area_and_coordinates(image_bytes):
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
    if object_area:
        return (object_area, object_x, object_y)
    else:
        return (None, None, None)


def decide_state(current_state, image_bytes):
    (ball_screen_area, ball_x, ball_y) = get_ball_area_and_coordinates(image_bytes)
    if not ball_screen_area:
        return RobotState.LOST_SIGHT
    deviation = ball_x - IMG_WIDTH / 2
    match current_state:
        case RobotState.ARRIVED:
            return RobotState.ARRIVED
        case (
            RobotState.ORIENTING_CLOCKWISE
            | RobotState.ORIENTING_COUNTERCLOCKWISE
            | RobotState.LOST_SIGHT
        ):
            if abs(deviation) < ALLOWED_ABSOLUTE_DEVIATION_ORIENTING:
                return RobotState.APPROACHING
            elif deviation < 0:
                return RobotState.ORIENTING_COUNTERCLOCKWISE
            else:
                return RobotState.ORIENTING_CLOCKWISE
        case RobotState.APPROACHING:
            if abs(deviation) < ALLOWED_DEVIATION_PERCENTAGE_APPROACHING:
                if ball_screen_area >= REQUIRED_SCREEN_COVERAGE_IN_PIXELS:
                    return RobotState.ARRIVED
                return RobotState.APPROACHING
            elif deviation < 0:
                return RobotState.ORIENTING_COUNTERCLOCKWISE
            else:
                return RobotState.ORIENTING_CLOCKWISE


while robot.step(TIME_STEP) != -1:
    # "real" want heb dit ook getest met manueel aangemaakte "fake" bytes
    real_bytes = camera.getImage()
    robot.state = decide_state(robot.state, real_bytes)
    match robot.state:
        case RobotState.ORIENTING_COUNTERCLOCKWISE:
            rightMotor.setPosition(rightPositionSensor.getValue() + 1)
        case RobotState.ORIENTING_CLOCKWISE:
            leftMotor.setPosition(leftPositionSensor.getValue() + 1)
        case RobotState.APPROACHING:
            leftMotor.setPosition(leftPositionSensor.getValue() + 1)
            rightMotor.setPosition(rightPositionSensor.getValue() + 1)
        case RobotState.LOST_SIGHT:
            # arbitraire keuze om dan tegen de klok in te draaien
            rightMotor.setPosition(rightPositionSensor.getValue() + 1)
        case RobotState.ARRIVED:
            print("Ik sta hier goed.")
            sys.exit(0)
