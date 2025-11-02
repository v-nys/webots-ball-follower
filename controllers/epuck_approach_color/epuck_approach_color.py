from controller import Robot, Motor
from enum import Enum
import sys
from pathlib import Path
import numpy
import matplotlib
import matplotlib.image
import matplotlib.collections
import matplotlib.pyplot

# opgelet: veronderstelt een Python venv met numpy!

class RobotState(Enum):
    ORIENTING = 1 # weet bal liggen
    APPROACHING = 2 # bal is ongeveer centraal
    LOST_SIGHT = 3 # geen idee, dus spin
    ARRIVED = 4 # niets meer te doen

TIME_STEP = 64
robot = Robot()
robot.state = RobotState.ORIENTING
print(Path.cwd())

camera = robot.getDevice('camera')
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
# wat ook kan: gewoon vanaf start programma bijhouden vanaf 0
leftPositionSensor = robot.getDevice('left wheel sensor')
leftPositionSensor.enable(TIME_STEP)
rightPositionSensor = robot.getDevice('right wheel sensor')
rightPositionSensor.enable(TIME_STEP)
camera.enable(TIME_STEP)

def decide_state(current_state, image_bytes):
    numpy_1_dim_bgra_byte_array = numpy.frombuffer(image_bytes, dtype=numpy.uint8)
    numpy_3_dim_bgra_byte_array = numpy.reshape(numpy_1_dim_bgra_byte_array, shape=(240, 320, 4), order='C')
    numpy_3_dim_rgba_byte_array = numpy_3_dim_bgra_byte_array[..., [2, 1, 0, 3]]
    numpy_3_dim_rgb_float_array = numpy_3_dim_rgba_byte_array[..., :3] / 255.0
    numpy_3_dim_hue_angle_array = matplotlib.colors.rgb_to_hsv(numpy_3_dim_rgb_float_array)[..., :1].squeeze() * 360.0
    flattened_hues = numpy_3_dim_hue_angle_array.flatten()
    # print(flattened_hues)
    number_of_red = len([hue for hue in flattened_hues if hue <= 10 or hue >= 350])
    red_ratio = number_of_red / len(flattened_hues)
    if red_ratio >= 0.02:
        print("Ik zie iets rood!")
    else:
        print("Ik zie niets!")
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
    robot.state = decide_state(robot.state, camera.getImage())
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