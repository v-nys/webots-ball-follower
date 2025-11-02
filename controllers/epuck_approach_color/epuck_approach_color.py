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
    numpy_1_dim_rgba_byte_array = numpy.frombuffer(image_bytes, dtype=numpy.uint8)
    numpy_3_dim_rgba_byte_array = numpy.reshape(numpy_1_dim_rgba_byte_array, shape=(240, 320, 4), order='C')
    # TODO: eerste check: converteer terug naar opgeslagen afbeelding, is ze identiek aan camerabeeld?
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