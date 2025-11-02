from controller import Robot, Motor

TIME_STEP = 64
robot = Robot()

camera = robot.getDevice('camera')
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
camera.enable(TIME_STEP)

# set the target position of the motors
# leftMotor.setPosition(10.0)
# rightMotor.setPosition(10.0)

while robot.step(TIME_STEP) != -1:
   pass