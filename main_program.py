# -*- coding: utf-8 -*-
"""
Creation: 2020 01 18 14:00
Authors: Guillaume COUZINET & Elias NEUMAIER
Context: Computer Sciences for Engineers -- TEC Reutlingen Hochschule
"""


import ev3dev.ev3 as ev3
import math
import logging
import time
from statistics import mean

logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.DEBUG)
logging.info("Program : START")

'''
-------------------------------------------------------------------------------
------ CONSTANTS AND GLOBALS --------------------------------------------------
-------------------------------------------------------------------------------
'''
# ----- CONSTANTS ------

# Time of the challenge (in second)
CHALLENGE_TIME = 300
TIME_TO_FIND_DIAMONDS = 180

# Set the colours of the elements
RINGS_COLOURS = [4, 5]
LIMIT_BAND_COLOUR = 3
CAVE_COLOUR = 6
GROUND_COLOUR = 1

# ----- Function : is_distance_ok
# DISTANCE_MIN is the minimum permissible distance (in millimetres)
DISTANCE_MIN = 150
TOLERANCE = 10


# ------ GLOBAL VARIABLES ------

# Motors definition
MotorRight = ev3.LargeMotor('outB')
MotorLeft = ev3.LargeMotor('outA')

# Distance sensor definition
UltraSonic = ev3.UltrasonicSensor('in2')

# Put the US sensor into distance mode.
#   <!> The sensor return values in millimetres <!>
UltraSonic.mode='US-DIST-CM'

# -- Gyroscope --
Gyro = ev3.GyroSensor('in4')
Gyro.mode = 'GYRO-ANG'


# Definition of colour sensor and setting the mode
ColSensor = ev3.ColorSensor('in1')
ColSensor.mode = 'COL-COLOR'
# inputs of the different colours in 'COL-COLOR'-mode
# 0: No color   1: Black   2: Blue   3: Green   4: Yellow   5: Red   6: White
# 7: Brown

# Medium motor (needed to open the arm) definition
ArmMotor = ev3.MediumMotor('outD')

# Button sensor (needed to start the challenge)
Touch = ev3.TouchSensor('in3')

# ObstacleCounter is the obstacle counter
ObstacleCounter = 0


'''
-------------------------------------------------------------------------------
------ MATHS ------------------------------------------------------------------
-------------------------------------------------------------------------------
'''

def angleCalculation(distance, RADIUS=28):
    """
    Function calculating the angle of rotation with a distance
        and the radius of the wheels.

    distance -- the value of the distance to reach in millimeter
    RETURN -- angle of rotation in degres
    """
    return (distance * 360) / (2 * math.pi * RADIUS)


'''
-------------------------------------------------------------------------------
------ MOTORS -----------------------------------------------------------------
-------------------------------------------------------------------------------
'''

def start_motors_forever(speed):
    '''
    Function giving the start signal to the motors with a
        given operating speed.

    speed -- value between 100 and 900 indicating the rate of the maximum
        possible speed to reach.
    '''
    global MotorRight, MotorLeft
    MotorRight.run_forever(speed_sp=speed)
    MotorLeft.run_forever(speed_sp=speed)



def stop_motors(action='brake'):
    '''
    Function that sends the stop signal to the motors.

    action -- value indicating the stopping method to be used by the motors.
        Refer to the official EV3 documentation.
    '''
    global MotorRight, MotorLeft
    MotorRight.stop(stop_action=action)
    MotorLeft.stop(stop_action=action)


def rotate_motors_right():
    MotorLeft.run_forever(speed_sp=100)
    MotorRight.run_forever(speed_sp=-100)


def rotate_motors_left():
    MotorLeft.run_forever(speed_sp=-100)
    MotorRight.run_forever(speed_sp=100)


'''
-------------------------------------------------------------------------------
------ PATH -------------------------------------------------------------------
-------------------------------------------------------------------------------
'''


def driveForward(distance, speed=500):
    """
    Function to drive the robot straight forward for a given distance

    distance -- the value of the distance to reach in millimeter
    """
    global MotorRight, MotorLeft

    MotorRight.run_to_rel_pos(position_sp=angleCalculation(distance),
        speed_sp=speed)
    MotorLeft.run_to_rel_pos(position_sp=angleCalculation(distance),
        speed_sp=speed)

    logging.info('DriveForward : Start to drive')
    MotorRight.wait_while('running')
    MotorLeft.wait_while('running')
    logging.info('DriveForward : Stop')


def rotate(angle):
    """
    Function rotating clockwisely the robot to a given angle.
        We want use the smallest angle possible.
        That is why we use the modulo operator.
        Then we calculate the difference between the angle before moving and
        the angle during the movement.

    angle -- the angle (negative or positive) of rotation to reach in degres
    """
    global MotorRight, MotorLeft, Gyro

    # "angleTarget" is the variable geting the rest of the
    #   angle divided by 360 degrees
    angleTarget = 0
    # "angleOld" is the angle measured by the GyroscopSensor before moving.
    angleOld = Gyro.value()

    if (angle % 360 <= 180):
        # If the rest is smaller than 180 degrees, we turning on the right.
        angleTarget = angle % 360
        rotate_motors_right()
    else:
        # If the rest is bigger than 180 degrees, we turning on the left.
        # Then we calculate the right angle.
        angleTarget = 360 - (angle % 360)
        rotate_motors_left()

    # FLAG boolean
    angleReached = False
    logging.info('Rotation : Rotate until the target angle is reached')
    while not angleReached:
        if abs(angleOld - Gyro.value()) >= angleTarget:
            angleReached = True
            stop_motors()
            logging.info('Rotation : Stop')



'''
-------------------------------------------------------------------------------
------ FUNCTIONS --------------------------------------------------------------
-------------------------------------------------------------------------------
'''

def is_distance_ok():
    '''
    Function that compares the distance between the robot and the nearest
        object facing it. If this object is below the minimum distance value,
        the stop signal is sent to the motors.

    RETURN -- Boolean indicating if the robot is too close of an obstacle.
    '''
    distanceOK = True

    distance = UltraSonic.value()
    if distance <= DISTANCE_MIN:
        distanceOK = False

    return distanceOK


def circle_found(ringColour):
    '''
    Function setting up the robot in position before do the circle.

    ringColor -- is the color of the detected circle.
    '''
    logging.info('CAR : START')

    logging.info('CAR : Rotate to find the ground')
    rotate_motors_right()
    reachGroundColor = False

    while not reachGroundColor:
        if ColSensor.value() == GROUND_COLOUR:
            logging.info('CAR : Ground found !')
            reachGroundColor = True
            stop_motors()

    driveAround(ringColour)
    logging.info('CAR : END')


def driveAround(ringColour):
    '''
    Function that determines the rotational speed of the right
        motor by dichotomy.

    ringColor -- is the color of the detected circle.
    '''
    logging.info('Circle : START')
    global MotorRight, MotorLeft

    minSpeed = 300
    maxSpeed = 500

    logging.info('Circle : Start motors')
    MotorLeft.run_forever(speed_sp=minSpeed)
    MotorRight.run_forever(speed_sp=maxSpeed)

    logging.info('Circle : Sleep for 7s during the drive')
    time.sleep(7)
    logging.info('Circle : Stop motors')
    stop_motors()
    logging.info('Circle : END')


def open_grab_close():
    logging.info('OGC : START')
    logging.info('OGC : Going into the circle')
    # Go back
    driveForward(-150, speed=500)

    # Open the arm
    logging.info('OGC : Open arm')
    ArmMotor.run_timed(time_sp=3500, speed_sp=-900)
    ArmMotor.wait_while('running')

    # Drive in the middle of the circle
    logging.info('OGC : Grab the diamond')
    driveForward(300, speed=500)

    # Close the arm
    logging.info('OGC : Close arm')
    ArmMotor.run_timed(time_sp=3500, speed_sp=900)
    ArmMotor.wait_while('running')

    # Drive away forward
    logging.info('OGC : Going away')
    driveForward(250, speed=500)
    logging.info('OGC : END')


'''
-------------------------------------------------------------------------------
------ MAIN LOOP --------------------------------------------------------------
-------------------------------------------------------------------------------
'''

logging.info('Program : Robot ready, waiting...')
while Touch.value() == 0:
    time.sleep(0.01)
    logging.info('Program : Button pressed !')

startTime = time.time()
timeToDiamonds = True
inTheCave = False
loopCounter = 0

logging.info('Program : Starting Main Loop !')

while ( time.time() - startTime ) <= CHALLENGE_TIME:

    if loopCounter > 90:
        loopCounter = 0
    else:
        loopCounter = loopCounter + 1

    if (time.time() - startTime) > TIME_TO_FIND_DIAMONDS:
        logging.info('Main : No time for diamonds')
        timeToDiamonds = False

    if not inTheCave:
        start_motors_forever()

    # Check the distance to be sure that there is no obstacle
    #   or robot ahead and bypass if there is something
    if not is_distance_ok():
        logging.info('Main : Obstacle detected')
        stop_motors()
        driveForward(distance=-200)
        rotate( (90 + loopCounter) * (math.pow(-1, loopCounter)))

    # Check the colour, if its the colour of the bands turn around
    if ColSensor.value() == LIMIT_BAND_COLOUR:
        logging.info('Main : Limit band detected')
        stop_motors()
        driveForward(distance=-200)
        rotate( (90 + loopCounter) * (math.pow(-1, loopCounter)))

    # Check the colour, if its one of the diamonds, circle the diamond
    #   and collect it
    if ColSensor.value() in RINGS_COLOURS and timeToDiamonds:
        logging.info('Main : Circle detected')
        colour = ColSensor.value()
        stop_motors()
        circle_found(colour)
        rotate(-90)
        open_grab_close()

    # Check the colour is a cave and if the time is up to go in a cave
    if ColSensor.value() == CAVE_COLOUR and not timeToDiamonds:
        logging.info('Main : Cave detected')
        stop_motors()
        driveForward(150)
        inTheCave = True

logging.info('Program : Stop motor, time is up, challenge ended')
stop_motors()

logging.info('Program : END')
