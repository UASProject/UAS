from UAS_Functions import lightbulb_detach, turnOffMotors,servo_close,GetBlocks,arm_and_takeoff
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor, Adafruit_StepperMotor
from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import atexit
import RPi.GPIO as GPIO
from pixy import *
from ctypes import *

# Initialize Pixy Interpreter thread #
pixy_init()

<<<<<<< HEAD
while 1:
	x,y=GetBlocks()

=======
import argparse
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect',
                    help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect

# Connect to the Vehicle
'''
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)
'''
#arm motors and takeoff to set altitude
'''
#arm_and_takeoff(aTargetAltitude)
vehicle.airspeed = 3 #set default speed
'''
#get blocks data from pixy
'''
x,y=GetBlocks()
print(x)
print(y)
'''
>>>>>>> c2f49b347046e9663947edd00dbd775ad6594f45
#servo_close()

#lightbulb_detach()

